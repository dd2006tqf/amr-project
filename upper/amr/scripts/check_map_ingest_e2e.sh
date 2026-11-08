#!/usr/bin/env bash
# ==============================================================================
# check_map_ingest_e2e.sh
#
# 端到端自动化验收流水线 (Task 9 - demo_warehouse 地图导入与调度闭环验收)
#
# 验证链路:
#   Stage 1: ROS 2 nav2_map_server 发布 demo_warehouse (L型双向走廊, 宽 1.2m)
#   Stage 2: map_ingest_node 消费 /map 与 markers.yaml, 输出 config/generated/stations.yaml
#            - 校验 station_pickup_A 与 station_dropoff_B 拓扑就绪
#            - 校验 road_nodes >= 1, 关键边包含真实折线 (polyline)
#            - 校验 map_hash 非空且格式为 fnv1a64_*
#   Stage 3: 启动 dispatcher_node (加载新 stations.yaml, planner_type:=skeleton,
#            expected_map_hash 钉死), 启动 path_tracker_node + 航向/里程计仿真闭环,
#            启动 rest_gateway 接入网关
#   Stage 4: 发起端到端业务订单 (REST Gateway POST /api/v1/orders)
#            - 校验 SkeletonPlanner 规划出真实沿 L 走廊绕行的折线路径 (无穿墙)
#            - 校验 /v2/list_traffic_reservations 成功占用全航线拓扑锁 (owner=e2e_m1)
#            - 闭环仿真车辆沿路径循迹到达终点, 验证 CompleteMission "arrived at goal"
#
# 退出代码:
#   0: 验收全项通过
#   1: 任一断言失败, 附带详尽日志 dump, 严禁私自降低断言
# ==============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

# ------------------------------------------------------------------------------
# 宿主机 / 容器自动适配: 若宿主机无 ros2 环境, 自动转发至 amr_dispatcher 容器
# ------------------------------------------------------------------------------
if [[ -z "${AMR_E2E_INNER:-}" ]]; then
  if ! command -v ros2 >/dev/null 2>&1; then
    if docker ps --format '{{.Names}}' | grep -q "^amr_dispatcher$"; then
      echo "[INFO] 宿主机未检测到 ros2 命令, 转发执行至 amr_dispatcher 容器内部..."
      exec docker exec -i -e AMR_E2E_INNER=1 -w /workspace amr_dispatcher bash /workspace/scripts/check_map_ingest_e2e.sh "$@"
    else
      echo "[ERROR] 本机无 ros2 且 amr_dispatcher 容器未运行, 无法执行端到端验收" >&2
      exit 1
    fi
  fi
fi

# ------------------------------------------------------------------------------
# 容器内 ROS 2 环境初始化
# 注意: colcon 生成的 setup.bash 中引用了未设置的 COLCON_TRACE 变量,
# 在 set -u 模式下会报错, 因此在 source 阶段临时关闭 -u。
# ------------------------------------------------------------------------------
set +u
if [[ -z "${ROS_DISTRO:-}" && -f /opt/ros/jazzy/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
fi
if [[ -f "${WORKSPACE_DIR}/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${WORKSPACE_DIR}/install/setup.bash"
fi
set -u

command -v ros2 >/dev/null 2>&1 || {
  echo "[ERROR] ros2 CLI not found; environment not configured properly." >&2
  exit 1
}

# ------------------------------------------------------------------------------
# 隔离环境: 为端到端测试分配独立的 ROS_DOMAIN_ID 与 REST 端口, 避免与运行中的系统冲突
# 容器默认系统运行于 ROS_DOMAIN_ID=42, 此处必须使用不同域 (如 77)
# ------------------------------------------------------------------------------
export ROS_DOMAIN_ID="${AMR_E2E_DOMAIN_ID:-77}"
export PYTHONUNBUFFERED=1

REST_PORT="${AMR_E2E_REST_PORT:-18080}"
MAP_DIR="${WORKSPACE_DIR}/test/maps/demo_warehouse"
MAP_YAML="${MAP_DIR}/map.yaml"
MARKERS_YAML="${MAP_DIR}/markers.yaml"
GENERATED_STATIONS="${WORKSPACE_DIR}/config/generated/stations.yaml"

echo "================================================================="
echo "       AMR Dispatcher 地图导入与调度拓扑闭环端到端验收 (Task 9)   "
echo "================================================================="
echo "[ENV] WORKSPACE_DIR : ${WORKSPACE_DIR}"
echo "[ENV] ROS_DOMAIN_ID : ${ROS_DOMAIN_ID}"
echo "[ENV] REST_PORT     : ${REST_PORT}"
echo "[ENV] MAP_YAML      : ${MAP_YAML}"
echo "[ENV] MARKERS_YAML  : ${MARKERS_YAML}"
echo "================================================================="

# ------------------------------------------------------------------------------
# 临时工作目录与日志管理
# ------------------------------------------------------------------------------
TMP_DIR="$(mktemp -d -t amr_e2e_XXXXXX)"
MAP_SERVER_LOG="${TMP_DIR}/map_server.log"
INGEST_LOG="${TMP_DIR}/map_ingest.log"
DISP_LOG="${TMP_DIR}/dispatcher.log"
TRACKER_LOG="${TMP_DIR}/path_tracker.log"
REST_LOG="${TMP_DIR}/rest_gateway.log"
ODOM_SIM_LOG="${TMP_DIR}/odom_sim.log"
PLAN_CAPTURED="${TMP_DIR}/plan_captured.yaml"

# 跟踪子进程 PID
PIDS=()

cleanup() {
  local exit_code=$?
  set +e
  echo ""
  echo "[CLEANUP] 终止测试背景进程..."
  for pid in "${PIDS[@]}"; do
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
      kill -TERM "$pid" 2>/dev/null || true
    fi
  done
  sleep 0.5
  for pid in "${PIDS[@]}"; do
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
      kill -KILL "$pid" 2>/dev/null || true
    fi
  done
  if [[ $exit_code -ne 0 ]]; then
    echo "[CLEANUP] 测试退出异常 (exit_code=${exit_code}), 日志留存于 ${TMP_DIR}"
  else
    rm -rf "${TMP_DIR}"
  fi
  exit "$exit_code"
}
trap cleanup EXIT INT TERM

fail() {
  echo ""
  echo "================================================================="
  echo "  [ASSERTION FAILED] $*"
  echo "================================================================="
  echo "-------- map_server.log (tail 40) --------"
  tail -40 "$MAP_SERVER_LOG" 2>/dev/null || true
  echo "-------- map_ingest.log (tail 60) --------"
  tail -60 "$INGEST_LOG" 2>/dev/null || true
  echo "-------- dispatcher.log (tail 80) --------"
  tail -80 "$DISP_LOG" 2>/dev/null || true
  echo "-------- path_tracker.log (tail 40) --------"
  tail -40 "$TRACKER_LOG" 2>/dev/null || true
  echo "-------- rest_gateway.log (tail 40) --------"
  tail -40 "$REST_LOG" 2>/dev/null || true
  echo "-------- odom_sim.log (tail 40) --------"
  tail -40 "$ODOM_SIM_LOG" 2>/dev/null || true
  echo "================================================================="
  exit 1
}

# 轮询文件包含特定模式 (默认 20s 超时)
wait_log() {
  local pattern="$1" file="$2" timeout_sec="${3:-20}"
  local limit=$((timeout_sec * 10))
  for ((i = 0; i < limit; i++)); do
    if grep -qE "$pattern" "$file" 2>/dev/null; then
      return 0
    fi
    sleep 0.1
  done
  return 1
}

# ------------------------------------------------------------------------------
# 阶段 0: 校验输入测试地图
# ------------------------------------------------------------------------------
echo ""
echo "[Stage 0] 校验 demo_warehouse 输入工件..."
[[ -f "${MAP_DIR}/map.pgm" ]] || fail "map.pgm missing: ${MAP_DIR}/map.pgm"
[[ -f "${MAP_YAML}" ]] || fail "map.yaml missing: ${MAP_YAML}"
[[ -f "${MARKERS_YAML}" ]] || fail "markers.yaml missing: ${MARKERS_YAML}"
echo "  [OK] demo_warehouse 包含 map.pgm, map.yaml, markers.yaml"

# ------------------------------------------------------------------------------
# 阶段 1: 启动 nav2_map_server 并通过生命周期接口激活
# ------------------------------------------------------------------------------
echo ""
echo "[Stage 1] 启动 nav2_map_server 并发布 /map..."
ros2 run nav2_map_server map_server \
  --ros-args \
  -p yaml_filename:="${MAP_YAML}" \
  -p frame_id:=map \
  > "${MAP_SERVER_LOG}" 2>&1 &
MAP_SERVER_PID=$!
PIDS+=("$MAP_SERVER_PID")

# 等待 map_server 节点在 ROS 图中注册
for ((i = 0; i < 50; i++)); do
  if ros2 node list 2>/dev/null | grep -q "/map_server"; then
    break
  fi
  sleep 0.1
done
ros2 node list 2>/dev/null | grep -q "/map_server" || fail "map_server 节点启动超时"

# 生命周期状态转换: configure -> activate
echo "  [Lifecycle] 配置 (configure) map_server..."
ros2 lifecycle set /map_server configure >/dev/null 2>&1 || fail "map_server configure 失败"
echo "  [Lifecycle] 激活 (activate) map_server..."
ros2 lifecycle set /map_server activate >/dev/null 2>&1 || fail "map_server activate 失败"

# 校验 /map 话题已被发布
for ((i = 0; i < 50; i++)); do
  if ros2 topic list 2>/dev/null | grep -q "^/map$"; then
    break
  fi
  sleep 0.1
done
ros2 topic list 2>/dev/null | grep -q "^/map$" || fail "/map 话题未被发布"
echo "  [OK] map_server 已进入 active 状态并发布 /map (transient_local)"

# ------------------------------------------------------------------------------
# 阶段 2: 启动 map_ingest_node 消费 /map, 生成 config/generated/stations.yaml
# ------------------------------------------------------------------------------
echo ""
echo "[Stage 2] 启动 map_ingest_node 执行骨架化拓扑提取..."
rm -f "${GENERATED_STATIONS}"

ros2 run amr_dispatcher_ros map_ingest_node \
  --ros-args \
  -p markers_file:="${MARKERS_YAML}" \
  -p output_stations_file:="${GENERATED_STATIONS}" \
  > "${INGEST_LOG}" 2>&1 &
INGEST_PID=$!
PIDS+=("$INGEST_PID")

# 等待 map_ingest_node 完成生成并发布 MarkerArray
echo "  [Wait] 等待拓扑解析完成日志: 'Successfully ingested map and generated'..."
if ! wait_log "Successfully ingested map and generated" "${INGEST_LOG}" 30; then
  fail "map_ingest_node 未在 30s 内完成拓扑提取"
fi

if ! wait_log "Published topology markers" "${INGEST_LOG}" 10; then
  fail "map_ingest_node 未发布拓扑可视化 markers"
fi

[[ -f "${GENERATED_STATIONS}" ]] || fail "未找到生成的 ${GENERATED_STATIONS}"
echo "  [OK] 站点拓扑文件已成功生成: ${GENERATED_STATIONS}"

# 验证 stations.yaml 内容质量与关键断言
echo "  [Verify] 断言 stations.yaml 几何与语义完整性..."
python3 - "${GENERATED_STATIONS}" <<'PYEOF'
import sys, yaml

stations_file = sys.argv[1]
with open(stations_file, 'r', encoding='utf-8') as f:
    data = yaml.safe_load(f)

# 1. map_hash 格式与非空断言
version = data.get("version", {})
map_hash = version.get("map_hash", "")
assert map_hash.startswith("fnv1a64_"), f"map_hash 必须为 fnv1a64_* 前缀, 实际为: {map_hash}"
assert len(map_hash) > 10, f"map_hash 太短: {map_hash}"

# 2. 站点断言: 必须包含 pickup_A 和 dropoff_B
stations = {s["id"]: s for s in data.get("stations", [])}
assert "station_pickup_A" in stations, "缺少 station_pickup_A"
assert "station_dropoff_B" in stations, "缺少 station_dropoff_B"
assert stations["station_pickup_A"].get("type") == "pickup", "station_pickup_A 类型必须为 pickup"
assert stations["station_dropoff_B"].get("type") == "dropoff", "station_dropoff_B 类型必须为 dropoff"

# 3. 路网节点断言: road_nodes >= 1
road_nodes = data.get("road_nodes", [])
assert len(road_nodes) >= 1, f"road_nodes 数量应 >= 1, 实际为: {len(road_nodes)}"

# 4. 拓扑边与折线断言: 必须有包含 polyline 的真实边
edges = data.get("edges", [])
assert len(edges) >= 1, f"edges 数量应 >= 1, 实际为: {len(edges)}"

found_long_polyline = False
for edge in edges:
    polyline = edge.get("polyline", [])
    dist = edge.get("distance_m", 0.0)
    if len(polyline) >= 20 and dist > 5.0:
        found_long_polyline = True
        break
assert found_long_polyline, "未找到折线长度足够且点数 >= 20 的主航线边"

print(f"  [Python-Verify-PASS] map_hash={map_hash}, stations={len(stations)}, road_nodes={len(road_nodes)}, edges={len(edges)}")
PYEOF

EXTRACTED_MAP_HASH="$(grep "map_hash:" "${GENERATED_STATIONS}" | awk '{print $2}' | tr -d '"\r\n')"
echo "  [OK] 提取到的地图 Hash: ${EXTRACTED_MAP_HASH}"

# ------------------------------------------------------------------------------
# 阶段 3: 启动闭环系统: dispatcher_node + path_tracker_node + odom_sim + rest_gateway
# ------------------------------------------------------------------------------
echo ""
echo "[Stage 3] 启动调度中枢与闭环控制系统..."

# 3.1 启动 dispatcher_node
ros2 run amr_dispatcher_ros dispatcher_node \
  --ros-args \
  -p stations_file:="${GENERATED_STATIONS}" \
  -p planner_type:=skeleton \
  -p expected_map_hash:="${EXTRACTED_MAP_HASH}" \
  > "${DISP_LOG}" 2>&1 &
DISP_PID=$!
PIDS+=("$DISP_PID")

# 等待 dispatcher_node 就绪并配置激活生命周期
for ((i = 0; i < 50; i++)); do
  if ros2 node list 2>/dev/null | grep -q "/dispatcher_node"; then
    break
  fi
  sleep 0.1
done
ros2 node list 2>/dev/null | grep -q "/dispatcher_node" || fail "dispatcher_node 启动超时"

echo "  [Lifecycle] 配置并激活 dispatcher_node..."
ros2 lifecycle set /dispatcher_node configure >/dev/null 2>&1 || fail "dispatcher_node configure 失败"
ros2 lifecycle set /dispatcher_node activate >/dev/null 2>&1 || fail "dispatcher_node activate 失败"

if ! wait_log "Path planner: SkeletonPlanner" "${DISP_LOG}" 10; then
  fail "dispatcher_node 未确认使用 SkeletonPlanner 规划器"
fi
echo "  [OK] dispatcher_node 已激活, 规划器锁定为 SkeletonPlanner, 版本门禁: ${EXTRACTED_MAP_HASH}"

# 3.2 启动 path_tracker_node (Pure Pursuit 前置点跟踪)
ros2 run amr_dispatcher_ros path_tracker_node \
  --ros-args \
  -p controller_type:=pure_pursuit \
  -p target_speed:=0.5 \
  -p lookahead_distance:=0.6 \
  -p control_rate_hz:=20.0 \
  > "${TRACKER_LOG}" 2>&1 &
TRACKER_PID=$!
PIDS+=("$TRACKER_PID")

# 3.3 启动 unicycle 运动学与里程计高精度闭环仿真器
# 初始位姿放置于 station_pickup_A 真实物理位置 (1.5, 2.0, yaw=0.0)
python3 - <<'PYEOF' > "${ODOM_SIM_LOG}" 2>&1 &
import math, time, rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class OdomSimulator(Node):
    def __init__(self):
        super().__init__('e2e_odom_simulator')
        self.x = 1.5
        self.y = 2.0
        self.yaw = 0.0
        self.v = 0.0
        self.w = 0.0
        self.last_cmd_time = time.time()
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel_raw', self.cmd_cb, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.timer = self.create_timer(0.02, self.update_physics) # 50 Hz
        self.last_update = time.time()
        self.get_logger().info('OdomSimulator started at (1.5, 2.0, 0.0)')

    def cmd_cb(self, msg: Twist):
        self.v = msg.linear.x
        self.w = msg.angular.z
        self.last_cmd_time = time.time()

    def update_physics(self):
        now = time.time()
        dt = now - self.last_update
        self.last_update = now

        # 若超过 0.3s 未收到新速度指令, 衰减停机
        if now - self.last_cmd_time > 0.3:
            self.v = 0.0
            self.w = 0.0

        # Unicycle 运动学积分
        self.yaw += self.w * dt
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # 2D yaw -> 四元数
        odom.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = self.w

        self.odom_pub.publish(odom)

rclpy.init()
node = OdomSimulator()
try:
    rclpy.spin(node)
except Exception:
    pass
finally:
    node.destroy_node()
    rclpy.shutdown()
PYEOF
ODOM_SIM_PID=$!
PIDS+=("$ODOM_SIM_PID")
echo "  [OK] OdomSimulator 闭环仿真器已在后台启动 (50Hz)"

# 3.4 启动 rest_gateway 接入网关
ros2 run amr_dispatcher_tools rest_gateway \
  --ros-args \
  -p port:="${REST_PORT}" \
  > "${REST_LOG}" 2>&1 &
REST_PID=$!
PIDS+=("$REST_PID")

if ! wait_log "REST Gateway started on port ${REST_PORT}" "${REST_LOG}" 10; then
  fail "rest_gateway 启动超时 (port ${REST_PORT})"
fi
echo "  [OK] rest_gateway 已在端口 ${REST_PORT} 就绪"

# ------------------------------------------------------------------------------
# 阶段 4: 触发业务订单并验证端到端闭环
# ------------------------------------------------------------------------------
echo ""
echo "[Stage 4] 提交业务订单 (REST Gateway) 并校验折线路径、航线锁与车辆到达..."

# 预先启动 /plan 话题捕获器 (抓取第一条发布的全局路径)
# /plan 为 VOLATILE QoS: 订阅者必须在 dispatcher 发布前完成 DDS 匹配,
# 否则会漏掉唯一的那条 path 消息。ros2 topic echo 的订阅数经 daemon 缓存
# 可能虚高(误报已匹配), 因此改用独立 rclpy 节点: 先自旋完成与 publisher
# 的真实 DDS 匹配并落 ready 标记, 脚本确认 ready 后才提交订单。
PLAN_READY_FLAG="${TMP_DIR}/plan_capture_ready"
PLAN_DONE_FLAG="${TMP_DIR}/plan_capture_done"
rm -f "${PLAN_CAPTURED}" "${PLAN_READY_FLAG}" "${PLAN_DONE_FLAG}"
python3 - "${PLAN_CAPTURED}" "${PLAN_READY_FLAG}" "${PLAN_DONE_FLAG}" > "${TMP_DIR}/plan_capture.log" 2>&1 <<'PYEOF' &
import sys, time, yaml, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Path

out_path, ready_flag, done_flag = sys.argv[1], sys.argv[2], sys.argv[3]

def pose_to_dict(ps):
    return {
        "header": {"frame_id": ps.header.frame_id},
        "pose": {
            "position": {"x": ps.pose.position.x, "y": ps.pose.position.y, "z": ps.pose.position.z},
            "orientation": {"x": ps.pose.orientation.x, "y": ps.pose.orientation.y,
                            "z": ps.pose.orientation.z, "w": ps.pose.orientation.w},
        },
    }

class PlanCapture(Node):
    def __init__(self):
        super().__init__("e2e_plan_capture")
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST)
        self.sub = self.create_subscription(Path, "/plan", self.cb, qos)
        self.got = False
        # 自旋约 1.5s 以完成与 dispatcher publisher 的 DDS 匹配
        end = time.time() + 1.5
        while time.time() < end and not self.got:
            rclpy.spin_once(self, timeout_sec=0.05)
        # 匹配完成后落 ready 标记 (即使尚未收到消息)
        if not self.got:
            with open(ready_flag, "w") as f:
                f.write("ready")
            # 继续等待首条 path 消息
            end2 = time.time() + 30.0
            while time.time() < end2 and not self.got:
                rclpy.spin_once(self, timeout_sec=0.05)

    def cb(self, msg):
        if self.got:
            return
        self.got = True
        data = {"header": {"frame_id": msg.header.frame_id},
                "poses": [pose_to_dict(p) for p in msg.poses]}
        with open(out_path, "w") as f:
            yaml.safe_dump(data, f, sort_keys=False)
        with open(done_flag, "w") as f:
            f.write("done")

rclpy.init()
node = PlanCapture()
try:
    pass  # 自旋已在构造函数内完成
except Exception:
    pass
finally:
    node.destroy_node()
    rclpy.shutdown()
PYEOF
PLAN_CAPTURE_PID=$!
PIDS+=("$PLAN_CAPTURE_PID")

# 等待捕获节点完成 DDS 匹配 (ready 标记), 最长 10s
echo "  [Wait] 等待 /plan 捕获器 DDS 匹配就绪..."
if ! wait_log "ready" "${PLAN_READY_FLAG}" 10; then
  fail "/plan 捕获器未在 10s 内完成 DDS 匹配 (无法可靠捕获路径)"
fi
echo "  [OK] /plan 捕获器已就绪, 将捕获 dispatcher 派发时发布的唯一 path"

# 提交订单: station_pickup_A -> station_dropoff_B
echo "  [REST] 发起 POST 请求提交搬运任务 (id=e2e_m1, pickup_A -> dropoff_B)..."
ORDER_RESP=$(python3 - <<PYEOF
import urllib.request, json
url = "http://127.0.0.1:${REST_PORT}/api/v1/orders?id=e2e_m1&from=station_pickup_A&to=station_dropoff_B&prio=10"
req = urllib.request.Request(url, method='POST')
try:
    with urllib.request.urlopen(req, timeout=5) as response:
        print(response.read().decode('utf-8'))
except Exception as e:
    print(f'{{"error": "{e}"}}')
PYEOF
)
echo "  [REST] 网关响应: ${ORDER_RESP}"
echo "${ORDER_RESP}" | grep -q '"status":"ok"' || fail "REST 订单提交被拒: ${ORDER_RESP}"
echo "${ORDER_RESP}" | grep -q '"accepted":true' || fail "订单未被接受: ${ORDER_RESP}"

# 4.1 校验 dispatcher 派发日志
echo "  [Dispatcher] 等待任务派发日志..."
if ! wait_log "Dispatched mission: e2e_m1" "${DISP_LOG}" 10; then
  fail "dispatcher 未在 10s 内派发任务 e2e_m1"
fi
echo "  [OK] 调度器成功派发任务 e2e_m1"

# 4.2 校验 /v2/list_traffic_reservations 航线全锁占用 (严格验证混合图全路线锁)
echo "  [Lock] 验证航线全路线锁 (/v2/list_traffic_reservations)..."
RESERVATIONS_OUTPUT=$(ros2 service call /v2/list_traffic_reservations amr_dispatcher_interfaces/srv/ListTrafficReservations "{}" 2>&1 || true)
echo "  [Lock] 锁表快照:"
echo "${RESERVATIONS_OUTPUT}" | sed 's/^/    /'

# 断言锁表中拥有 e2e_m1 作为 owner
echo "${RESERVATIONS_OUTPUT}" | grep -q "e2e_m1" || fail "未在锁表中检测到 e2e_m1 占用的航线拓扑锁"
echo "  [OK] 拓扑全航线锁已成功锁定, 资源归属: e2e_m1"

# 4.3 校验 SkeletonPlanner 路径几何 (不穿墙, 包含真实 L 拐角)
echo "  [Path] 校验 /plan 折线几何特性 (L走廊保形)..."
# 等待 plan 捕获完成 (最长 15s, 依赖捕获节点落 done 标记或文件已写入)
for ((i = 0; i < 150; i++)); do
  if [[ -f "${PLAN_DONE_FLAG}" ]] && [[ -s "${PLAN_CAPTURED}" ]] && grep -q "poses:" "${PLAN_CAPTURED}" 2>/dev/null; then
    break
  fi
  sleep 0.1
done
[[ -s "${PLAN_CAPTURED}" ]] || fail "未能捕获到 /plan 话题数据"

python3 - "${PLAN_CAPTURED}" <<'PYEOF'
import sys, yaml

plan_file = sys.argv[1]
with open(plan_file, 'r', encoding='utf-8') as f:
    data = yaml.safe_load(f)

poses = data.get("poses", [])
assert len(poses) >= 10, f"规划的路径点数过少: {len(poses)}"

# L 走廊物理范围约束:
# 横向走廊: x in [1.0, 8.2], y in [1.3, 2.7] (允许适当容差)
# 纵向走廊: x in [7.3, 8.7], y in [1.3, 6.7]
# 走廊交汇拐角: x~8.0, y~2.0
# 墙壁区域: 例如 x=5.0, y=5.0 绝对不能穿过!

has_horizontal_points = False
has_vertical_points = False
cut_through_walls = False

for p in poses:
    pos = p["pose"]["position"]
    x = pos["x"]
    y = pos["y"]

    in_h_corridor = (1.0 <= x <= 8.3) and (1.3 <= y <= 2.7)
    in_v_corridor = (7.3 <= x <= 8.7) and (1.3 <= y <= 6.7)

    if not (in_h_corridor or in_v_corridor):
        print(f"  [ERROR] 路径点 ({x:.2f}, {y:.2f}) 落在走廊边界外 (穿墙)!")
        cut_through_walls = True

    if x < 6.0 and abs(y - 2.0) < 0.6:
        has_horizontal_points = True
    if y > 3.0 and abs(x - 8.0) < 0.6:
        has_vertical_points = True

assert not cut_through_walls, "SkeletonPlanner 路径穿透了墙壁障碍!"
assert has_horizontal_points, "缺少横向走廊段路径点"
assert has_vertical_points, "缺少纵向走廊段路径点"

print(f"  [Python-Path-PASS] 路径总点数: {len(poses)}, 经检验完全约束在 L 走廊内, 无穿墙捷径")
PYEOF
echo "  [OK] SkeletonPlanner 路径经受住了严格的无穿墙几何断言"

# 4.4 等待车辆闭环航行到达目标 (Pure Pursuit + OdomSim 闭环驱动)
echo "  [Tracking] 车辆正在沿折线巡航 (~10米, 速度 0.5m/s, 预期 20~25s)..."
if ! wait_log "Mission completed: e2e_m1 \(arrived at goal\)" "${DISP_LOG}" 60; then
  fail "车辆未在 60s 内成功完成任务并上报 arrived at goal"
fi
echo "  [OK] 调度器输出: Mission completed: e2e_m1 (arrived at goal)"

# 4.5 校验任务完成后拓扑锁已被安全释放 (释放防残留)
echo "  [Lock] 验证任务完成后航线锁自动释放..."
sleep 0.5
AFTER_LOCKS=$(ros2 service call /v2/list_traffic_reservations amr_dispatcher_interfaces/srv/ListTrafficReservations "{}" 2>&1 || true)
if echo "${AFTER_LOCKS}" | grep -q "e2e_m1"; then
  fail "任务完成后拓扑锁未被正常释放: ${AFTER_LOCKS}"
fi
echo "  [OK] 航线锁已安全清空"

echo ""
echo "================================================================="
echo "  [ALL PASSED] 地图导入与调度拓扑全链路端到端集成测试圆满通过!"
echo "================================================================="
exit 0
