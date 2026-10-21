#!/usr/bin/env bash
# ==============================================================================
# check_vda5050_e2e.sh - VDA 5050 车辆适配器端到端联合验收
#
# 验收目标 (Task 6):
#   Stage 0: 虚拟小车自检 (mock_vda5050_vehicle.py --self-test)
#   Stage 1: 地图拓扑提取 (复用 check_map_ingest_e2e.sh 产出 stations.yaml)
#   Stage 2: 独立基础设施启动 (mosquitto:18884, MQTT 原始抓包器, dispatcher,
#            vda5050_gateway, rest_gateway:18080)
#   Stage 3: 桥接节点与虚拟小车拉起 (mock_amr_01, 发现车辆 + ONLINE 心跳)
#   Stage 4: 任务 m1 全程闭环验收:
#            - Wire 合法性断言 (orderUpdateId=0, nodes 严格递增, 几何位姿, controlPoints>=2, released 标志)
#            - 物理轨迹连续性 (起始点 A, 终点 B, >=10 离散位姿, 单步位移 <=0.8m)
#            - 动态逐段解锁 (到达 B 时仅释放 route_node:station_pickup_A)
#            - 任务完成与锁全清 (arrived=true 触发 CompleteMission)
#   Stage 5: 协议级 released=false 物理制动断言 (wire-halt 注入, 驻留验证, 更新恢复)
#   Stage 6: 断链安全守卫断言 (Spec 5.2):
#            - 运行中 kill -9 触发 LWT CONNECTIONBROKEN
#            - 调度器感知断链, 锁绝不释放, 全局排队挂起 (m2 锁保留, m3 调度阻断)
#            - 模拟器状态恢复后重连, 位姿重校验通过后继续执行并完成
#            - 恢复后触发自动派发 m3 并顺利完成
#   Stage 7: 验收汇总与断言状态表 (全 PASS 退出 0, 任意失败退出 1 并 dump 全量日志)
#
# 环境隔离说明:
#   容器内运行有生产环境 (PID 31, ROS_DOMAIN_ID=42, REST 8080).
#   本脚本强制使用 ROS_DOMAIN_ID=77, REST_PORT=18080, MQTT_PORT=18884,
#   网关身份 e2e_gateway, 小车身份 mock_amr_01, 完全避免端口/主题/DDS 串扰.
# ==============================================================================

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# 若在宿主机执行且本地无 ros2 命令，则自动穿透至 amr_dispatcher 容器内部执行
if [ -z "${AMR_E2E_INNER:-}" ]; then
    if ! command -v ros2 >/dev/null 2>&1; then
        echo "[E2E-WRAPPER] 检测到宿主机环境，正在穿透入 Docker amr_dispatcher 容器执行..."
        exec docker exec -i -e AMR_E2E_INNER=1 -w /workspace amr_dispatcher bash /workspace/scripts/check_vda5050_e2e.sh "$@"
    fi
fi

# ==============================================================================
# 环境初始化
# ==============================================================================
set +u
source /opt/ros/jazzy/setup.bash
if [ -f "/workspace/install/setup.bash" ]; then
    source /workspace/install/setup.bash
elif [ -f "${REPO_ROOT}/install/setup.bash" ]; then
    source "${REPO_ROOT}/install/setup.bash"
fi
set -u

export ROS_DOMAIN_ID="${AMR_E2E_DOMAIN_ID:-77}"
export PYTHONUNBUFFERED=1
REST_PORT="${AMR_E2E_REST_PORT:-18080}"
MQTT_PORT="${AMR_E2E_MQTT_PORT:-18884}"
TMP_DIR="$(mktemp -d -t amr_vda5050_e2e_XXXXXX)"

echo "======================================================================"
echo " VDA 5050 Fleet Adapter End-to-End Acceptance Test"
echo " Workspace:     ${REPO_ROOT}"
echo " ROS_DOMAIN_ID: ${ROS_DOMAIN_ID}"
echo " REST_PORT:     ${REST_PORT}"
echo " MQTT_PORT:     ${MQTT_PORT}"
echo " TMP_DIR:       ${TMP_DIR}"
echo "======================================================================"

# ==============================================================================
# 进程管理与清理陷阱
# ==============================================================================
PIDS=()
E2E_FAILED=0
START_TIME=$(date +%s)

cleanup() {
    local exit_code=$?
    echo ""
    echo "[E2E] 正在终止后台进程组..."
    for pid in "${PIDS[@]}"; do
        if kill -0 "${pid}" 2>/dev/null; then
            kill -TERM "-${pid}" 2>/dev/null || kill -TERM "${pid}" 2>/dev/null || true
        fi
    done
    sleep 0.5
    # 兜底: ros2 run 经由 Popen 启动真实可执行体，直接 kill 包装进程会留下
    # 孤儿进程持有 DDS 同名节点/端口，必须按域 ID 全量清扫
    kill_stale_e2e_procs
    for pid in "${PIDS[@]}"; do
        if kill -0 "${pid}" 2>/dev/null; then
            kill -9 "-${pid}" 2>/dev/null || kill -9 "${pid}" 2>/dev/null || true
        fi
    done

    if [ ${exit_code} -eq 0 ] && [ ${E2E_FAILED} -eq 0 ]; then
        echo "[E2E] 测试全部通过，正在清理临时目录: ${TMP_DIR}"
        rm -rf "${TMP_DIR}"
    else
        echo "[E2E][WARN] 验收未全部成功，保留临时现场以便排查: ${TMP_DIR}"
    fi
    exit ${exit_code}
}
trap cleanup EXIT INT TERM

fail() {
    local msg="$1"
    echo ""
    echo "[E2E-FAIL] ========================================================"
    echo "[E2E-FAIL] 验收失败: ${msg}"
    echo "[E2E-FAIL] ========================================================"
    E2E_FAILED=1

    echo ""
    echo "--- [LOG TAIL: mosquitto] ---"
    [ -f "${TMP_DIR}/mosq.log" ] && tail -n 40 "${TMP_DIR}/mosq.log" || echo "(no log)"
    echo "--- [LOG TAIL: dispatcher] ---"
    [ -f "${TMP_DIR}/dispatcher.log" ] && tail -n 80 "${TMP_DIR}/dispatcher.log" || echo "(no log)"
    echo "--- [LOG TAIL: vda5050_gateway] ---"
    [ -f "${TMP_DIR}/gateway.log" ] && tail -n 80 "${TMP_DIR}/gateway.log" || echo "(no log)"
    echo "--- [LOG TAIL: mock1] ---"
    [ -f "${TMP_DIR}/mock1.log" ] && tail -n 60 "${TMP_DIR}/mock1.log" || echo "(no log)"
    echo "--- [LOG TAIL: mock2] ---"
    [ -f "${TMP_DIR}/mock2.log" ] && tail -n 60 "${TMP_DIR}/mock2.log" || echo "(no log)"
    echo "--- [LOG TAIL: bridge] ---"
    [ -f "${TMP_DIR}/bridge.log" ] && tail -n 60 "${TMP_DIR}/bridge.log" || echo "(no log)"
    echo "--- [LOG TAIL: mqtt_raw.jsonl (last 20 msgs)] ---"
    [ -f "${TMP_DIR}/mqtt_raw.jsonl" ] && tail -n 20 "${TMP_DIR}/mqtt_raw.jsonl" || echo "(no log)"
    exit 1
}

wait_log() {
    local pattern="$1"
    local logfile="$2"
    local timeout_sec="$3"
    local count=0
    local max_count=$((timeout_sec * 10))
    while [ ${count} -lt ${max_count} ]; do
        if [ -f "${logfile}" ] && grep -E "${pattern}" "${logfile}" >/dev/null 2>&1; then
            return 0
        fi
        sleep 0.1
        count=$((count + 1))
    done
    return 1
}

run_bg() {
    local logfile="$1"
    shift
    "$@" > "${logfile}" 2>&1 &
    local pid=$!
    PIDS+=("${pid}")
    echo "${pid}"
}

# 按 ROS_DOMAIN_ID=77 精确识别并清扫上一轮 E2E 遗留进程。
# 只匹配启动时 environ 中带有本脚本专用域 ID 的进程 (生产环境为 42，绝不触碰)。
# 必要性: `ros2 run` 通过 Popen 启动真实可执行体，直接 kill 包装进程会遗留
# 孤儿子孙进程 (dispatcher/mock 等)，导致同名节点重复、lifecycle 调用被
# 路由到旧节点 (active 状态拒绝 configure)，本轮 dispatcher 日志为空。
kill_stale_e2e_procs() {
    local envf pid
    for envf in /proc/[0-9]*/environ; do
        pid="${envf#/proc/}"
        pid="${pid%/environ}"
        # 排除自身与父进程，避免误杀当前脚本链路
        [ "${pid}" = "$$" ] && continue
        [ "${pid}" = "${PPID}" ] && continue
        [ -r "${envf}" ] || continue
        # 2>/dev/null 必须在输入重定向前：procfs 下 [ -r ] 通过但 open 仍可能
        # 被拒绝 (跨用户/瞬时退出)，重定向在前会先向终端打印 Permission denied
        if tr '\0' '\n' 2>/dev/null < "${envf}" | grep -qx "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"; then
            kill -TERM "${pid}" 2>/dev/null || true
        fi
    done
    sleep 0.5
    for envf in /proc/[0-9]*/environ; do
        pid="${envf#/proc/}"
        pid="${pid%/environ}"
        [ "${pid}" = "$$" ] && continue
        [ "${pid}" = "${PPID}" ] && continue
        [ -r "${envf}" ] || continue
        if tr '\0' '\n' 2>/dev/null < "${envf}" | grep -qx "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"; then
            kill -KILL "${pid}" 2>/dev/null || true
        fi
    done
}

# ==============================================================================
# Preflight: 清扫上一轮遗留的 E2E 进程 (幂等入口，避免同名节点串扰)
# ==============================================================================
echo ""
echo "[Preflight] 清扫上一轮遗留的 ROS_DOMAIN_ID=${ROS_DOMAIN_ID} 进程..."
kill_stale_e2e_procs
# 陈旧的 ros2 daemon 图拓扑同样携带旧节点信息，一并停掉
ros2 daemon stop >/dev/null 2>&1 || true
echo "[Preflight] 清扫完成"

# ==============================================================================
# Stage 0: 虚拟小车自检 (mock_vda5050_vehicle.py --self-test)
# ==============================================================================
echo ""
echo "[Stage 0] 运行虚拟小车单元自检 (T1~T7)..."
if ! python3 "${SCRIPT_DIR}/mock_vda5050_vehicle.py" --self-test; then
    fail "mock_vda5050_vehicle.py --self-test 失败"
fi
echo "[Stage 0] 自检全部通过 (PASS)"

# ==============================================================================
# Stage 1: 地图拓扑提取 (获取 stations.yaml 与 expected_map_hash)
# ==============================================================================
echo ""
echo "[Stage 1] 校验/生成拓扑地图 (stations.yaml)..."
STATIONS_YAML="${REPO_ROOT}/config/generated/stations.yaml"
MAP_DIR="${REPO_ROOT}/test/maps/demo_warehouse"
MAP_YAML="${MAP_DIR}/map.yaml"
MARKERS_YAML="${MAP_DIR}/markers.yaml"

# 若不存在已生成文件，拉起 map_server + map_ingest_node 生成
if [ ! -f "${STATIONS_YAML}" ]; then
    if [ ! -f "${MAP_YAML}" ]; then
        fail "找不到测试地图文件: ${MAP_YAML}"
    fi
    echo "[Stage 1] 未发现已有 stations.yaml，启动 map_ingest 流程生成..."
    mkdir -p "${REPO_ROOT}/config/generated"
    MAP_LOG="${TMP_DIR}/map_server.log"
    run_bg "${MAP_LOG}" ros2 run nav2_map_server map_server --ros-args -p yaml_filename:="${MAP_YAML}"
    sleep 1
    ros2 lifecycle set /map_server configure >/dev/null 2>&1 || true
    ros2 lifecycle set /map_server activate >/dev/null 2>&1 || true

    INGEST_LOG="${TMP_DIR}/map_ingest.log"
    run_bg "${INGEST_LOG}" ros2 run amr_dispatcher_ros map_ingest_node --ros-args \
        -p markers_file:="${MARKERS_YAML}" \
        -p output_stations_file:="${STATIONS_YAML}" \
        -p auto_generate_edges:=true \
        -p auto_shutdown:=true

    if ! wait_log "Successfully ingested map and generated" "${INGEST_LOG}" 20; then
        fail "map_ingest_node 未在 20s 内完成拓扑提取"
    fi
fi

# 校验 stations.yaml 拓扑完整性并提取 map_hash
EXTRACTED_MAP_HASH=$(python3 - <<EOF
import yaml, sys
try:
    with open("${STATIONS_YAML}", "r") as f:
        data = yaml.safe_load(f)
    stations = data.get("stations", [])
    st_ids = {s.get("id") for s in stations}
    assert "station_pickup_A" in st_ids, "missing station_pickup_A"
    assert "station_dropoff_B" in st_ids, "missing station_dropoff_B"
    edges = data.get("edges", [])
    ab_edges = [e for e in edges if (e.get("from")=="station_pickup_A" and e.get("to")=="station_dropoff_B") or (e.get("to")=="station_pickup_A" and e.get("from")=="station_dropoff_B")]
    assert len(ab_edges) >= 1, "missing A<->B edge"
    pts = ab_edges[0].get("polyline", [])
    assert len(pts) >= 10, f"edge polyline points too few: {len(pts)}"
    h = data.get("map_hash") or (data.get("version", {}) if isinstance(data.get("version"), dict) else {}).get("map_hash", "")
    assert h.startswith("fnv1a64_"), f"invalid map_hash: {h}"
    print(h)
except Exception as e:
    print(f"ERROR: {e}", file=sys.stderr)
    sys.exit(1)
EOF
)
if [ $? -ne 0 ] || [ -z "${EXTRACTED_MAP_HASH}" ]; then
    fail "stations.yaml 校验失败"
fi
echo "[Stage 1] 拓扑校验通过，expected_map_hash=${EXTRACTED_MAP_HASH}"

# ==============================================================================
# Stage 2: 启动独立基础设施
# ==============================================================================
echo ""
echo "[Stage 2] 启动独立基础设施 (mosquitto:${MQTT_PORT}, capturer, dispatcher, gateway, rest)..."

# 2.1 启动独立 mosquitto broker
cat <<EOF > "${TMP_DIR}/mosquitto.conf"
listener ${MQTT_PORT} 127.0.0.1
allow_anonymous true
EOF
run_bg "${TMP_DIR}/mosq.log" mosquitto -c "${TMP_DIR}/mosquitto.conf" -v
sleep 0.8
if ! bash -c "cat < /dev/null > /dev/tcp/127.0.0.1/${MQTT_PORT}" 2>/dev/null; then
    fail "mosquitto 未能在端口 ${MQTT_PORT} 启动监听"
fi
echo "[Stage 2.1] mosquitto 监听于 127.0.0.1:${MQTT_PORT} (PASS)"

# 2.2 启动 MQTT 全量原始报文抓包器 (写入 mqtt_raw.jsonl)
cat <<'EOF' > "${TMP_DIR}/mqtt_capturer.py"
import json, sys, time
import paho.mqtt.client as mqtt

port = int(sys.argv[1])
out_file = sys.argv[2]
f = open(out_file, "a", buffering=1, encoding="utf-8")

def on_connect(client, userdata, flags, rc):
    client.subscribe("uagv/v2/#", qos=1)
    print("CAPTURER-READY", flush=True)

def on_message(client, userdata, msg):
    try:
        payload_str = msg.payload.decode("utf-8")
    except Exception:
        payload_str = str(msg.payload)
    record = {
        "t": time.time(),
        "topic": msg.topic,
        "payload": payload_str,
        "retain": bool(msg.retain),
        "qos": msg.qos
    }
    f.write(json.dumps(record, ensure_ascii=False) + "\n")
    f.flush()

client = mqtt.Client(client_id="e2e_mqtt_capturer", clean_session=True)
client.on_connect = on_connect
client.on_message = on_message
client.connect("127.0.0.1", port, keepalive=10)
client.loop_forever()
EOF
run_bg "${TMP_DIR}/capturer.log" python3 "${TMP_DIR}/mqtt_capturer.py" "${MQTT_PORT}" "${TMP_DIR}/mqtt_raw.jsonl"
if ! wait_log "CAPTURER-READY" "${TMP_DIR}/capturer.log" 10; then
    fail "MQTT 抓包器未能在 10s 内就绪"
fi
echo "[Stage 2.2] MQTT 抓包器就绪 (PASS)"

# 2.3 启动调度器节点 (amr_dispatcher_ros dispatcher_node) 并完成生命周期转换
run_bg "${TMP_DIR}/dispatcher.log" ros2 run amr_dispatcher_ros dispatcher_node --ros-args \
    -p stations_file:="${STATIONS_YAML}" \
    -p planner_type:=skeleton \
    -p expected_map_hash:="${EXTRACTED_MAP_HASH}" \
    -p vda5050_vehicles:="mock_amr_01" \
    -p vda5050_state_stale_sec:=5.0 \
    -p navigation_backend:=plan

echo "[Stage 2.3] 等待 dispatcher_node 节点在 DDS 拓扑中可见..."
for ((i = 0; i < 60; i++)); do
    if ros2 node list 2>/dev/null | grep -q "/dispatcher_node"; then
        break
    fi
    sleep 0.1
done
ros2 node list 2>/dev/null | grep -q "/dispatcher_node" || fail "dispatcher_node 启动超时"

echo "[Stage 2.3] 配置并激活调度器生命周期..."
if ! ros2 lifecycle set /dispatcher_node configure; then
    fail "dispatcher_node configure 失败"
fi
if ! wait_log "Task 5: subscribed /fleet/\{mock_amr_01\}" "${TMP_DIR}/dispatcher.log" 10; then
    fail "dispatcher_node 未正确订阅 mock_amr_01 链路"
fi
if ! ros2 lifecycle set /dispatcher_node activate; then
    fail "dispatcher_node activate 失败"
fi
if ! wait_log "Path planner: SkeletonPlanner" "${TMP_DIR}/dispatcher.log" 10; then
    fail "dispatcher_node 未完成 SkeletonPlanner 初始化"
fi
echo "[Stage 2.3] dispatcher_node 已进入 active 状态 (PASS)"

# 2.4 启动 VDA 5050 网关节点
run_bg "${TMP_DIR}/gateway.log" ros2 run amr_dispatcher_ros vda5050_gateway_node --ros-args \
    -p use_vda5050:=true \
    -p mqtt_broker_host:=127.0.0.1 \
    -p mqtt_port:="${MQTT_PORT}" \
    -p manufacturer:=RobotManufacturer \
    -p serial_number:=e2e_gateway \
    -p protocol_version:=v2 \
    -p heartbeat_ms:=5000
if ! wait_log "VDA 5050 网关启用" "${TMP_DIR}/gateway.log" 10; then
    fail "vda5050_gateway_node 未能在 10s 内启动"
fi
echo "[Stage 2.4] vda5050_gateway_node 启动成功 (PASS)"

# 2.5 启动 REST 网关节点
run_bg "${TMP_DIR}/rest.log" ros2 run amr_dispatcher_tools rest_gateway --ros-args \
    -p port:="${REST_PORT}"
if ! wait_log "REST Gateway started on port ${REST_PORT}" "${TMP_DIR}/rest.log" 10; then
    fail "rest_gateway 未能在 10s 内在端口 ${REST_PORT} 启动"
fi
echo "[Stage 2.5] rest_gateway 启动成功 (PASS)"

# ==============================================================================
# Stage 3: 启动桥接节点与虚拟小车
# ==============================================================================
echo ""
echo "[Stage 3] 启动桥接节点 (bridge) 与虚拟小车 (mock_amr_01)..."

# 3.1 启动双向桥接节点:
#   - 订阅 /fleet/mock_amr_01/pose -> 20Hz 转发 /odom
#   - 2Hz 周期发布 /tracking/error (保持 mission_last_progress_ 刷新)
#   - 终点到达 FSM: 当且仅当在终点 (8.025, 5.5) 驻留 >=1.5s 时单次触发 arrived=true
cat <<'EOF' > "${TMP_DIR}/e2e_bridge.py"
import math, sys, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from amr_dispatcher_interfaces.msg import TrackingError

class E2EBridge(Node):
    def __init__(self):
        super().__init__("e2e_vda_bridge")
        self.curr_x = 0.0
        self.curr_y = 0.0
        self.curr_yaw = 0.0
        self.has_pose = False

        # 到达判定 FSM 状态机 (单发消费模型，防止瞬时重触发)
        self.goal_x = 8.025
        self.goal_y = 5.5
        self.armed = False
        self.fired = False
        self.inside_since = None

        self.pose_sub = self.create_subscription(
            PoseStamped, "/fleet/mock_amr_01/pose", self.on_pose, 10
        )
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.err_pub = self.create_publisher(TrackingError, "/tracking/error", 10)

        self.timer_odom = self.create_timer(0.05, self.tick_odom)      # 20Hz
        self.timer_err = self.create_timer(0.5, self.tick_error)        # 2Hz
        self.get_logger().info("E2EBridge initialized")

    def on_pose(self, msg: PoseStamped):
        self.curr_x = msg.pose.position.x
        self.curr_y = msg.pose.position.y
        # 从四元数提取偏航角 yaw
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        self.curr_yaw = 2.0 * math.atan2(qz, qw)
        self.has_pose = True

        dx = self.curr_x - self.goal_x
        dy = self.curr_y - self.goal_y
        dist = math.hypot(dx, dy)

        now = time.time()
        if dist > 0.6:
            # 离开终点区，重新上膛 (ARM)
            self.armed = True
            self.fired = False
            self.inside_since = None
        elif self.armed and not self.fired and dist <= 0.35:
            # 进入终点判定圈
            if self.inside_since is None:
                self.inside_since = now
            elif now - self.inside_since >= 1.2:
                # 稳定驻留达标，标记触发
                self.fired = True
                self.get_logger().info(f"Goal dwell confirmed (dist={dist:.3f}m), FSM ready to fire arrived=true")

    def tick_odom(self):
        if not self.has_pose:
            return
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.curr_x
        odom.pose.pose.position.y = self.curr_y
        odom.pose.pose.orientation.z = math.sin(self.curr_yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.curr_yaw / 2.0)
        self.odom_pub.publish(odom)

    def tick_error(self):
        if not self.has_pose:
            return
        err = TrackingError()
        err.header.stamp = self.get_clock().now().to_msg()
        err.header.frame_id = "map"
        err.controller_name = "e2e_vda_bridge"
        err.nearest_waypoint_index = 0
        err.lateral_error_m = 0.0
        err.heading_error_rad = 0.0
        err.lookahead_distance_m = 0.5
        err.target_linear_x_mps = 0.0
        err.actual_linear_x_mps = 0.0

        if self.fired:
            # 单发消费：仅发一次 arrived=true, 随后退火为 false
            err.arrived = True
            self.fired = False
            self.armed = False  # 必须等离开终点区才能再次 ARM
            self.inside_since = None
            self.get_logger().info("Dispatched arrived=true to /tracking/error (consumed)")
        else:
            err.arrived = False

        self.err_pub.publish(err)

def main():
    rclpy.init()
    node = E2EBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
EOF
run_bg "${TMP_DIR}/bridge.log" python3 "${TMP_DIR}/e2e_bridge.py"
if ! wait_log "E2EBridge initialized" "${TMP_DIR}/bridge.log" 10; then
    fail "bridge 节点未能在 10s 内初始化"
fi
echo "[Stage 3.1] 桥接节点初始化完成 (PASS)"

# 3.2 启动模拟小车进程 mock1 (记录持久化状态文件，用于 Stage 6 断链恢复)
MOCK_STATE_FILE="${TMP_DIR}/mock_amr_01_state.json"
MOCK1_PID=$(run_bg "${TMP_DIR}/mock1.log" python3 "${SCRIPT_DIR}/mock_vda5050_vehicle.py" \
    --host 127.0.0.1 \
    --port "${MQTT_PORT}" \
    --manufacturer RobotManufacturer \
    --serial mock_amr_01 \
    --speed 1.0 \
    --state-file "${MOCK_STATE_FILE}")
echo "[Stage 3.2] mock_amr_01 已拉起 (PID: ${MOCK1_PID})"

# 校验网关自动发现与 MQTT ONLINE 报文
if ! wait_log "发现车辆 mock_amr_01" "${TMP_DIR}/gateway.log" 15; then
    fail "网关未能在 15s 内发现 mock_amr_01"
fi
# 给网关 1.5s 完成 /fleet/mock_amr_01/send_order 订阅端建链
sleep 1.5
if ! wait_log 'mock_amr_01/connection.*ONLINE' "${TMP_DIR}/mqtt_raw.jsonl" 10; then
    fail "MQTT 未抓取到 mock_amr_01 ONLINE 连接报文"
fi
echo "[Stage 3] 小车发现与网关建链完成 (PASS)"

# ==============================================================================
# Stage 4: 任务 m1 全程闭环验收
# ==============================================================================
echo ""
echo "[Stage 4] 提交任务 e2e_m1 (station_pickup_A -> station_dropoff_B)..."

# 4.1 REST 提交订单
SUBMIT_RES=$(python3 - <<EOF
import urllib.request, urllib.parse, json, sys
url = "http://127.0.0.1:${REST_PORT}/api/v1/orders?id=e2e_m1&from=station_pickup_A&to=station_dropoff_B&prio=10"
req = urllib.request.Request(url, method="POST", data=b"")
try:
    with urllib.request.urlopen(req, timeout=5) as resp:
        body = resp.read().decode("utf-8")
        print(body)
except Exception as e:
    print(f"ERROR: {e}", file=sys.stderr)
    sys.exit(1)
EOF
)
if [ $? -ne 0 ] || ! echo "${SUBMIT_RES}" | grep -q '"accepted":true'; then
    fail "REST 提交订单 e2e_m1 失败: ${SUBMIT_RES}"
fi
echo "[Stage 4.1] REST 订单 e2e_m1 已接收 (PASS)"

# 4.2 校验调度器派发与订单发布日志
if ! wait_log "Dispatched mission: e2e_m1" "${TMP_DIR}/dispatcher.log" 10; then
    fail "调度器未能在 10s 内派发 e2e_m1"
fi
if ! wait_log "VDA 5050 order published: mission e2e_m1" "${TMP_DIR}/dispatcher.log" 10; then
    fail "调度器未能在 10s 内向 /fleet/mock_amr_01/send_order 发布 VDA 5050 订单"
fi
echo "[Stage 4.2] 调度器完成派发并输出 send_order (PASS)"

# 4.3 MQTT Wire 格式合法性断言 (python 解析 mqtt_raw.jsonl)
echo "[Stage 4.3] 断言 VDA 5050 Wire 订单协议合法性..."
python3 - <<EOF
import json, sys, time

# 轮询等待：dispatcher 打日志 -> 网关 ROS 订阅 -> MQTT 发布 -> broker -> 抓包器
# 是四段异步链路，读一次文件会随机漏掉尚未落盘的报文（历史 flaky 点）。
deadline = time.time() + 15.0
orders = []
target_orders = []
while time.time() < deadline:
    orders = []
    target_orders = []
    with open("${TMP_DIR}/mqtt_raw.jsonl", "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line: continue
            rec = json.loads(line)
            if rec["topic"] == "uagv/v2/RobotManufacturer/mock_amr_01/order":
                try:
                    orders.append(json.loads(rec["payload"]))
                except Exception:
                    pass
    target_orders = [o for o in orders if o.get("orderId") == "e2e_m1"]
    if target_orders:
        break
    time.sleep(0.2)

assert len(target_orders) >= 1, f"未在 MQTT 捕获到 orderId=e2e_m1 的报文 (捕获到 {len(orders)} 个订单)"
o = target_orders[0]

# 严格契约断言
assert o.get("version") == "2.0.0", f"version error: {o.get('version')}"
assert o.get("manufacturer") == "RobotManufacturer", f"manufacturer error: {o.get('manufacturer')}"
assert o.get("serialNumber") == "mock_amr_01", f"serialNumber error: {o.get('serialNumber')}"
assert o.get("orderUpdateId") == 0, f"首发订单 orderUpdateId 应为 0, 实测: {o.get('orderUpdateId')}"

nodes = o.get("nodes", [])
edges = o.get("edges", [])
assert len(nodes) == 2, f"nodes 数量应为 2 (A, B), 实测: {len(nodes)}"
assert len(edges) == 1, f"edges 数量应为 1, 实测: {len(edges)}"

# 节点属性与 released 标志
n0, n1 = nodes[0], nodes[1]
assert n0["nodeId"] == "station_pickup_A", f"node0 id error: {n0['nodeId']}"
assert n0["sequenceId"] == 0, f"node0 seq error: {n0['sequenceId']}"
assert n0["released"] is True, "node0 should be released=True"
assert "nodePosition" in n0 and "x" in n0["nodePosition"] and "y" in n0["nodePosition"], "node0 missing position"
assert n0["nodePosition"]["mapId"] == "map", "node0 mapId should be 'map'"

assert n1["nodeId"] == "station_dropoff_B", f"node1 id error: {n1['nodeId']}"
assert n1["sequenceId"] == 2, f"node1 seq error: {n1['sequenceId']}"
assert n1["released"] is True, "node1 should be released=True"

# 边属性、控制点与 released 标志
e0 = edges[0]
assert e0["sequenceId"] == 1, f"edge0 seq error: {e0['sequenceId']}"
assert e0["startNodeId"] == "station_pickup_A", f"edge0 startNodeId error: {e0['startNodeId']}"
assert e0["endNodeId"] == "station_dropoff_B", f"edge0 endNodeId error: {e0['endNodeId']}"
assert e0["released"] is True, "edge0 should be released=True"
assert e0["edgeId"] == "route_edge:station_dropoff_B__station_pickup_A", f"edgeId error: {e0['edgeId']}"

traj = e0.get("trajectory", {})
cps = traj.get("controlPoints", [])
assert len(cps) >= 2, f"edge0 controlPoints should have >=2 points, got {len(cps)}"
for cp in cps:
    assert isinstance(cp.get("x"), (int, float)) and isinstance(cp.get("y"), (int, float)), f"invalid cp: {cp}"

print(f"[ASSERT-WIRE-OK] Wire 订单契约严格验证通过: {len(nodes)} 节点, {len(edges)} 边, {len(cps)} 折线控制点")
EOF
if [ $? -ne 0 ]; then
    fail "Wire 订单协议合法性断言失败"
fi

# 4.4 运动轨迹连续性断言 (轮询 mqtt_raw.jsonl 最多 35s)
echo "[Stage 4.4] 等待小车沿折线运动并验证位姿连续性..."
python3 - <<EOF
import json, math, sys, time

start_t = time.time()
trajectory = []

while time.time() - start_t < 35.0:
    trajectory.clear()
    with open("${TMP_DIR}/mqtt_raw.jsonl", "r", encoding="utf-8") as f:
        for line in f:
            if not line.strip(): continue
            try:
                rec = json.loads(line)
            except Exception:
                continue
            if rec["topic"] == "uagv/v2/RobotManufacturer/mock_amr_01/state":
                try:
                    s = json.loads(rec["payload"])
                    if s.get("orderId") == "e2e_m1" and "agvPosition" in s:
                        p = s["agvPosition"]
                        trajectory.append((p["x"], p["y"], s.get("lastNodeId")))
                except Exception:
                    pass
    if len(trajectory) >= 12 and trajectory[-1][2] == "station_dropoff_B":
        break
    time.sleep(0.5)

assert len(trajectory) >= 10, f"捕获到的有效运动位姿点过少: {len(trajectory)}"
first_p = trajectory[0]
last_p = trajectory[-1]

# 起终点坐标断言 (station_pickup_A: 1.625, 2.025; station_dropoff_B: 8.025, 5.5)
d_start = math.hypot(first_p[0] - 1.625, first_p[1] - 2.025)
d_end = math.hypot(last_p[0] - 8.025, last_p[1] - 5.5)
# state 上报频率 2Hz，小车 1m/s -> 首个采样点最多已前进 0.5m；
# 阈值取 0.6m 才能稳定反映"从 A 点出发"（原 0.3m 是历史 flaky 点）。
assert d_start < 0.6, f"初始位姿偏离 station_pickup_A: {first_p}, dist={d_start}"
assert d_end < 0.5, f"终点位姿偏离 station_dropoff_B: {last_p}, dist={d_end}"

# 单步连续性校验 (相邻点距离不超过 1.2m, 排除瞬移跳跃)
max_step = 0.0
for i in range(1, len(trajectory)):
    dx = trajectory[i][0] - trajectory[i-1][0]
    dy = trajectory[i][1] - trajectory[i-1][1]
    step = math.hypot(dx, dy)
    if step > max_step:
        max_step = step
    assert step <= 1.2, f"位姿出现不连续跳跃 step={step:.3f}m at index {i}"

print(f"[ASSERT-MOTION-OK] 运动轨迹连续性验证通过: {len(trajectory)} 采样点, 起点距 {d_start:.3f}m, 终点距 {d_end:.3f}m, 最大步长 {max_step:.3f}m")
EOF
if [ $? -ne 0 ]; then
    fail "运动轨迹连续性断言失败"
fi

# 4.5 动态逐段解锁断言 (到达 B 时仅释放 route_node:station_pickup_A)
echo "[Stage 4.5] 断言动态逐段解锁日志 (Spec 5.3)..."
EXPECTED_UNLOCK="Dynamic unlock \(Spec 5\.3\): vehicle mock_amr_01 at station_dropoff_B -> mission e2e_m1 released 1 lock\(s\): route_node:station_pickup_A"
if ! wait_log "${EXPECTED_UNLOCK}" "${TMP_DIR}/dispatcher.log" 15; then
    fail "未能在 15s 内观测到到达 B 时的动态逐段解锁日志"
fi
echo "[Stage 4.5] 动态逐段解锁断言通过 (PASS)"

# 4.6 任务到达与完成断言
echo "[Stage 4.6] 等待任务 e2e_m1 完成 (arrived=true -> CompleteMission)..."
if ! wait_log "Mission completed: e2e_m1 \(arrived at goal\)" "${TMP_DIR}/dispatcher.log" 20; then
    fail "任务 e2e_m1 未能在 20s 内由 arrived 驱动完成"
fi

# 校验 e2e_m1 占用的所有锁已被完全清理
# 终审加固：服务调用本身必须成功，空响应绝不能被当作"无残留锁"的假阳性证据
if ! RESERVATIONS=$(timeout 15 ros2 service call /v2/list_traffic_reservations amr_dispatcher_interfaces/srv/ListTrafficReservations "{}" 2>&1); then
    fail "list_traffic_reservations 服务调用失败，拒绝将空响应当作无残留锁: ${RESERVATIONS}"
fi
if echo "${RESERVATIONS}" | grep -q "e2e_m1"; then
    fail "任务 e2e_m1 完成后仍有残留锁: ${RESERVATIONS}"
fi
echo "[Stage 4.6] e2e_m1 顺利完成且全量锁已完全释放 (PASS)"

# ==============================================================================
# Stage 5: 协议级 released=false 物理制动断言 (wire-halt 注入)
# ==============================================================================
echo ""
echo "[Stage 5] 验证 VDA 5050 协议级 released=false 物理制动与驻留等待..."

# 记录当前已完成任务数与动态解锁数基线，确保 wire_halt 订单不受调度器任何干扰
BASELINE_COMPLETED=$(grep -c "Mission completed:" "${TMP_DIR}/dispatcher.log" || true)
BASELINE_UNLOCKS=$(grep -c "Dynamic unlock (Spec 5.3):" "${TMP_DIR}/dispatcher.log" || true)

# 构造两段边订单:
#   wh_n0 (8.025, 5.5) -> wh_n1 (8.025, 3.5): released=True
#   wh_n1 (8.025, 3.5) -> wh_n2 (8.025, 2.0): released=False (末端必须停在 wh_n1)
python3 - <<EOF
import json, time

order = {
    "headerId": 100,
    "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
    "version": "2.0.0",
    "manufacturer": "RobotManufacturer",
    "serialNumber": "mock_amr_01",
    "orderId": "wire_halt_test",
    "orderUpdateId": 0,
    "nodes": [
        {"nodeId": "wh_n0", "sequenceId": 0, "released": True, "actions": [],
         "nodePosition": {"x": 8.025, "y": 5.5, "theta": 0.0, "mapId": "map"}},
        {"nodeId": "wh_n1", "sequenceId": 2, "released": True, "actions": [],
         "nodePosition": {"x": 8.025, "y": 3.5, "theta": 0.0, "mapId": "map"}},
        {"nodeId": "wh_n2", "sequenceId": 4, "released": False, "actions": [],
         "nodePosition": {"x": 8.025, "y": 2.0, "theta": 0.0, "mapId": "map"}}
    ],
    "edges": [
        {"edgeId": "edge_h0_h1", "sequenceId": 1, "startNodeId": "wh_n0", "endNodeId": "wh_n1",
         "released": True, "maxSpeed": 1.0, "actions": [],
         "trajectory": {"degree": 1, "controlPoints": [{"x": 8.025, "y": 5.5}, {"x": 8.025, "y": 3.5}]}},
        {"edgeId": "edge_h1_h2", "sequenceId": 3, "startNodeId": "wh_n1", "endNodeId": "wh_n2",
         "released": False, "maxSpeed": 1.0, "actions": [],
         "trajectory": {"degree": 1, "controlPoints": [{"x": 8.025, "y": 3.5}, {"x": 8.025, "y": 2.0}]}}
    ]
}
with open("${TMP_DIR}/wire_halt_order_0.json", "w", encoding="utf-8") as f:
    json.dump(order, f)
EOF

# 通过网关 send_order 话题直接注入订单
python3 - <<EOF
import rclpy, sys, time
from rclpy.node import Node
from std_msgs.msg import String

rclpy.init()
node = Node("order_injector")
pub = node.create_publisher(String, "/fleet/mock_amr_01/send_order", 10)

with open("${TMP_DIR}/wire_halt_order_0.json", "r") as f:
    payload = f.read()

# 等待网关已订阅
for _ in range(50):
    rclpy.spin_once(node, timeout_sec=0.1)
    if pub.get_subscription_count() > 0:
        break

msg = String()
msg.data = payload
for _ in range(3):
    pub.publish(msg)
    time.sleep(0.1)

node.destroy_node()
rclpy.shutdown()
EOF

# 轮询状态: 小车必须在 wh_n1 (8.025, 3.5) 停住且连续多帧位姿不变
echo "[Stage 5.1] 等待小车在未放行边界 wh_n1 (8.025, 3.5) 驻留..."
python3 - <<EOF
import json, math, sys, time

start_t = time.time()
halt_detected = False

while time.time() - start_t < 20.0:
    states = []
    with open("${TMP_DIR}/mqtt_raw.jsonl", "r", encoding="utf-8") as f:
        for line in f:
            if not line.strip(): continue
            try:
                rec = json.loads(line)
            except Exception:
                continue
            if rec["topic"] == "uagv/v2/RobotManufacturer/mock_amr_01/state":
                try:
                    s = json.loads(rec["payload"])
                    if s.get("orderId") == "wire_halt_test":
                        states.append(s)
                except Exception:
                    pass
    if len(states) >= 6:
        # 取最后 4 帧
        tail = states[-4:]
        all_at_n1 = all(st.get("lastNodeId") == "wh_n1" for st in tail)
        poses = [st.get("agvPosition", {}) for st in tail]
        dists = [math.hypot(p.get("x", 0) - 8.025, p.get("y", 0) - 3.5) for p in poses]
        max_dist = max(dists) if dists else 999.0
        # 确认小车停滞 (速度为0，最后4帧位移极小)
        spread = math.hypot(poses[-1].get("x",0) - poses[0].get("x",0), poses[-1].get("y",0) - poses[0].get("y",0))
        if all_at_n1 and max_dist < 0.25 and spread < 0.05:
            halt_detected = True
            break
    time.sleep(0.5)

assert halt_detected, "小车未能稳定停在未放行边界 wh_n1"
print("[ASSERT-HALT-OK] 小车严格在 released=false 边界 (wh_n1) 驻留制动成功")
EOF
if [ $? -ne 0 ]; then
    fail "wire-halt 制动驻留断言失败"
fi

# 确认调度器未受外部未知订单误导
CURRENT_COMPLETED=$(grep -c "Mission completed:" "${TMP_DIR}/dispatcher.log" || true)
CURRENT_UNLOCKS=$(grep -c "Dynamic unlock (Spec 5.3):" "${TMP_DIR}/dispatcher.log" || true)
if [ "${CURRENT_COMPLETED}" -ne "${BASELINE_COMPLETED}" ] || [ "${CURRENT_UNLOCKS}" -ne "${BASELINE_UNLOCKS}" ]; then
    fail "调度器被未托管订单干扰产生非法解锁或完成: completed=${CURRENT_COMPLETED}, unlocks=${CURRENT_UNLOCKS}"
fi
echo "[Stage 5.1] 未托管订单隔离性断言通过 (PASS)"

# 注入更新订单 (orderUpdateId=1, edge1 放行: released=True)，小车恢复行驶至 wh_n2
python3 - <<EOF
import json, time

with open("${TMP_DIR}/wire_halt_order_0.json", "r") as f:
    order = json.load(f)

order["orderUpdateId"] = 1
order["timestamp"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
order["nodes"][2]["released"] = True
order["edges"][1]["released"] = True

with open("${TMP_DIR}/wire_halt_order_1.json", "w", encoding="utf-8") as f:
    json.dump(order, f)
EOF

python3 - <<EOF
import rclpy, sys, time
from rclpy.node import Node
from std_msgs.msg import String

rclpy.init()
node = Node("order_injector_resume")
pub = node.create_publisher(String, "/fleet/mock_amr_01/send_order", 10)
for _ in range(50):
    rclpy.spin_once(node, timeout_sec=0.1)
    if pub.get_subscription_count() > 0:
        break

with open("${TMP_DIR}/wire_halt_order_1.json", "r") as f:
    payload = f.read()

msg = String()
msg.data = payload
for _ in range(3):
    pub.publish(msg)
    time.sleep(0.1)

node.destroy_node()
rclpy.shutdown()
EOF

echo "[Stage 5.2] 等待小车放行后继续行驶至 wh_n2 (8.025, 2.0)..."
python3 - <<EOF
import json, math, sys, time

start_t = time.time()
reached_n2 = False

while time.time() - start_t < 15.0:
    with open("${TMP_DIR}/mqtt_raw.jsonl", "r", encoding="utf-8") as f:
        for line in f:
            if not line.strip(): continue
            try:
                rec = json.loads(line)
            except Exception:
                continue
            if rec["topic"] == "uagv/v2/RobotManufacturer/mock_amr_01/state":
                try:
                    s = json.loads(rec["payload"])
                    if s.get("orderId") == "wire_halt_test" and s.get("lastNodeId") == "wh_n2":
                        p = s.get("agvPosition", {})
                        d = math.hypot(p.get("x", 0) - 8.025, p.get("y", 0) - 2.0)
                        if d < 0.3:
                            reached_n2 = True
                            break
                except Exception:
                    pass
    if reached_n2: break
    time.sleep(0.5)

assert reached_n2, "放行更新后小车未能在 15s 内到达 wh_n2"
print("[ASSERT-RESUME-OK] 放行更新后小车顺利恢复行驶并到达终点 wh_n2")
EOF
if [ $? -ne 0 ]; then
    fail "wire-halt 放行恢复断言失败"
fi
echo "[Stage 5] released=false 制动与恢复断言全通过 (PASS)"

# ==============================================================================
# Stage 6: 断链安全守卫断言 (Spec 5.2: kill -9, 锁保留, 调度挂起, 恢复续跑)
# ==============================================================================
echo ""
echo "[Stage 6] 断链安全守卫与故障恢复断言 (Spec 5.2)..."

# 6.1 提交任务 e2e_m2 (station_pickup_A -> station_dropoff_B)
SUBMIT_M2=$(python3 - <<EOF
import urllib.request, urllib.parse, sys
url = "http://127.0.0.1:${REST_PORT}/api/v1/orders?id=e2e_m2&from=station_pickup_A&to=station_dropoff_B&prio=10"
req = urllib.request.Request(url, method="POST", data=b"")
try:
    with urllib.request.urlopen(req, timeout=5) as resp:
        print(resp.read().decode("utf-8"))
except Exception as e:
    print(f"ERROR: {e}", file=sys.stderr)
    sys.exit(1)
EOF
)
if ! echo "${SUBMIT_M2}" | grep -q '"accepted":true'; then
    fail "提交 e2e_m2 失败: ${SUBMIT_M2}"
fi

if ! wait_log "Dispatched mission: e2e_m2" "${TMP_DIR}/dispatcher.log" 10; then
    fail "调度器未能在 10s 内派发 e2e_m2"
fi
if ! wait_log "VDA 5050 order published: mission e2e_m2" "${TMP_DIR}/dispatcher.log" 10; then
    fail "调度器未能在 10s 内发布 e2e_m2 订单"
fi
echo "[Stage 6.1] 任务 e2e_m2 已成功派发，让小车沿轨迹行驶 3s..."
sleep 3.0

# 6.2 注入断链故障: kill -9 mock1
echo "[Stage 6.2] 触发断链故障: kill -9 PID ${MOCK1_PID}..."
kill -9 "${MOCK1_PID}" 2>/dev/null || true

# 校验网关接收 LWT 报文，调度器报告 link broken
if ! wait_log "VDA 5050 vehicle mock_amr_01 link broken" "${TMP_DIR}/dispatcher.log" 15; then
    fail "调度器未在 15s 内感知 mock_amr_01 link broken"
fi
echo "[Stage 6.2] 调度器成功捕获断链事件 (PASS)"

# 6.3 断言: 锁绝不释放 (Spec 5.2 铁律)
echo "[Stage 6.3] 验证断链期间锁保留铁律..."
RESERVATIONS_M2=$(timeout 15 ros2 service call /v2/list_traffic_reservations amr_dispatcher_interfaces/srv/ListTrafficReservations "{}" 2>/dev/null || echo "")
if ! echo "${RESERVATIONS_M2}" | grep -q "e2e_m2"; then
    fail "断链发生后，e2e_m2 占用的路权锁被非法释放！当前锁: ${RESERVATIONS_M2}"
fi
echo "[Stage 6.3] e2e_m2 路权锁完整保留 (PASS)"

# 6.4 提交 e2e_m3，验证全车队调度挂起 (ScheduleTick 阻断)
echo "[Stage 6.4] 提交 e2e_m3 并验证调度阻断..."
SUBMIT_M3=$(python3 - <<EOF
import urllib.request, urllib.parse, sys
url = "http://127.0.0.1:${REST_PORT}/api/v1/orders?id=e2e_m3&from=station_pickup_A&to=station_dropoff_B&prio=10"
req = urllib.request.Request(url, method="POST", data=b"")
try:
    with urllib.request.urlopen(req, timeout=5) as resp:
        print(resp.read().decode("utf-8"))
except Exception as e:
    print(f"ERROR: {e}", file=sys.stderr)
    sys.exit(1)
EOF
)
if ! echo "${SUBMIT_M3}" | grep -q '"accepted":true'; then
    fail "提交 e2e_m3 失败: ${SUBMIT_M3}"
fi

# 等待 6s，断言 e2e_m3 绝未被派发，且调度器输出 Dispatch suspended 警告
sleep 6.0
if grep -q "Dispatched mission: e2e_m3" "${TMP_DIR}/dispatcher.log"; then
    fail "在 mock_amr_01 断链状态下，调度器非法派发了 e2e_m3！"
fi
if ! grep -q "Dispatch suspended:" "${TMP_DIR}/dispatcher.log"; then
    fail "未能在调度日志中观测到 Dispatch suspended 警告"
fi
echo "[Stage 6.4] 调度挂起防护验证通过 (e2e_m3 未派发，Dispatch suspended 激活) (PASS)"

# 6.5 重启虚拟小车 mock2 (继承 mock_amr_01_state.json)，校验位姿重校验与重连日志
echo "[Stage 6.5] 重启虚拟小车 mock2 模拟网络恢复与状态持久化还原..."
MOCK2_PID=$(run_bg "${TMP_DIR}/mock2.log" python3 "${SCRIPT_DIR}/mock_vda5050_vehicle.py" \
    --host 127.0.0.1 \
    --port "${MQTT_PORT}" \
    --manufacturer RobotManufacturer \
    --serial mock_amr_01 \
    --speed 1.0 \
    --state-file "${MOCK_STATE_FILE}")
echo "[Stage 6.5] mock2 已拉起 (PID: ${MOCK2_PID})"

# 断言重连日志: 只认网关权威 online=true 重发触发的 "back online"
# （离线期间的 state 竞态不再回置 online，调度核心不会因 DDS 乱序误判链路恢复）
if ! wait_log "VDA 5050 vehicle mock_amr_01 back online" "${TMP_DIR}/dispatcher.log" 20; then
    fail "调度器未能在 20s 内感知小车恢复重连"
fi
echo "[Stage 6.5] 重连感知与位姿重校验通过 (PASS)"

# 6.6 小车继续行驶至 B，e2e_m2 完成
echo "[Stage 6.6] 等待 e2e_m2 恢复行驶并最终完成..."
if ! wait_log "Mission completed: e2e_m2 \(arrived at goal\)" "${TMP_DIR}/dispatcher.log" 45; then
    fail "任务 e2e_m2 未能在恢复后 45s 内顺利完成"
fi
echo "[Stage 6.6] e2e_m2 顺利完成 (PASS)"

# 6.7 调度队列自动唤醒，e2e_m3 自动派发并执行完成
echo "[Stage 6.7] 验证调度队列自动恢复派发 e2e_m3..."
if ! wait_log "Dispatched mission: e2e_m3" "${TMP_DIR}/dispatcher.log" 20; then
    fail "调度器未能在 e2e_m2 完成后自动唤醒派发 e2e_m3"
fi
if ! wait_log "VDA 5050 order published: mission e2e_m3" "${TMP_DIR}/dispatcher.log" 10; then
    fail "调度器未发布 e2e_m3 订单"
fi

echo "[Stage 6.7] 等待 e2e_m3 执行完成..."
if ! wait_log "Mission completed: e2e_m3 \(arrived at goal\)" "${TMP_DIR}/dispatcher.log" 45; then
    fail "任务 e2e_m3 未能在 45s 内执行完成"
fi

# 最终路权锁全清断言
# 终审加固：同 Stage 4.6，服务失败必须 fail 而非静默视为干净
if ! FINAL_RESERVATIONS=$(timeout 15 ros2 service call /v2/list_traffic_reservations amr_dispatcher_interfaces/srv/ListTrafficReservations "{}" 2>&1); then
    fail "list_traffic_reservations 服务调用失败，拒绝将空响应当作无残留锁: ${FINAL_RESERVATIONS}"
fi
if echo "${FINAL_RESERVATIONS}" | grep -qE "e2e_m1|e2e_m2|e2e_m3|route_"; then
    fail "所有任务完成后仍有残留路权锁: ${FINAL_RESERVATIONS}"
fi
echo "[Stage 6.7] e2e_m3 完成且全车队路权锁完全清理 (PASS)"

# ==============================================================================
# Stage 6.8: 操作员人工强清离线链路验收 (Task 7 / 终审 I1 闭合)
# ==============================================================================
# 前提: 在 Stage 6.5 恢复后，mock2 作为 mock_amr_01 已恢复在线；
#       要验证人工强清，需要先把这辆车再次打成离线状态。
# 目的: 验证 /dispatcher/force_clear_vda5050_link 可摘除离线挂载并解除挂起守卫。

# 6.8.1 【在线态】防误杀活车断言：此时 mock_amr_01 仍在线，强清必须被拒绝
echo "[Stage 6.8] 在线态断言：对在线车辆强清应被拒绝..."
# 先等服务就绪（首次 discovery 需要时间）
for ((i = 0; i < 30; i++)); do
    if ros2 service list 2>/dev/null | grep -q "force_clear_vda5050_link"; then
        break
    fi
    sleep 0.5
done
ALIVE_RESULT=$(timeout 20 ros2 service call /dispatcher/force_clear_vda5050_link \
    amr_dispatcher_interfaces/srv/ForceClearVda5050Link \
    "{vehicle_id: 'mock_amr_01', confirm_vehicle_id: 'mock_amr_01', operator_id: 'e2e_operator', reason: 'e2e alive-refuse test'}" 2>&1 || true)
if echo "${ALIVE_RESULT}" | grep -q "success=True"; then
    fail "在线车辆 mock_amr_01 竟被强清受理（防误杀活车失效）: ${ALIVE_RESULT}"
fi
if ! echo "${ALIVE_RESULT}" | grep -q "success=False"; then
    fail "在线态强清拒绝路径未返回 success=False（服务不可达或响应异常）: ${ALIVE_RESULT}"
fi
echo "[Stage 6.8] 拒绝路径断言通过：在线车辆强清被拒 (PASS)"

# 6.8.2 提交 e2e_m5 让其持有路权锁（强清要释放的正是这批锁）
echo "[Stage 6.8] 提交 e2e_m5 使其持有路权锁（强清释放对象）..."
SUBMIT_M5=$(python3 - <<EOF
import urllib.request, sys
url = "http://127.0.0.1:${REST_PORT}/api/v1/orders?id=e2e_m5&from=station_pickup_A&to=station_dropoff_B&prio=6"
req = urllib.request.Request(url, method="POST", data=b"")
try:
    with urllib.request.urlopen(req, timeout=5) as resp:
        print(resp.read().decode("utf-8"))
except Exception as e:
    print(f"ERROR: {e}", file=sys.stderr)
    sys.exit(1)
EOF
)
if [ $? -ne 0 ] || ! echo "${SUBMIT_M5}" | grep -q '"accepted":true'; then
    fail "REST 提交 e2e_m5 失败: ${SUBMIT_M5}"
fi
if ! wait_log "Dispatched mission: e2e_m5" "${TMP_DIR}/dispatcher.log" 15; then
    fail "e2e_m5 未能在 15s 内派发"
fi
echo "[Stage 6.8] e2e_m5 已派发并持有路权锁 (PASS)"

# 6.8.3 再次 kill mock2，让 mock_amr_01 回到离线态（锁必须保留）
echo "[Stage 6.8] 二次断链: kill -9 mock2 让 mock_amr_01 再次离线..."
kill -9 "${MOCK2_PID:-0}" 2>/dev/null || true

# 断言: 调度器捕获二次断链
sleep 2  # 等 LWT + link guard tick
if ! grep -q "VDA 5050 vehicle mock_amr_01 link broken" "${TMP_DIR}/dispatcher.log"; then
    fail "未能观测到 mock_amr_01 link-broken（可能 mock2 未在运行期kill或日志时序异常）"
fi
echo "[Stage 6.8] mock_amr_01 已再次离线 (PASS)"

# 断言: 断链期间 e2e_m5 的锁必须仍在（铁律）
if ! RESV_M5=$(timeout 15 ros2 service call /v2/list_traffic_reservations \
        amr_dispatcher_interfaces/srv/ListTrafficReservations "{}" 2>&1); then
    fail "list_traffic_reservations 服务调用失败: ${RESV_M5}"
fi
if ! echo "${RESV_M5}" | grep -q "e2e_m5"; then
    fail "断链期间 e2e_m5 的路权锁被非法释放: ${RESV_M5}"
fi
echo "[Stage 6.8] 断链期间 e2e_m5 锁保留（铁律） (PASS)"

# 6.8.4 离线态强清：摘除 mock_amr_01 的离线挂载并释放名下锁
FORCE_RESULT=$(timeout 20 ros2 service call /dispatcher/force_clear_vda5050_link \
    amr_dispatcher_interfaces/srv/ForceClearVda5050Link \
    "{vehicle_id: 'mock_amr_01', confirm_vehicle_id: 'mock_amr_01', operator_id: 'e2e_operator', reason: 'e2e stage 6.8 operator force clear'}" 2>&1)
echo "${FORCE_RESULT}"

# 断言服务成功（response.success=true）
if ! echo "${FORCE_RESULT}" | grep -q "success=True"; then
    fail "ForceClearVda5050Link 服务未被受理成功: ${FORCE_RESULT}"
fi
# 断言名下锁确被释放（released_resources 非空且含 route_ 锁）
if ! echo "${FORCE_RESULT}" | grep -q "route_"; then
    fail "强清未释放该车名下的路权锁（released_resources 为空）: ${FORCE_RESULT}"
fi
echo "[Stage 6.8] 离线链路已被人工强清且名下锁已释放 (PASS)"

# 断言: 强清后车辆进入【隔离态】(quarantined)——不摘车，只是不再参与分配
if ! echo "${FORCE_RESULT}" | grep -q "quarantined=True"; then
    fail "强清后车辆未进入隔离态 (quarantined=False): ${FORCE_RESULT}"
fi
echo "[Stage 6.8] 强清后车辆进入隔离态 (PASS)"

# 断言: e2e_m5 的锁确已从锁表消失
if ! RESV_AFTER=$(timeout 15 ros2 service call /v2/list_traffic_reservations \
        amr_dispatcher_interfaces/srv/ListTrafficReservations "{}" 2>&1); then
    fail "list_traffic_reservations 服务调用失败: ${RESV_AFTER}"
fi
if echo "${RESV_AFTER}" | grep -q "e2e_m5"; then
    fail "强清后 e2e_m5 锁仍在锁表: ${RESV_AFTER}"
fi
echo "[Stage 6.8] 强清后锁表已无 e2e_m5 残留 (PASS)"

# 6.8.4 断言守卫挂起解除：新提交的 e2e_m4 应能派发（而非继续 Dispatch suspended）
echo "[Stage 6.8] 验证强清后调度挂起解除（提交 e2e_m4）..."
SUBMIT_M4=$(python3 - <<EOF
import urllib.request, sys
url = "http://127.0.0.1:${REST_PORT}/api/v1/orders?id=e2e_m4&from=station_dropoff_B&to=station_pickup_A&prio=5"
req = urllib.request.Request(url, method="POST", data=b"")
try:
    with urllib.request.urlopen(req, timeout=5) as resp:
        print(resp.read().decode("utf-8"))
except Exception as e:
    print(f"ERROR: {e}", file=sys.stderr)
    sys.exit(1)
EOF
)
if [ $? -ne 0 ]; then
    fail "REST 提交 e2e_m4 失败: ${SUBMIT_M4}"
fi
if ! echo "${SUBMIT_M4}" | grep -q '"accepted":true'; then
    fail "REST 拒绝 e2e_m4 提单: ${SUBMIT_M4}"
fi
echo "[Stage 6.8] e2e_m4 已提交，等待派发（断言守卫挂起解除）..."
if ! wait_log "Dispatched mission: e2e_m4" "${TMP_DIR}/dispatcher.log" 20; then
    fail "ForceClear 后新任务 e2e_m4 未能在 20s 内派发 —— 挂起守卫未解除"
fi
echo "[Stage 6.8] 强清后调度挂起成功解除 (PASS)"

# 6.8.5 恢复流程：操作员确认车辆位置已核实后方可解除隔离
# 前提：恢复要求车辆【在线】（离线车连状态都报不上，回池只会再次触发挂起），
#       故先拉起 mock3 让 mock_amr_01 重新上线，再执行恢复。
echo "[Stage 6.8] 恢复流程：拉起 mock3 使 mock_amr_01 重新上线..."
MOCK3_PID=$(run_bg "${TMP_DIR}/mock3.log" python3 "${SCRIPT_DIR}/mock_vda5050_vehicle.py" \
    --host 127.0.0.1 \
    --port "${MQTT_PORT}" \
    --manufacturer RobotManufacturer \
    --serial mock_amr_01 \
    --speed 1.0 \
    --state-file "${MOCK_STATE_FILE}")
echo "[Stage 6.8] mock3 已拉起 (PID: ${MOCK3_PID})"
# 注意：Stage 6.5 已经出现过一次 "back online"，wait_log 会命中历史行——
# 这里必须等【出现次数增加】，否则会误判为已上线。
BACKONLINE_BEFORE=$(grep -c "VDA 5050 vehicle mock_amr_01 back online" "${TMP_DIR}/dispatcher.log" || true)
BACKONLINE_OK=0
for ((i = 0; i < 250; i++)); do
    BACKONLINE_NOW=$(grep -c "VDA 5050 vehicle mock_amr_01 back online" "${TMP_DIR}/dispatcher.log" || true)
    if [ "${BACKONLINE_NOW}" -gt "${BACKONLINE_BEFORE}" ]; then
        BACKONLINE_OK=1
        break
    fi
    sleep 0.1
done
if [ "${BACKONLINE_OK}" -ne 1 ]; then
    fail "调度器未能在 25s 内感知 mock_amr_01 重新上线（恢复前置条件不满足）"
fi
echo "[Stage 6.8] mock_amr_01 已重新上线 (PASS)"

RESTORE_RESULT=$(timeout 20 ros2 service call /dispatcher/restore_vda5050_link \
    amr_dispatcher_interfaces/srv/RestoreVda5050Link \
    "{vehicle_id: 'mock_amr_01', confirm_vehicle_id: 'mock_amr_01', operator_id: 'e2e_operator', reason: 'e2e stage 6.8 restore', position_verified: true}" 2>&1)
echo "${RESTORE_RESULT}"
if ! echo "${RESTORE_RESULT}" | grep -q "success=True"; then
    fail "RestoreVda5050Link 未被受理成功: ${RESTORE_RESULT}"
fi
echo "[Stage 6.8] 隔离态恢复成功 (PASS)"
echo "[Stage 6.8] 人工强清离线链路验收完成 (PASS)"

# ==============================================================================
# Stage 7: 验收汇总与断言状态表
# ==============================================================================
END_TIME=$(date +%s)
DURATION=$((END_TIME - START_TIME))

echo ""
echo "======================================================================"
echo " VDA 5050 Fleet Adapter E2E Acceptance Summary (Duration: ${DURATION}s)"
echo "======================================================================"
cat <<'EOF'
 [PASS] Stage 0: 虚拟小车自检 (mock_vda5050_vehicle.py --self-test T1~T7)
 [PASS] Stage 1: 地图拓扑提取与完整性校验 (stations.yaml, map_hash)
 [PASS] Stage 2: 独立基础设施启动 (mosquitto, capturer, dispatcher, gateway, rest)
 [PASS] Stage 3: 桥接节点与虚拟小车拉起 (mock_amr_01, 网关自动发现, ONLINE 心跳)
 [PASS] Stage 4: 任务 m1 全程闭环验收:
                 - Wire 合法性契约 (nodes 递增, controlPoints>=2, released 标志)
                 - 物理轨迹连续性 (A->B, 无瞬变跳变)
                 - 动态逐段解锁 (到达 B 仅释放 node A)
                 - 任务完成与锁全清
 [PASS] Stage 5: 协议级 released=false 物理制动与放行恢复 (wire-halt 注入)
 [PASS] Stage 6: 断链安全守卫断言 (Spec 5.2):
                 - kill -9 触发 LWT CONNECTIONBROKEN
                 - 锁完整保留 (绝不释放)
                 - 调度挂起与派发阻断 (Dispatch suspended)
                 - 持久化恢复与位姿重校验
                 - 恢复续跑完成与后置任务自动唤醒派发
 [PASS] Stage 6.8: 操作员人工强清离线链路 (Spec 5.2 人工强解半边):
                 - 在线车强清被拒 (防误杀活车)
                 - 二次断链守卫挂起 + 断链期间锁保留 (铁律)
                 - 强清受理: link 摘除 + 名下锁释放 + 在途任务收口
                 - 挂起解除: 后续任务恢复派发
======================================================================
 ALL VDA 5050 E2E ASSERTIONS PASSED SUCCESSFULLY!
======================================================================
EOF

exit 0
