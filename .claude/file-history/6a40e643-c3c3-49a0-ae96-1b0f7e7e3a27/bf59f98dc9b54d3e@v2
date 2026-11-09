#!/usr/bin/env bash
# ==============================================================================
# check_topology_hot_update_e2e.sh - 在线拓扑热生效端到端验收（相位 11 / C+D+E）
#
# 现场问题
# --------
# 临时施工区只能靠进容器 `vim stations.yaml`，既没有连通性校验，也没有对在途
# 车辆的安全仲裁；而 map_ingest / Web 地图上传会**整份覆盖**那个文件，
# 手改的施工区会被静默抹掉。本验收锁的是新链路的三条硬约束：
#
#   1. **拒绝时不写盘**：预览/校验失败绝不在磁盘上留下半份施工区
#   2. **原子性**：一条边都不封（不是"封了一半"），在途车压在边上时整单拒绝
#   3. **持久与失效**：重启后施工区还在；基础地图换版后施工区整体失效并归档
#
# 断言（T1~T12，与计划的验收表一一对应）
#   T1  初始 revision 0、无叠加层文件
#   T2  断连叠加层 preview -> 409 would_disconnect_topology + 不可达站点 + 不写盘
#   T3  缺 reason -> 400 + 仍不写盘
#   T4  合法封非桥边 -> 200、文件存在、revision 1、/metrics amr_topology_blocked_edges 1
#   T5  重放同一请求 -> 200 "noop"、revision 不变（幂等，零写盘零计数）
#   T6  错误 expected_map_hash -> 409 map_hash_mismatch、revision 不变
#   T7  封非桥边 b-c 后提单 -> 下发的 VDA 订单走旁路（不含 b，含 n1）
#   T8  在途车正压在将要封的边上 -> 409 edge_occupied_in_flight + 车辆 id，
#      且该车**没有**收到新的订单更新（没有半路改道）
#   T9  kill + 重启 dispatcher -> 叠加层从文件回放，revision 仍在，
#      list_traffic_reservations 仍见 traffic_block:topology_overlay:* owner
#   T10 改 stations.yaml 的 map_hash 等一拍 -> 叠加层失效（revision 0）、
#      出现 .stale.* 归档件、审计里有 topology_overlay_invalidated
#   T11 unblock_edge -> 计数归零、metrics 归零、预约表不再有该 owner
#   T12 legacy 红线：全程无叠加层时 /v2/list_stations 投影与基线逐字一致
#
# 环境隔离：域 93 / REST 18101 / MQTT 18102，与生产（42 / 8080 / 1883）
#           及既有 e2e（77..92）不重叠。拓扑用**夹具**（主链 a-b-c-d-e +
#           单条 a→d 旁路经 n1），保证"桥边/非桥边/车压在边上"三种情形
#           确定可复现，不依赖 demo 地图的偶然形状。
#
# 退出码：0 = 全部通过；非 0 = 断言失败（现场保留在容器 /tmp 便于排查）
# ==============================================================================
set -eo pipefail
set -E
trap 'echo "[TOPO-FATAL] line=${LINENO} status=$? cmd=${BASH_COMMAND}" >&2' ERR

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

CONTAINER="${AMR_TOPO_CONTAINER:-amr_dispatcher}"
TOPO_DOMAIN_ID="${AMR_TOPO_DOMAIN_ID:-93}"
TOPO_REST_PORT="${AMR_TOPO_REST_PORT:-18101}"
TOPO_MQTT_PORT="${AMR_TOPO_MQTT_PORT:-18102}"
# 内层模式（CI 的 e2e-fleet job：脚本本身就跑在 ROS 容器里）：把 /workspace
# 换成脚本所在仓库根目录，并用 shim 顶掉 docker 子命令。二者的**唯一**区别
# 是"在哪里执行"；断言序列与生产红线检查逐字不变。
# 必须在任何 docker 调用之前完成替换（下面的 docker ps 就在检查容器）。
if [ -n "${AMR_E2E_INNER:-}" ]; then
    CONTAINER_WORKDIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
    INNER_CONTAINER_NAME="${CONTAINER:-amr_dispatcher}"
    # shellcheck source=scripts/lib/inner_docker_shim.sh
    . "$(dirname "${BASH_SOURCE[0]}")/lib/inner_docker_shim.sh"
else
    CONTAINER_WORKDIR="${AMR_TOPO_WORKDIR:-/workspace}"
fi
STAMP_DIR="${CONTAINER_WORKDIR}/.topo_stamp"
VEHICLE_ID="topo_amr_01"
MAP_HASH="fnv1a64_topoe2e0000abcd"

TOPO_FAILED=0

for tool in docker python3; do
    # 内层模式没有 docker 二进制：shim 提供同名函数，command -v 也能看到它
    command -v "${tool}" >/dev/null 2>&1 || {
        echo "[TOPO-FAIL] 宿主机缺少必需工具: ${tool}" >&2
        exit 2
    }
done
docker ps --format '{{.Names}}' | grep -qx "${CONTAINER}" || {
    echo "[TOPO-FAIL] 容器 ${CONTAINER} 未运行" >&2
    exit 2
}
# 生产红线：域 42 / 端口 8080 / MQTT 1883 绝不触碰
if [ "${TOPO_DOMAIN_ID}" = "42" ] || [ "${TOPO_REST_PORT}" = "8080" ] \
   || [ "${TOPO_MQTT_PORT}" = "1883" ]; then
    echo "[TOPO-FAIL] 拒绝使用生产域/端口 (42 / 8080 / 1883)" >&2
    exit 2
fi

fail() {
    echo ""
    echo "[TOPO-FAIL] 验收失败: $1"
    echo "--- [LOG TAIL: dispatcher] ---"
    docker exec "${CONTAINER}" bash -c "tail -n 50 '${STAMP_DIR}/dispatcher.log' 2>/dev/null" || echo "(no log)"
    echo "--- [OVERLAY FILE] ---"
    docker exec "${CONTAINER}" bash -c "cat '${STAMP_DIR}/overlay.yaml' 2>/dev/null" || echo "(no file)"
    echo "--- [AUDIT] ---"
    docker exec "${CONTAINER}" bash -c "cat '${STAMP_DIR}/audit.jsonl' 2>/dev/null" || echo "(no audit)"
    echo "--- [REST] ---"
    curl -s --max-time 3 "http://127.0.0.1:${TOPO_REST_PORT}/api/v1/topology/overlay?yaml=1" || true
    TOPO_FAILED=1
    exit 1
}

cleanup() {
    local exit_code=$?
    echo ""
    echo "[TOPO] 正在清理..."
    docker exec "${CONTAINER}" bash -c "
        for envf in /proc/[0-9]*/environ; do
            pid=\"\${envf#/proc/}\"; pid=\"\${pid%/environ}\"
            [ -r \"\${envf}\" ] || continue
            if tr '\\0' '\\n' 2>/dev/null < \"\${envf}\" | grep -qx 'ROS_DOMAIN_ID=${TOPO_DOMAIN_ID}'; then
                kill -KILL \"\${pid}\" 2>/dev/null || true
            fi
        done
        pkill -9 -f '[r]est_gateway.*port:=${TOPO_REST_PORT}' 2>/dev/null || true
        pkill -9 -f '[m]ock_vda5050_vehicle.py --host 127.0.0.1 --port ${TOPO_MQTT_PORT}' 2>/dev/null || true
        pkill -9 -f '[m]osquitto -c ${STAMP_DIR}/mosquitto.conf' 2>/dev/null || true
        ROS_DOMAIN_ID=${TOPO_DOMAIN_ID} bash -c 'source /opt/ros/jazzy/setup.bash >/dev/null 2>&1; ros2 daemon stop >/dev/null 2>&1' || true
    " >/dev/null 2>&1 || true
    if [ ${exit_code} -eq 0 ] && [ ${TOPO_FAILED} -eq 0 ]; then
        docker exec "${CONTAINER}" bash -c "rm -rf '${STAMP_DIR}'" >/dev/null 2>&1 || true
        echo "[TOPO] 全部通过，已清理临时现场"
    else
        echo "[TOPO][WARN] 验收未全部成功，现场保留在容器内: ${STAMP_DIR}"
    fi
    exit ${exit_code}
}
trap cleanup EXIT INT TERM

docker exec "${CONTAINER}" bash -c "rm -rf '${STAMP_DIR}' && mkdir -p '${STAMP_DIR}'"

echo "======================================================================"
echo " Online Topology Hot-Update End-to-End Acceptance"
echo " ROS_DOMAIN_ID: ${TOPO_DOMAIN_ID}  REST: ${TOPO_REST_PORT}  MQTT: ${TOPO_MQTT_PORT}"
echo "======================================================================"

# ---------------------------------------------------------------------------
# 夹具拓扑：
#
#     a -2- b -2- c -2- d -2- e      主链 a..e = 8m
#      \                        /
#       +--5------ n1 ------5--+     旁路一 a-n1-d = 10m
#       |
#       +--6------ n2 ------6--+     旁路二 a-n2-d = 12m
#
# **两条旁路都刻意比主链长**：无叠加层时最短路径是主链 a-b-c-d-e，
# 封掉 b-c 之后才被迫改走 a-n1-d-e。若旁路更短，则封不封 b-c 的
# 最短路都一样是旁路，"路径是否换了"根本测不出来。
#
# 为什么需要**第二条**旁路：T8 要在车正走 a-n1 时封 a-n1。如果 a-n1 是
# a 与其余站点的唯一联系（只有一条旁路时确实如此，因为 T4 已封了 b-c），
# 那么这次提交会先撞上 would_disconnect_topology —— 连通性检查在在途
# 影响分析之前，这是**正确**的行为，但它让 T8 永远测不到
# edge_occupied_in_flight。补一条 n2 之后，封 a-n1 全图仍连通，
# 请求才会走到在途仲裁那一步。这正是现场的样子：施工封一段，
# 别的绕行还在，真正要拦的是"从车轮下抽路"。
#
# 边分类（决定每个断言能测什么）：
#   * 桥边 d-e：封掉后 e 孤立 -> T2 的 would_disconnect_topology
#   * 非桥边 b-c：a 仍可经 a-n1-d / a-n2-d 到 c/d/e，全图连通
#     -> T4 的"合法提交"，并让 T7 能观察"最短路真的被改写"
#   * 旁路一 a-n1：T7 派单后车正压在这条边上 -> T8 的 edge_occupied_in_flight
# ---------------------------------------------------------------------------
cat > /tmp/topo_stations.yaml <<EOF
version:
  map_hash: "${MAP_HASH}"
  graph_version: "v1"
stations:
  - id: "a"
    x: 0.0
    y: 0.0
    yaw: 0.0
    type: "pickup"
  - id: "b"
    x: 2.0
    y: 0.0
    yaw: 0.0
    type: "pickup"
  - id: "c"
    x: 4.0
    y: 0.0
    yaw: 0.0
    type: "dropoff"
  - id: "d"
    x: 6.0
    y: 0.0
    yaw: 0.0
    type: "dropoff"
  - id: "e"
    x: 8.0
    y: 0.0
    yaw: 0.0
    type: "dropoff"
road_nodes:
  - id: "n1"
    x: 3.0
    y: 4.0
    yaw: 0.0
  - id: "n2"
    x: 3.0
    y: -6.0
    yaw: 0.0
edges:
  - from: "a"
    to: "b"
    distance_m: 2.0
    bidirectional: true
    enabled: true
    polyline:
      - [0.0, 0.0]
      - [2.0, 0.0]
  - from: "b"
    to: "c"
    distance_m: 2.0
    bidirectional: true
    enabled: true
    polyline:
      - [2.0, 0.0]
      - [4.0, 0.0]
  - from: "c"
    to: "d"
    distance_m: 2.0
    bidirectional: true
    enabled: true
    polyline:
      - [4.0, 0.0]
      - [6.0, 0.0]
  - from: "d"
    to: "e"
    distance_m: 2.0
    bidirectional: true
    enabled: true
    polyline:
      - [6.0, 0.0]
      - [8.0, 0.0]
  - from: "a"
    to: "n1"
    distance_m: 5.0
    bidirectional: true
    enabled: true
    polyline:
      - [0.0, 0.0]
      - [3.0, 4.0]
  - from: "n1"
    to: "d"
    distance_m: 5.0
    bidirectional: true
    enabled: true
    polyline:
      - [3.0, 4.0]
      - [6.0, 0.0]
  - from: "a"
    to: "n2"
    distance_m: 6.0
    bidirectional: true
    enabled: true
    polyline:
      - [0.0, 0.0]
      - [3.0, -6.0]
  - from: "n2"
    to: "d"
    distance_m: 6.0
    bidirectional: true
    enabled: true
    polyline:
      - [3.0, -6.0]
      - [6.0, 0.0]
EOF
docker cp /tmp/topo_stations.yaml "${CONTAINER}:${STAMP_DIR}/stations.yaml" >/dev/null

# ---------------------------------------------------------------------------
# 栈：mosquitto + dispatcher(开叠加层) + vda5050_gateway + rest + mock 车
# ---------------------------------------------------------------------------
cat > /tmp/topo_stack.sh <<INNER
set -eo pipefail
cd "${CONTAINER_WORKDIR}"
source /opt/ros/jazzy/setup.bash
source "${CONTAINER_WORKDIR}/install/setup.bash"
export ROS_DOMAIN_ID=${TOPO_DOMAIN_ID}
export PYTHONUNBUFFERED=1
STAMP="${STAMP_DIR}"

cat > "\${STAMP}/mosquitto.conf" <<EOF
listener ${TOPO_MQTT_PORT} 127.0.0.1
allow_anonymous true
EOF
mosquitto -c "\${STAMP}/mosquitto.conf" -v > "\${STAMP}/mosq.log" 2>&1 &
sleep 1

ros2 run amr_dispatcher_ros dispatcher_node --ros-args \\
    -p stations_file:="\${STAMP}/stations.yaml" \\
    -p planner_type:=skeleton \\
    -p expected_map_hash:="${MAP_HASH}" \\
    -p topology_overlay_file:="\${STAMP}/overlay.yaml" \\
    -p audit_log_file:="\${STAMP}/audit.jsonl" \\
    -p vda5050_vehicles:="${VEHICLE_ID}" \\
    -p vda5050_state_stale_sec:=30.0 \\
    > "\${STAMP}/dispatcher.log" 2>&1 &

for ((i = 0; i < 200; i++)); do
    if ros2 node list 2>/dev/null | grep -q "/dispatcher_node"; then break; fi
    sleep 0.1
done
ros2 lifecycle set /dispatcher_node configure >/dev/null 2>&1
ros2 lifecycle set /dispatcher_node activate >/dev/null 2>&1

ros2 run amr_dispatcher_ros vda5050_gateway_node --ros-args \\
    -p use_vda5050:=true -p mqtt_broker_host:=127.0.0.1 -p mqtt_port:=${TOPO_MQTT_PORT} \\
    -p manufacturer:=RobotManufacturer -p serial_number:=topo_gateway \\
    -p protocol_version:=v2 -p heartbeat_ms:=5000 \\
    > "\${STAMP}/gateway.log" 2>&1 &

ros2 run amr_dispatcher_tools rest_gateway --ros-args \\
    -p port:=${TOPO_REST_PORT} \\
    -p html_path:="${CONTAINER_WORKDIR}/tools/operator_console.html" \\
    > "\${STAMP}/rest.log" 2>&1 &

for ((i = 0; i < 200; i++)); do
    if bash -c "cat < /dev/null > /dev/tcp/127.0.0.1/${TOPO_REST_PORT}" 2>/dev/null; then
        echo READY > "\${STAMP}/READY"; break
    fi
    sleep 0.1
done
sleep 3600
INNER
docker cp /tmp/topo_stack.sh "${CONTAINER}:${STAMP_DIR}/stack.sh" >/dev/null

cat > /tmp/topo_vehicle.sh <<INNER
source /opt/ros/jazzy/setup.bash >/dev/null 2>&1
cd "${CONTAINER_WORKDIR}"
exec python3 scripts/mock_vda5050_vehicle.py --host 127.0.0.1 --port ${TOPO_MQTT_PORT} \\
    --manufacturer RobotManufacturer --serial ${VEHICLE_ID} --speed 0.6 --battery 90 \\
    --state-file "${STAMP_DIR}/vehicle_state.json"
INNER
docker cp /tmp/topo_vehicle.sh "${CONTAINER}:${STAMP_DIR}/vehicle.sh" >/dev/null

start_stack() {
    docker exec -d "${CONTAINER}" bash "${STAMP_DIR}/stack.sh"
    local ready=0
    for ((i = 0; i < 400; i++)); do
        if docker exec "${CONTAINER}" bash -c "test -f '${STAMP_DIR}/READY'" 2>/dev/null; then
            ready=1; break
        fi
        sleep 0.2
    done
    [ "${ready}" -eq 1 ] || fail "拓扑栈未在预期时间内就绪"
}

start_vehicle() {
    docker exec -d "${CONTAINER}" bash "${STAMP_DIR}/vehicle.sh"
}

# 等车辆上报 state（否则自动选车会停在 "no state report yet"）
# 等车辆上报 state。
# 判据用网关日志而不是 /api/v1/vehicle_links：后者的数据来自 /dispatcher/state（2Hz），
# 而 dispatcher 只把 **受管车** 填进 vehicle_links —— 车辆在链路表里尚未登记时，
# 该端点根本不包含这辆车（不是"position_valid=false"，而是"没有这一项"）。
# 网关日志 "发现车辆 <id>" 才是链路真正建立的第一手证据。
wait_vehicle_state() {
    local ok=0
    for ((i = 0; i < 200; i++)); do
        if docker exec "${CONTAINER}" bash -c \
                "grep -q '发现车辆 ${VEHICLE_ID}' '${STAMP_DIR}/gateway.log' 2>/dev/null"; then
            ok=1; break
        fi
        sleep 0.25
    done
    [ "${ok}" -eq 1 ] || fail "受管车 ${VEHICLE_ID} 未在 50s 内被网关发现（见 gateway.log）"
    # 再等车把首帧 state 推上来：dispatcher 侧"无状态"会暂停自动选车
    for ((i = 0; i < 200; i++)); do
        if docker exec "${CONTAINER}" bash -c \
                "grep -q 'state report' '${STAMP_DIR}/dispatcher.log' 2>/dev/null"; then
            break
        fi
        if docker exec "${CONTAINER}" bash -c \
                "grep -q 'VDA 5050 state' '${STAMP_DIR}/dispatcher.log' 2>/dev/null"; then
            break
        fi
        sleep 0.25
    done
    # 只要车在跑（state 文件有 header_id），就能被派单
    docker exec "${CONTAINER}" bash -c "test -s '${STAMP_DIR}/vehicle_state.json'" \
        || fail "受管车 ${VEHICLE_ID} 未落盘 state（模拟器未正常运行）"
}

REST="http://127.0.0.1:${TOPO_REST_PORT}"

# python 侧统一入口：所有 REST 调用与 JSON 断言都走这里，避免 shell 里拼 JSON
cat > /tmp/topo_api.py <<'PY'
import json
import sys
import urllib.error
import urllib.request

BASE = sys.argv[1]


def call(path, body=None, method=None):
    data = None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(
        BASE + path, data=data,
        method=method or ("GET" if body is None else "POST"),
        headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=60) as resp:
            return resp.status, json.loads(resp.read().decode())
    except urllib.error.HTTPError as exc:
        raw = exc.read().decode()
        try:
            return exc.code, json.loads(raw)
        except ValueError:
            return exc.code, {"raw": raw}


def main():
    op = sys.argv[2]
    args = json.loads(sys.argv[3]) if len(sys.argv) > 3 else {}
    if op == "overlay":
        status, body = call("/api/v1/topology/overlay?yaml=1")
    elif op == "preview":
        status, body = call("/api/v1/topology/preview", args)
    elif op == "hot_update":
        status, body = call("/api/v1/topology/hot_update", args)
    elif op == "block":
        status, body = call("/api/v1/topology/block_edge", args)
    elif op == "unblock":
        status, body = call("/api/v1/topology/unblock_edge", args)
    elif op == "submit":
        status, body = call("/api/v1/orders", {}, method="POST")
    elif op == "state":
        status, body = call("/api/v1/state")
    elif op == "metrics":
        req = urllib.request.Request(BASE + "/metrics")
        with urllib.request.urlopen(req, timeout=30) as resp:
            print(resp.read().decode())
            return
    else:
        raise SystemExit(f"unknown op {op}")
    # 注意：body 自己也有 "status"（ok/noop/rejected 语义字面量），
    # 直接 **body 会把 HTTP 状态码覆盖掉。HTTP 码单独放 "http_status"。
    print(json.dumps({"http_status": status, **body}, ensure_ascii=False))


main()
PY
docker cp /tmp/topo_api.py "${CONTAINER}:${STAMP_DIR}/api.py" >/dev/null

api() {
    docker exec "${CONTAINER}" python3 "${STAMP_DIR}/api.py" "${REST}" "$@"
}

start_stack
start_vehicle
wait_vehicle_state
echo "[TOPO] 栈就绪，车辆已上报 state (PASS)"

# ---------------------------------------------------------------------------
# T1：初始状态
# ---------------------------------------------------------------------------
echo ""
echo "[T1] 初始 revision 0、无叠加层文件"
OVERLAY_JSON="$(api overlay)"
echo "${OVERLAY_JSON}" | grep -q '"revision": *0' || fail "T1 初始 revision 非 0: ${OVERLAY_JSON}"
echo "${OVERLAY_JSON}" | grep -q '"overlay_active": *false' || fail "T1 初始不应有生效叠加层"
docker exec "${CONTAINER}" bash -c "test ! -f '${STAMP_DIR}/overlay.yaml'" \
    || fail "T1 初始不应存在叠加层文件"
echo "[T1] revision 0 且无叠加层文件 (PASS)"

# ---------------------------------------------------------------------------
# T12（提前做）：无叠加层时 /v2/list_stations 与基线逐字一致
# 必须在任何叠加层出现之前取基线，之后无法回溯
# ---------------------------------------------------------------------------
echo ""
echo "[T12] 无叠加层时 /v2/list_stations 与基线逐字一致"
# 采集必须在**容器内**完成并落盘到容器路径：STAMP_DIR 是容器内路径，
# 宿主机重定向到它必然失败（本脚本踩过：重定向失败被 `|| true` 吞掉，
# 于是 wc 读不到文件，在 `set -e` 下静默退出）。因此整段都在容器内跑。
list_stations_snapshot() {
    local out="$1"
    docker exec "${CONTAINER}" bash -c "
        set -o pipefail
        source /opt/ros/jazzy/setup.bash >/dev/null 2>&1
        source ${CONTAINER_WORKDIR}/install/setup.bash >/dev/null 2>&1
        export ROS_DOMAIN_ID=${TOPO_DOMAIN_ID}
        timeout 20 ros2 service call /v2/list_stations \
            amr_dispatcher_interfaces/srv/ListStations '{}' 2>/dev/null \
            | sed -n '/ListStations_Response(/,\$p' > '${out}'
        test -s '${out}'
    "
}
list_stations_snapshot "${STAMP_DIR}/stations_baseline.txt" \
    || fail "T12 未能取得 /v2/list_stations 基线（服务未响应？）"
BASELINE_LINES="$(docker exec "${CONTAINER}" bash -c "wc -l < '${STAMP_DIR}/stations_baseline.txt'" | tr -d '\r')"
[ "${BASELINE_LINES:-0}" -ge 1 ] || fail "T12 基线为空"
docker exec "${CONTAINER}" bash -c "grep -q \"'a'\" '${STAMP_DIR}/stations_baseline.txt'" \
    || fail "T12 基线投影里没有站点 a（服务未按预期返回）"
echo "[T12] 基线投影已保存（${BASELINE_LINES} 行，含站点 a）"

# ---------------------------------------------------------------------------
# T2：断连预览必须被拒且不写盘
# ---------------------------------------------------------------------------
echo ""
echo "[T2] 断连叠加层 preview -> 409 would_disconnect_topology，且不写盘"
PREVIEW_ENGINE="$(api preview "$(python3 - <<'PY'
import json
print(json.dumps({
    "overlay_yaml": 'blocked_edges:\n  - from: "d"\n    to: "e"\n',
    "operator_id": "op_e2e",
    "reason": "cable trench",
}))
PY
)")"
echo "${PREVIEW_ENGINE}" | grep -q '"http_status": *409' || fail "T2 断连预览未返回 409: ${PREVIEW_ENGINE}"
echo "${PREVIEW_ENGINE}" | grep -q '"error_code": *"would_disconnect_topology"' \
    || fail "T2 error_code 非 would_disconnect_topology: ${PREVIEW_ENGINE}"
echo "${PREVIEW_ENGINE}" | grep -q '"unreachable_nodes": *\[[^]]*"e"' \
    || fail "T2 未列出不可达站点 e: ${PREVIEW_ENGINE}"
echo "${PREVIEW_ENGINE}" | grep -q '"file_written": *false' || fail "T2 dry_run 不应写盘"
docker exec "${CONTAINER}" bash -c "test ! -f '${STAMP_DIR}/overlay.yaml'" \
    || fail "T2 预览被拒后仍写出了叠加层文件（拒绝时必须零写盘）"
echo "[T2] 409 + 不可达站点 e + 未写盘 (PASS)"

# ---------------------------------------------------------------------------
# T3：缺审计字段 -> 400 且不写盘
# ---------------------------------------------------------------------------
echo ""
echo "[T3] 缺 reason -> 400，且仍不写盘"
MISSING="$(api hot_update '{"overlay_yaml":"blocked_edges:\n  - from: \"b\"\n    to: \"c\"\n","operator_id":"op_e2e","confirm":true}')"
echo "${MISSING}" | grep -q '"http_status": *400' || fail "T3 缺 reason 未返回 400: ${MISSING}"
docker exec "${CONTAINER}" bash -c "test ! -f '${STAMP_DIR}/overlay.yaml'" \
    || fail "T3 400 之后仍写出了叠加层文件"
echo "[T3] 400 且未写盘 (PASS)"

# ---------------------------------------------------------------------------
# T4：合法封非桥边 b-c
# ---------------------------------------------------------------------------
echo ""
echo "[T4] 封非桥边 b-c -> 200、文件存在、revision 1、指标计数 1"
BLOCKED="$(api block '{"from_station":"b","to_station":"c","operator_id":"op_e2e","reason":"water main","confirm":true}')"
echo "${BLOCKED}" | grep -q '"http_status": *200' || fail "T4 封 b-c 未返回 200: ${BLOCKED}"
echo "${BLOCKED}" | grep -q '"overlay_revision": *1' || fail "T4 revision 非 1: ${BLOCKED}"
docker exec "${CONTAINER}" bash -c "test -f '${STAMP_DIR}/overlay.yaml'" \
    || fail "T4 提交成功后叠加层文件不存在"
docker exec "${CONTAINER}" bash -c "grep -q 'route_edge:b__c\|from: \"b\"' '${STAMP_DIR}/overlay.yaml'" \
    || fail "T4 叠加层文件里没有刚封的边"
sleep 3   # 指标来自 /dispatcher/state（2Hz），等一拍
api metrics | grep -qx 'amr_topology_blocked_edges 1' || fail "T4 指标 blocked_edges 非 1"
api metrics | grep -qx 'amr_topology_overlay_revision 1' || fail "T4 指标 revision 非 1"
echo "[T4] 200 + revision 1 + 文件落盘 + 指标一致 (PASS)"

# ---------------------------------------------------------------------------
# T5：重放同一请求 -> noop，revision 不变
# ---------------------------------------------------------------------------
echo ""
echo "[T5] 重放同一请求 -> 200 noop，revision 不变"
REPLAY="$(api block '{"from_station":"b","to_station":"c","operator_id":"op_e2e","reason":"water main","confirm":true}')"
echo "${REPLAY}" | grep -q '"http_status": *200' || fail "T5 重放未返回 200: ${REPLAY}"
echo "${REPLAY}" | grep -q '"status": *"noop"' || fail "T5 重放未返回 noop（幂等失效）: ${REPLAY}"
echo "${REPLAY}" | grep -q '"overlay_revision": *1' || fail "T5 重放后 revision 变了: ${REPLAY}"
echo "[T5] 幂等：noop 且 revision 保持 1 (PASS)"

# ---------------------------------------------------------------------------
# T6：错误的 expected_map_hash -> 409
# ---------------------------------------------------------------------------
echo ""
echo "[T6] 错误 expected_map_hash -> 409 map_hash_mismatch"
BADHASH="$(api block '{"from_station":"c","to_station":"d","operator_id":"op_e2e","reason":"x","confirm":true,"expected_map_hash":"fnv1a64_deadbeef"}')"
echo "${BADHASH}" | grep -q '"http_status": *409' || fail "T6 错误 hash 未返回 409: ${BADHASH}"
echo "${BADHASH}" | grep -q '"error_code": *"map_hash_mismatch"' || fail "T6 error_code 非 map_hash_mismatch: ${BADHASH}"
echo "${BADHASH}" | grep -q '"overlay_revision": *1' || fail "T6 被拒后 revision 变了: ${BADHASH}"
echo "[T6] 409 map_hash_mismatch，revision 不变 (PASS)"

# ---------------------------------------------------------------------------
# T7：提单 -> 下发的 VDA 订单真的绕开了被封边
# ---------------------------------------------------------------------------
echo ""
echo "[T7] 提单 a->e -> 下发的 VDA 订单不含 b、改走旁路 n1"
# b-c 被封后主链断开，a->e 只能走 a-n1-d-e（5+5+2 = 12m）。
# 断言对象必须是**车辆实收的订单**，而不是 dispatcher 的日志行：
# 那条 "VDA 5050 order published: ... (orderId=..., N nodes, M edges, X bytes)"
# 只有节点计数、没有节点 id，用 grep 找 n1 永远匹配不到（本脚本踩过这个坑）。
# 模拟车把收到的原始订单原样写进 vehicle_state.json 的 raw_order，
# 那是"车实际收到了什么"的唯一真源。
# /api/v1/orders 的入参在 query string 上（id/from/to），body 可为空对象。
# 用容器内的独立小脚本发请求：避免在 $( ) 里嵌 heredoc（可读性与转义都更稳）。
cat > /tmp/topo_submit.py <<'PYSUBMIT'
import json
import sys
import urllib.error
import urllib.request

base, query = sys.argv[1], sys.argv[2]
req = urllib.request.Request(
    f"{base}/api/v1/orders?{query}", data=b"{}", method="POST",
    headers={"Content-Type": "application/json"})
try:
    with urllib.request.urlopen(req, timeout=30) as resp:
        print(json.dumps({"http_status": resp.status, **json.loads(resp.read().decode())},
                         ensure_ascii=False))
except urllib.error.HTTPError as exc:
    print(json.dumps({"http_status": exc.code, "body": exc.read().decode()}, ensure_ascii=False))
PYSUBMIT
docker cp /tmp/topo_submit.py "${CONTAINER}:${STAMP_DIR}/submit.py" >/dev/null
SUBMIT="$(docker exec "${CONTAINER}" python3 "${STAMP_DIR}/submit.py" "${REST}" \
    'id=m_topo&from=a&to=e&idempotency_key=k_topo')"
echo "${SUBMIT}" | grep -q '"accepted": *true' || fail "T7 提单未被受理: ${SUBMIT}"

# 等派发（要等 state 到达 + 选车 + 发单）
DISPATCHED=0
for ((i = 0; i < 120; i++)); do
    if docker exec "${CONTAINER}" bash -c "grep -q 'Dispatched mission: m_topo' '${STAMP_DIR}/dispatcher.log'"; then
        DISPATCHED=1; break
    fi
    sleep 0.25
done
[ "${DISPATCHED}" -eq 1 ] || fail "T7 任务 m_topo 未在 30s 内派发"

# 从车辆状态文件读实收订单的节点序列。模拟车在收到订单时立即落盘，
# 之后每 0.5s 刷一次；轮询到 orderId 对上为止。
cat > /tmp/topo_order_nodes.py <<'PYNODES'
import json
import sys

path, want = sys.argv[1], sys.argv[2]
try:
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
except (OSError, ValueError):
    print("")
    raise SystemExit(0)
raw = data.get("raw_order") or {}
if raw.get("orderId") != want:
    print("")
    raise SystemExit(0)
print(" ".join(n.get("nodeId", "") for n in raw.get("nodes", [])))
PYNODES
docker cp /tmp/topo_order_nodes.py "${CONTAINER}:${STAMP_DIR}/order_nodes.py" >/dev/null

NODES=""
for ((i = 0; i < 40; i++)); do
    NODES="$(docker exec "${CONTAINER}" python3 "${STAMP_DIR}/order_nodes.py" \
        "${STAMP_DIR}/vehicle_state.json" "m_topo" 2>/dev/null | tr -d '\r')"
    [ -n "${NODES}" ] && break
    sleep 0.25
done
[ -n "${NODES}" ] || fail "T7 车辆未收到 m_topo 的订单（raw_order 为空或仍是旧单）"
echo "[T7] 车辆实收订单节点序列: ${NODES}"

# 双条件：不含被封的 b，且必须经过旁路 n1。
# 只看"不含 b"不够（路径畸形时也可能不含 b）；只看"含 n1"也不够
# （无叠加层时主链更短，本来就不该含 n1）。两者同时成立才唯一证明
# "叠加层生效到了派发路径"。
case " ${NODES} " in
    *" b "*) fail "T7 订单里仍有被封的 b（叠加层未生效到派发路径）: ${NODES}" ;;
esac
case " ${NODES} " in
    *" n1 "*) : ;;
    *) fail "T7 订单未经过旁路 n1（路径没换）: ${NODES}" ;;
esac
case " ${NODES} " in
    *" c "*) fail "T7 订单里仍有被封的 c（b-c 两侧都该绕开）: ${NODES}" ;;
esac
echo "[T7] 派发路径已避开被封边，改走旁路 n1 (PASS)"

# ---------------------------------------------------------------------------
# T8：在途车正压在将要封的边上 -> 409，且没有收到新订单
# ---------------------------------------------------------------------------
echo ""
echo "[T8] 在途车压在将要封的边上 -> 409 edge_occupied_in_flight"
# T7 的订单是 a-n1-d-e，车刚起步（0.6m/s，a-n1 = 5m 要 8.3s）：
# 此刻 lastNodeId 仍是 a，但物理上正走 a-n1 —— 封这条边就是从车轮下抽路。
# 不额外 sleep：拖久了车会到 n1，那时 a-n1 已不在剩余航段里，
# 请求会变成 kUnaffected 而合法通过（那是另一个正确行为，测不到本用例）。
UNDER="$(api block '{"from_station":"a","to_station":"n1","operator_id":"op_e2e","reason":"under vehicle","confirm":true}')"
echo "${UNDER}" | grep -q '"http_status": *409' || fail "T8 封在途车所在边未返回 409: ${UNDER}"
echo "${UNDER}" | grep -q '"error_code": *"edge_occupied_in_flight"' \
    || fail "T8 error_code 非 edge_occupied_in_flight: ${UNDER}"
echo "${UNDER}" | grep -q "\"${VEHICLE_ID}\"" || fail "T8 拒绝时未回显车辆 id: ${UNDER}"
echo "${UNDER}" | grep -q '"overlay_revision": *1' || fail "T8 被拒后 revision 变了: ${UNDER}"
# 关键：被拒不能导致半路改道 —— 订单下发次数不应增加
ORDERS_BEFORE="$(docker exec "${CONTAINER}" bash -c "grep -c 'order published: mission m_topo' '${STAMP_DIR}/dispatcher.log' || true" | tr -d '\r')"
sleep 1
ORDERS_AFTER="$(docker exec "${CONTAINER}" bash -c "grep -c 'order published: mission m_topo' '${STAMP_DIR}/dispatcher.log' || true" | tr -d '\r')"
[ "${ORDERS_BEFORE}" = "${ORDERS_AFTER}" ] \
    || fail "T8 被拒后仍给在途车下发了新订单 (${ORDERS_BEFORE} -> ${ORDERS_AFTER})"
echo "[T8] 409 + 车辆 id + 无半路改道 (PASS)"

# 让在途任务跑完，避免它干扰后续断言
for ((i = 0; i < 240; i++)); do
    if docker exec "${CONTAINER}" bash -c "grep -q 'Mission completed: m_topo\|Mission failed: m_topo\|CANCELED' '${STAMP_DIR}/dispatcher.log'"; then
        break
    fi
    sleep 0.25
done

# ---------------------------------------------------------------------------
# T9：kill + 重启 -> 叠加层从文件回放
# ---------------------------------------------------------------------------
echo ""
echo "[T9] 重启 dispatcher -> 叠加层从文件回放，预约表仍见 overlay owner"
DISP_PID="$(docker exec "${CONTAINER}" bash -c "
    for envf in /proc/[0-9]*/environ; do
        pid=\"\${envf#/proc/}\"; pid=\"\${pid%/environ}\"
        [ -r \"\${envf}\" ] || continue
        if tr '\\0' '\\n' 2>/dev/null < \"\${envf}\" | grep -qx 'ROS_DOMAIN_ID=${TOPO_DOMAIN_ID}'; then
            if tr '\\0' ' ' 2>/dev/null < /proc/\$pid/cmdline | grep -q 'lib/amr_dispatcher_ros/dispatcher_node'; then
                echo \$pid; exit 0
            fi
        fi
    done
" | tr -d '\r')"
[ -n "${DISP_PID}" ] || fail "T9 未找到 dispatcher 进程"
docker exec "${CONTAINER}" bash -c "kill -9 ${DISP_PID}" >/dev/null 2>&1 || true
sleep 1

# 重启前必须先停掉容器内的 ros2 daemon：它缓存了旧进程的节点图，
# 会让下一次 `ros2 node list` 立刻返回**已死的** /dispatcher_node，
# 于是生命周期 configure/activate 打给一个不存在的节点（静默失败），
# 叠加层永远不会被加载 —— 本脚本踩过这个坑（表现为"日志里没有 Loaded"，
# 但其实进程启动正常，只是从不 configure）。
docker exec "${CONTAINER}" bash -c "
    export ROS_DOMAIN_ID=${TOPO_DOMAIN_ID}
    source /opt/ros/jazzy/setup.bash >/dev/null 2>&1
    ros2 daemon stop >/dev/null 2>&1 || true
" >/dev/null 2>&1 || true

# 只重启 dispatcher（不重启整栈：叠加层回放要验证的是它自己从磁盘读的能力）
docker exec -d "${CONTAINER}" bash -c "
    export ROS_DOMAIN_ID=${TOPO_DOMAIN_ID}
    source /opt/ros/jazzy/setup.bash >/dev/null 2>&1
    source ${CONTAINER_WORKDIR}/install/setup.bash >/dev/null 2>&1
    exec ros2 run amr_dispatcher_ros dispatcher_node --ros-args \
        -p stations_file:='${STAMP_DIR}/stations.yaml' \
        -p planner_type:=skeleton \
        -p expected_map_hash:='${MAP_HASH}' \
        -p topology_overlay_file:='${STAMP_DIR}/overlay.yaml' \
        -p audit_log_file:='${STAMP_DIR}/audit.jsonl' \
        -p vda5050_vehicles:='${VEHICLE_ID}' \
        -p vda5050_state_stale_sec:=30.0 \
        >> '${STAMP_DIR}/dispatcher.log' 2>&1
"

# 等新进程真的出现在节点图上（daemon 已停，这里是第一手发现），再 configure/activate。
# 叠加层是在 on_configure 里加载的，所以必须等 configure 完成后日志才会出现。
ACTIVATED=0
for ((i = 0; i < 200; i++)); do
    if docker exec "${CONTAINER}" bash -c "
        export ROS_DOMAIN_ID=${TOPO_DOMAIN_ID}
        source /opt/ros/jazzy/setup.bash >/dev/null 2>&1
        source ${CONTAINER_WORKDIR}/install/setup.bash >/dev/null 2>&1
        ros2 node list 2>/dev/null | grep -q '^/dispatcher_node$'
    "; then
        docker exec "${CONTAINER}" bash -c "
            export ROS_DOMAIN_ID=${TOPO_DOMAIN_ID}
            source /opt/ros/jazzy/setup.bash >/dev/null 2>&1
            source ${CONTAINER_WORKDIR}/install/setup.bash >/dev/null 2>&1
            ros2 lifecycle set /dispatcher_node configure >/dev/null 2>&1
            ros2 lifecycle set /dispatcher_node activate >/dev/null 2>&1
        " >/dev/null 2>&1 || true
        if docker exec "${CONTAINER}" bash -c "grep -q 'Loaded topology overlay (rev=1' '${STAMP_DIR}/dispatcher.log'"; then
            ACTIVATED=1; break
        fi
    fi
    sleep 0.5
done
[ "${ACTIVATED}" -eq 1 ] \
    || fail "T9 重启后未从磁盘回放叠加层（新的 dispatcher 未被 configure，或日志无 'Loaded topology overlay (rev=1'）"
sleep 3
RESV="$(docker exec "${CONTAINER}" bash -c "
    source /opt/ros/jazzy/setup.bash >/dev/null 2>&1
    source ${CONTAINER_WORKDIR}/install/setup.bash >/dev/null 2>&1
    ROS_DOMAIN_ID=${TOPO_DOMAIN_ID} timeout 15 ros2 service call /v2/list_traffic_reservations \
        amr_dispatcher_interfaces/srv/ListTrafficReservations '{}' 2>/dev/null | tail -n +2
" 2>/dev/null || true)"
echo "${RESV}" | grep -q 'route_edge:b__c' || fail "T9 重启后预约表里没有 b-c 的封路锁: ${RESV}"
echo "${RESV}" | grep -q 'traffic_block:topology_overlay:' \
    || fail "T9 重启后封路锁 owner 不是 topology_overlay（叠加层未反映到预约表）: ${RESV}"
echo "[T9] 文件回放 + 预约表 owner=traffic_block:topology_overlay:* (PASS)"

# ---------------------------------------------------------------------------
# T10：基础地图换版 -> 叠加层失效归档
# ---------------------------------------------------------------------------
echo ""
echo "[T10] 基础地图 map_hash 变化 -> 叠加层失效、归档 .stale、审计留痕"
docker exec "${CONTAINER}" bash -c "
    sed -i 's/fnv1a64_topoe2e0000abcd/fnv1a64_topoe2e9999ffff/' '${STAMP_DIR}/stations.yaml'
    grep map_hash '${STAMP_DIR}/stations.yaml'
"

# 等 mtime 热重载（reload_timer_ 1Hz）+ 失效处理
INVALIDATED=0
for ((i = 0; i < 60; i++)); do
    if docker exec "${CONTAINER}" bash -c "ls '${STAMP_DIR}'/overlay.yaml.stale.* >/dev/null 2>&1"; then
        INVALIDATED=1; break
    fi
    sleep 0.5
done
[ "${INVALIDATED}" -eq 1 ] || fail "T10 基础地图换版后未出现 .stale 归档件"
docker exec "${CONTAINER}" bash -c "test ! -f '${STAMP_DIR}/overlay.yaml'" \
    || fail "T10 失效后生效叠加层文件应已被改名归档"
docker exec "${CONTAINER}" bash -c "grep -q 'topology_overlay_invalidated' '${STAMP_DIR}/audit.jsonl' 2>/dev/null" \
    || fail "T10 审计文件里没有 topology_overlay_invalidated"
echo "[T10] 归档 .stale + 审计留痕 (PASS)"

# 失效后当前叠加层应为空：重新提交同一条边不应再被当成"已生效"
OVERLAY_AFTER="$(api overlay)"
echo "${OVERLAY_AFTER}" | grep -q '"blocked_edges": *0' \
    || fail "T10 失效后叠加层仍报有封路边: ${OVERLAY_AFTER}"
echo "[T10] 失效后 blocked_edges=0 (PASS)"

# ---------------------------------------------------------------------------
# T11：unblock -> 计数归零、metrics 归零、预约表清空
# ---------------------------------------------------------------------------
echo ""
echo "[T11] unblock_edge -> 计数与预约表归零"
# T10 把叠加层清空了，这里重新封一条再解封，验证 unblock 的真实删除路径
api block '{"from_station":"b","to_station":"c","operator_id":"op_e2e","reason":"re-block","confirm":true,"expected_map_hash":"fnv1a64_topoe2e9999ffff"}' \
    | grep -q '"http_status": *200' || fail "T11 失效后重新封路失败（地图版本已换，应带新 hash）"
UNBLOCK="$(api unblock '{"from_station":"b","to_station":"c","operator_id":"op_e2e","reason":"work complete","confirm":true,"expected_map_hash":"fnv1a64_topoe2e9999ffff"}')"
echo "${UNBLOCK}" | grep -q '"http_status": *200' || fail "T11 解封未返回 200: ${UNBLOCK}"
OVERLAY_FINAL="$(api overlay)"
echo "${OVERLAY_FINAL}" | grep -q '"blocked_edges": *0' || fail "T11 解封后仍有封路边: ${OVERLAY_FINAL}"
sleep 3
api metrics | grep -qx 'amr_topology_blocked_edges 0' || fail "T11 指标 blocked_edges 未归零"
RESV_FINAL="$(docker exec "${CONTAINER}" bash -c "
    source /opt/ros/jazzy/setup.bash >/dev/null 2>&1
    source ${CONTAINER_WORKDIR}/install/setup.bash >/dev/null 2>&1
    ROS_DOMAIN_ID=${TOPO_DOMAIN_ID} timeout 15 ros2 service call /v2/list_traffic_reservations \
        amr_dispatcher_interfaces/srv/ListTrafficReservations '{}' 2>/dev/null | tail -n +2
" 2>/dev/null || true)"
if echo "${RESV_FINAL}" | grep -q 'traffic_block:topology_overlay:'; then
    fail "T11 解封后预约表仍留有 overlay 封路锁: ${RESV_FINAL}"
fi
echo "[T11] 计数/metrics/预约表全部归零 (PASS)"

# ---------------------------------------------------------------------------
# T12（收尾）：叠加层清空后 /v2/list_stations 与基线一致
# ---------------------------------------------------------------------------
echo ""
echo "[T12] 清空叠加层后 /v2/list_stations 与基线逐字一致"
list_stations_snapshot "${STAMP_DIR}/stations_after.txt" \
    || fail "T12 未能取得叠加层清空后的 /v2/list_stations 投影"

# 投影只含站点坐标（叠加层不新增站点时必然一致）。逐字比较会受 revision 之类
# 外部字段影响，因此比的是"服务响应正文"——它只由目录投影决定。
if ! docker exec "${CONTAINER}" bash -c "diff -q '${STAMP_DIR}/stations_baseline.txt' '${STAMP_DIR}/stations_after.txt' >/dev/null"; then
    echo "--- baseline ---"
    docker exec "${CONTAINER}" bash -c "cat '${STAMP_DIR}/stations_baseline.txt'"
    echo "--- after ---"
    docker exec "${CONTAINER}" bash -c "cat '${STAMP_DIR}/stations_after.txt'"
    fail "T12 清空叠加层后 /v2/list_stations 与基线不一致（legacy 红线）"
fi
echo "[T12] 与基线逐字一致 (PASS)"

echo ""
echo "======================================================================"
echo " [PASS] 在线拓扑热生效端到端验收全部通过:"
echo "   T1  初始 revision 0、无叠加层文件"
echo "   T2  断连预览 409 would_disconnect_topology + 不写盘"
echo "   T3  缺 reason 400 + 不写盘"
echo "   T4  封非桥边 200 + 文件落盘 + revision 1 + 指标一致"
echo "   T5  重放 -> noop，revision 不变（幂等）"
echo "   T6  错误 map_hash -> 409，revision 不变"
echo "   T7  下发订单不含被封的 b、改走旁路 n1"
echo "   T8  车压在边上 -> 409 edge_occupied_in_flight + 无半路改道"
echo "   T9  重启后从文件回放 + 预约表 owner=traffic_block:topology_overlay:*"
echo "   T10 地图换版 -> 失效归档 .stale + 审计 topology_overlay_invalidated"
echo "   T11 解封 -> 计数/metrics/预约表归零"
echo "   T12 无叠加层时 /v2/list_stations 与基线逐字一致（legacy 红线）"
echo "======================================================================"

exit 0
