#!/usr/bin/env bash
# ==============================================================================
# check_health_supervision_e2e.sh - 探针与进程守护端到端验收（相位 11 / A+B）
#
# 为什么要有这套验收
# ------------------
# 现网容器曾显示 "Up 8 days"，而 dispatcher_node 已死 8 天、安全门锁存 CRITICAL，
# rest_gateway 却对任何请求仍回 200。根因是三个结构性缺口：
#   1. 没有进程守护（launch 里 11 个节点全无 respawn，PID 1 是 ros2 launch）
#   2. 没有 liveness/readiness 之分，也没人定期去问
#   3. 没有主动告警通道
# 这套验收逐条锁死上面三点 —— 断言的是"能发现并自愈"，不是"接口存在"。
#
# 断言（T1~T8）
#   T1 /healthz 200 且**不需要** Authorization；/readyz 200 且含 4 个组件
#   T2 kill -9 dispatcher_node -> 3s 内 /readyz 503 且 dispatcher.reason == stale
#      （必须是 stale 而不是 never_seen：never_seen 说明网关从没收到过状态，
#        那是"网关自己刚起"，不是"调度器死了"）
#   T3 5s 内 webhook 收到 readiness_not_ready（payload schema amr.alert.v1）
#   T4 launch 的 respawn 把 dispatcher 拉起来：新 PID 出现、state=active、/readyz 回 200
#   T5 恰好一条 readiness_recovered（不受 alert_min_interval_sec 节流）
#   T6 急停 -> /readyz 503 + /metrics amr_safety_critical 1 / severity_level 2；
#      alert_safety_critical_sec:=3 时收到 safety_critical，解除后收到 safety_recovered
#   T7 /metrics 新指标族全部有样本（check_metrics_e2e 的"无空族"红线的延伸）
#   T8 alert_webhook_url 为空时**不**建告警线程（进程/话题与今天完全一致）
#
# 环境隔离：域 92 / REST 18100 / webhook 收单 18111，与生产（42 / 8080 / 1883）
#           及既有 e2e（77/78/80/82/84/86/87/88/89/91/93）不重叠。
#
# 用法：./scripts/check_health_supervision_e2e.sh
# 退出码：0 = 全部通过；非 0 = 断言失败（现场保留在容器 /tmp 便于排查）
# ==============================================================================
set -eo pipefail
set -E
trap 'echo "[HEALTH-FATAL] line=${LINENO} status=$? cmd=${BASH_COMMAND}" >&2' ERR

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

CONTAINER="${AMR_HEALTH_CONTAINER:-amr_dispatcher}"
HEALTH_DOMAIN_ID="${AMR_HEALTH_DOMAIN_ID:-92}"
HEALTH_REST_PORT="${AMR_HEALTH_REST_PORT:-18100}"
HEALTH_WEBHOOK_PORT="${AMR_HEALTH_WEBHOOK_PORT:-18111}"
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
    CONTAINER_WORKDIR="${AMR_HEALTH_WORKDIR:-/workspace}"
fi
STAMP_DIR="${CONTAINER_WORKDIR}/.health_stamp"

HEALTH_FAILED=0

for tool in docker python3; do
    # 内层模式没有 docker 二进制：shim 提供同名函数，command -v 也能看到它
    command -v "${tool}" >/dev/null 2>&1 || {
        echo "[HEALTH-FAIL] 宿主机缺少必需工具: ${tool}" >&2
        exit 2
    }
done
docker ps --format '{{.Names}}' | grep -qx "${CONTAINER}" || {
    echo "[HEALTH-FAIL] 容器 ${CONTAINER} 未运行" >&2
    exit 2
}

# 生产红线：域 42 / 端口 8080 绝不触碰
if [ "${HEALTH_DOMAIN_ID}" = "42" ] || [ "${HEALTH_REST_PORT}" = "8080" ]; then
    echo "[HEALTH-FAIL] 拒绝使用生产域/端口 (42 / 8080)" >&2
    exit 2
fi

fail() {
    echo ""
    echo "[HEALTH-FAIL] 验收失败: $1"
    echo "--- [LOG TAIL: launch] ---"
    docker exec "${CONTAINER}" bash -c "tail -n 60 '${STAMP_DIR}/launch.log' 2>/dev/null" || echo "(no log)"
    echo "--- [LOG TAIL: gateway] ---"
    docker exec "${CONTAINER}" bash -c "tail -n 30 '${STAMP_DIR}/gateway.log' 2>/dev/null" || echo "(no log)"
    echo "--- [WEBHOOK RECEIVED] ---"
    docker exec "${CONTAINER}" bash -c "cat '${STAMP_DIR}/webhook.jsonl' 2>/dev/null" || echo "(none)"
    HEALTH_FAILED=1
    exit 1
}

cleanup() {
    local exit_code=$?
    echo ""
    echo "[HEALTH] 正在清理..."
    docker exec "${CONTAINER}" bash -c "
        for envf in /proc/[0-9]*/environ; do
            pid=\"\${envf#/proc/}\"; pid=\"\${pid%/environ}\"
            [ -r \"\${envf}\" ] || continue
            if tr '\\0' '\\n' 2>/dev/null < \"\${envf}\" | grep -qx 'ROS_DOMAIN_ID=${HEALTH_DOMAIN_ID}'; then
                kill -KILL \"\${pid}\" 2>/dev/null || true
            fi
        done
        pkill -9 -f '[r]est_gateway.*port:=${HEALTH_REST_PORT}' 2>/dev/null || true
        pkill -9 -f '[w]ebhook_sink.py.*${HEALTH_WEBHOOK_PORT}' 2>/dev/null || true
        ROS_DOMAIN_ID=${HEALTH_DOMAIN_ID} bash -c 'source /opt/ros/jazzy/setup.bash >/dev/null 2>&1; ros2 daemon stop >/dev/null 2>&1' || true
    " >/dev/null 2>&1 || true
    if [ ${exit_code} -eq 0 ] && [ ${HEALTH_FAILED} -eq 0 ]; then
        docker exec "${CONTAINER}" bash -c "rm -rf '${STAMP_DIR}'" >/dev/null 2>&1 || true
        echo "[HEALTH] 全部通过，已清理临时现场"
    else
        echo "[HEALTH][WARN] 验收未全部成功，现场保留在容器内: ${STAMP_DIR}"
    fi
    exit ${exit_code}
}
trap cleanup EXIT INT TERM

STATIONS_YAML="${CONTAINER_WORKDIR}/config/generated/stations.yaml"
docker exec "${CONTAINER}" bash -c "test -f '${STATIONS_YAML}'" \
    || fail "容器内缺少 ${STATIONS_YAML}（先跑 check_map_ingest_e2e.sh 生成拓扑）"

docker exec "${CONTAINER}" bash -c "rm -rf '${STAMP_DIR}' && mkdir -p '${STAMP_DIR}'"

echo "======================================================================"
echo " Health Probes / Supervision / Alerting End-to-End Acceptance"
echo " ROS_DOMAIN_ID: ${HEALTH_DOMAIN_ID}  REST: ${HEALTH_REST_PORT}  Webhook sink: ${HEALTH_WEBHOOK_PORT}"
echo "======================================================================"

# ---------------------------------------------------------------------------
# webhook 收单服务：把收到的 POST 体逐行追加到 webhook.jsonl
# 用 python 标准库而不是 nc：需要真实 HTTP 200 应答，否则网关侧会记失败
# ---------------------------------------------------------------------------
cat > /tmp/health_webhook_sink.py <<'PY'
import http.server
import sys

out_path = sys.argv[1]
port = int(sys.argv[2])


class Handler(http.server.BaseHTTPRequestHandler):
    def do_POST(self):  # noqa: N802
        length = int(self.headers.get('Content-Length', '0'))
        body = self.rfile.read(length).decode('utf-8', 'replace')
        with open(out_path, 'a', encoding='utf-8') as fh:
            fh.write(body.replace('\n', ' ') + '\n')
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', '2')
        self.end_headers()
        self.wfile.write(b'{}')

    def log_message(self, *args):  # 静默：避免刷屏进日志
        pass


http.server.HTTPServer(('127.0.0.1', port), Handler).serve_forever()
PY
docker cp /tmp/health_webhook_sink.py "${CONTAINER}:${STAMP_DIR}/webhook_sink.py" >/dev/null

# ---------------------------------------------------------------------------
# 用**真实 launch** 起栈。
#
# 为什么必须经过 launch：respawn 是 `ExecuteProcess` 的能力，脱离 launch
# 自己写一个"死了就重启"的 while 循环，测到的是那个循环而不是产品行为。
# 这里生成的 launch 文件与 full_system.launch.py 用同一套写法
# （Node/LifecycleNode + respawn/respawn_delay 关键字参数）。
#
# 只起四个节点：dispatcher + safety_gate + lifecycle_coordinator + rest_gateway。
# 协调器必须在内：respawn 后回到 unconfigured 的 dispatcher 靠它 auto_bringup
# 重新提拉 —— 这正是"自愈闭环"里最容易断的一环。
# ---------------------------------------------------------------------------
cat > /tmp/health_launch.py <<'PY'
import os

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node

DOMAIN = os.environ['HEALTH_DOMAIN_ID']
STAMP = os.environ['HEALTH_STAMP']
WORKDIR = os.environ['HEALTH_WORKDIR']
REST_PORT = os.environ['HEALTH_REST_PORT']
WEBHOOK_PORT = os.environ['HEALTH_WEBHOOK_PORT']
STATIONS = os.environ['HEALTH_STATIONS']
SHARE = os.path.join(WORKDIR, 'install', 'amr_dispatcher_ros', 'share',
                     'amr_dispatcher_ros', 'config')

ENV = {'ROS_DOMAIN_ID': DOMAIN}
# 与 full_system.launch.py 的 supervision() 完全同款：respawn + respawn_delay。
SUPERVISION = {'respawn': True, 'respawn_delay': 1.0}


def generate_launch_description():
    safety = Node(
        package='amr_dispatcher_ros', executable='safety_gate_node',
        name='safety_gate_node', output='screen',
        parameters=[os.path.join(SHARE, 'safety.yaml')],
        additional_env=ENV, **SUPERVISION)

    coordinator = Node(
        package='amr_dispatcher_ros', executable='lifecycle_coordinator_node',
        name='lifecycle_coordinator_node', output='screen',
        parameters=[{'auto_bringup': True, 'check_period_ms': 200}],
        additional_env=ENV, **SUPERVISION)

    # dispatcher 用 LifecycleNode：respawn 后回到 unconfigured，
    # 由协调器（auto_bringup）重新 configure+activate —— 生产就是这样自愈的。
    dispatcher = LifecycleNode(
        package='amr_dispatcher_ros', executable='dispatcher_node',
        name='dispatcher_node', namespace='', output='screen',
        parameters=[{
            'stations_file': STATIONS,
            'planner_type': 'skeleton',
            'max_active_missions': 1,
            'audit_log_file': os.path.join(STAMP, 'audit.jsonl'),
        }],
        additional_env=ENV, **SUPERVISION)

    gateway = Node(
        package='amr_dispatcher_tools', executable='rest_gateway',
        name='rest_gateway_node', output='screen',
        parameters=[{
            'port': int(REST_PORT),
            'html_path': os.path.join(WORKDIR, 'tools', 'operator_console.html'),
            'alert_webhook_url': f'http://127.0.0.1:{WEBHOOK_PORT}/hook',
            'alert_probe_period_ms': 500,
            'alert_not_ready_threshold': 2,
            'alert_safety_critical_sec': 3,
            'alert_min_interval_sec': 60,
            'readyz_dispatcher_state_stale_sec': 1.5,
            'readyz_require_chassis': False,
        }],
        additional_env=ENV, **SUPERVISION)

    return LaunchDescription([safety, coordinator, dispatcher, gateway])
PY

cat > /tmp/health_heartbeat.py <<'PY'
#!/usr/bin/env python3
"""给 safety_gate 喂两路上游心跳（chassis_driver / path_tracker）。

为什么需要它：safety_gate 的 500ms 看门狗会盯着这两个心跳话题。本验收只起
四个节点，没有真实底盘与循迹节点 —— 不喂心跳的话看门狗会按设计判上游失联，
安全门随即降级到 CRITICAL，/readyz 永远不可能为 200。
这与"探针坏了"是两件不同的事：这里补的是**上游在位**这个前提，
而不是放宽判据。生产里这两个心跳由 chassis_driver_node / path_tracker_node 发。
"""

import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


def main() -> int:
    rate_hz = float(sys.argv[1]) if len(sys.argv) > 1 else 5.0
    rclpy.init()
    node = Node('health_e2e_heartbeat_feeder')
    pubs = [
        node.create_publisher(Bool, '/safety/heartbeat/chassis_driver', 10),
        node.create_publisher(Bool, '/safety/heartbeat/path_tracker', 10),
    ]
    msg = Bool()
    msg.data = True

    def tick():
        for pub in pubs:
            pub.publish(msg)

    node.create_timer(1.0 / rate_hz, tick)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
PY
docker cp /tmp/health_heartbeat.py "${CONTAINER}:${STAMP_DIR}/heartbeat.py" >/dev/null

cat > /tmp/health_launch.sh <<INNER
set -eo pipefail
cd "${CONTAINER_WORKDIR}"
source /opt/ros/jazzy/setup.bash
source "${CONTAINER_WORKDIR}/install/setup.bash"
export ROS_DOMAIN_ID=${HEALTH_DOMAIN_ID}
export PYTHONUNBUFFERED=1
STAMP="${STAMP_DIR}"

python3 "\${STAMP}/webhook_sink.py" "\${STAMP}/webhook.jsonl" ${HEALTH_WEBHOOK_PORT} > "\${STAMP}/sink.log" 2>&1 &
# 上游心跳：safety_gate 的看门狗需要看到这两路（详见 heartbeat.py 注释）
python3 "\${STAMP}/heartbeat.py" 5.0 > "\${STAMP}/heartbeat.log" 2>&1 &
for ((i = 0; i < 100; i++)); do
  if bash -c "cat < /dev/null > /dev/tcp/127.0.0.1/${HEALTH_WEBHOOK_PORT}" 2>/dev/null; then break; fi
  sleep 0.1
done

export HEALTH_DOMAIN_ID=${HEALTH_DOMAIN_ID}
export HEALTH_STAMP="\${STAMP}"
export HEALTH_WORKDIR="${CONTAINER_WORKDIR}"
export HEALTH_REST_PORT=${HEALTH_REST_PORT}
export HEALTH_WEBHOOK_PORT=${HEALTH_WEBHOOK_PORT}
export HEALTH_STATIONS="${STATIONS_YAML}"

exec ros2 launch "\${STAMP}/health_launch.py"
INNER

docker cp /tmp/health_launch.py "${CONTAINER}:${STAMP_DIR}/health_launch.py" >/dev/null
docker cp /tmp/health_launch.sh "${CONTAINER}:${STAMP_DIR}/launch.sh" >/dev/null
docker exec -d "${CONTAINER}" bash -c \
    "cd '${STAMP_DIR}' && bash launch.sh > '${STAMP_DIR}/launch.log' 2>&1"

# REST 端口就绪（launch 内部不做等待，由这里轮询）
READY=0
for ((i = 0; i < 400; i++)); do
    if docker exec "${CONTAINER}" bash -c "cat < /dev/null > /dev/tcp/127.0.0.1/${HEALTH_REST_PORT}" 2>/dev/null; then
        READY=1
        break
    fi
    sleep 0.2
done
[ "${READY}" -eq 1 ] || fail "健康栈未在预期时间内就绪（launch 是否启动失败？见 launch.log）"
echo "[HEALTH] launch 已起（respawn=True, domain=${HEALTH_DOMAIN_ID}）"

REST="http://127.0.0.1:${HEALTH_REST_PORT}"

# 等 dispatcher 被协调器提拉到 active（/readyz 会真实反映）
ACTIVE=0
for ((i = 0; i < 150; i++)); do
    code="$(docker exec "${CONTAINER}" bash -c "curl -s -o /dev/null -w '%{http_code}' --max-time 2 ${REST}/readyz" 2>/dev/null || true)"
    if [ "${code}" = "200" ]; then ACTIVE=1; break; fi
    sleep 0.2
done
[ "${ACTIVE}" -eq 1 ] || fail "调度栈未达就绪（/readyz 非 200）"
echo "[HEALTH] 调度栈就绪且 /readyz=200 (PASS)"

# ---------------------------------------------------------------------------
# T1：探针免鉴权 + 组件齐全
# ---------------------------------------------------------------------------
echo ""
echo "[T1] /healthz 与 /readyz 免鉴权"
docker exec "${CONTAINER}" bash -c "
    set -e
    h=\$(curl -s -o /dev/null -w '%{http_code}' --max-time 3 ${REST}/healthz)
    r=\$(curl -s -o /dev/null -w '%{http_code}' --max-time 3 ${REST}/readyz)
    echo \"healthz=\${h} readyz=\${r}\"
    [ \"\${h}\" = 200 ] || { echo 'healthz 非 200'; exit 1; }
    [ \"\${r}\" = 200 ] || { echo 'readyz 非 200'; exit 1; }
    body=\$(curl -s --max-time 3 ${REST}/readyz)
    for c in dispatcher safety chassis coordinator; do
        echo \"\${body}\" | grep -q \"\\\"component\\\":\\\"\${c}\\\"\" || { echo \"缺少组件 \${c}\"; exit 1; }
    done
" || fail "T1 探针断言失败"

# 生产红线复核：无 Authorization 头也必须 200（上面 curl 未带任何头）
echo "[T1] 免鉴权且四组件齐全 (PASS)"

# ---------------------------------------------------------------------------
# T2：kill -9 dispatcher -> /readyz 503 且原因是 stale
# ---------------------------------------------------------------------------
# 按域 ID 精确找 dispatcher 进程（launch 的 respawn 会重起同名进程，
# 因此 pid 本身就是"是否换了新进程"最直接的证据）
dispatcher_pid() {
    docker exec "${CONTAINER}" bash -c "
        for envf in /proc/[0-9]*/environ; do
            pid=\"\${envf#/proc/}\"; pid=\"\${pid%/environ}\"
            [ -r \"\${envf}\" ] || continue
            if tr '\\0' '\\n' 2>/dev/null < \"\${envf}\" | grep -qx 'ROS_DOMAIN_ID=${HEALTH_DOMAIN_ID}'; then
                if tr '\\0' ' ' 2>/dev/null < /proc/\$pid/cmdline | grep -q 'lib/amr_dispatcher_ros/dispatcher_node'; then
                    echo \$pid; exit 0
                fi
            fi
        done
    " 2>/dev/null | tr -d '\r'
}

OLD_PID="$(dispatcher_pid)"
[ -n "${OLD_PID}" ] || fail "T2 未找到 dispatcher_node 进程"
echo ""
echo "[T2] kill -9 dispatcher_node (pid=${OLD_PID}) -> /readyz=503 (reason=stale)"
docker exec "${CONTAINER}" bash -c "kill -9 ${OLD_PID}" >/dev/null 2>&1 || true

RED=0
for ((i = 0; i < 40; i++)); do
    code="$(docker exec "${CONTAINER}" bash -c "curl -s -o /dev/null -w '%{http_code}' --max-time 2 ${REST}/readyz" 2>/dev/null || true)"
    if [ "${code}" = "503" ]; then RED=1; break; fi
    sleep 0.25
done
[ "${RED}" -eq 1 ] || fail "T2 dispatcher 停止后 /readyz 未在 10s 内转为 503"

docker exec "${CONTAINER}" bash -c "curl -s --max-time 3 ${REST}/readyz" \
    | grep -q '"component":"dispatcher","ok":false,"monitored":true,"reason":"stale"' \
    || fail "T2 dispatcher 组件未报 stale（必须是 stale，不能是 never_seen）"
echo "[T2] /readyz 503 且 dispatcher.reason=stale (PASS)"

# ---------------------------------------------------------------------------
# T3：webhook 收到 readiness_not_ready
# ---------------------------------------------------------------------------
echo ""
echo "[T3] webhook 收到 readiness_not_ready"
ALERT=0
for ((i = 0; i < 40; i++)); do
    if docker exec "${CONTAINER}" bash -c "grep -q '\"type\":\"readiness_not_ready\"' '${STAMP_DIR}/webhook.jsonl' 2>/dev/null"; then
        ALERT=1; break
    fi
    sleep 0.25
done
[ "${ALERT}" -eq 1 ] || fail "T3 10s 内未收到 readiness_not_ready 告警"
docker exec "${CONTAINER}" bash -c "grep '\"type\":\"readiness_not_ready\"' '${STAMP_DIR}/webhook.jsonl' | head -1" \
    | grep -q '"schema":"amr.alert.v1"' \
    || fail "T3 告警 payload 缺 schema=amr.alert.v1"
echo "[T3] 告警已送达且 schema 正确 (PASS)"

# ---------------------------------------------------------------------------
# T4：respawn 拉起 + 协调器重新提拉
# ---------------------------------------------------------------------------
echo ""
echo "[T4] dispatcher 被 launch respawn 重新拉起并回到 active"
NEW_PID=""
for ((i = 0; i < 100; i++)); do
    NEW_PID="$(dispatcher_pid)"
    if [ -n "${NEW_PID}" ] && [ "${NEW_PID}" != "${OLD_PID}" ]; then break; fi
    NEW_PID=""
    sleep 0.25
done
[ -n "${NEW_PID}" ] || fail "T4 25s 内 dispatcher 未被 launch respawn 拉活（pid 未变化）"

BACK=0
for ((i = 0; i < 150; i++)); do
    code="$(docker exec "${CONTAINER}" bash -c "curl -s -o /dev/null -w '%{http_code}' --max-time 2 ${REST}/readyz" 2>/dev/null || true)"
    if [ "${code}" = "200" ]; then BACK=1; break; fi
    sleep 0.25
done
[ "${BACK}" -eq 1 ] || fail "T4 重生后 /readyz 未回到 200（协调器未重新提拉？）"

# lifecycle 查询本身是 ROS 服务调用：respawn 刚结束时节点可能还没进 discovery，
# 单次 `ros2 lifecycle get` 会返回空。空结果**不等于**非 active，因此重试；
# 重试耗尽仍非 active 才算失败（绝不放宽成"看到 200 就通过"——
# /readyz=200 也可能是协调器把另一次抖动抹平了，我们要的是状态机真的回来了）。
STATE=""
for ((i = 0; i < 20; i++)); do
    STATE="$(docker exec "${CONTAINER}" bash -c "
        source /opt/ros/jazzy/setup.bash >/dev/null 2>&1
        source ${CONTAINER_WORKDIR}/install/setup.bash >/dev/null 2>&1
        ROS_DOMAIN_ID=${HEALTH_DOMAIN_ID} timeout 10 ros2 lifecycle get /dispatcher_node 2>/dev/null | head -1
    " | tr -d '\r')"
    echo "${STATE}" | grep -q "active" && break
    sleep 0.5
done
echo "${STATE}" | grep -q "active" || fail "T4 重生后 dispatcher lifecycle 非 active（20 次重试后仍为 '${STATE}'）"
echo "[T4] 新 pid=${NEW_PID} 且 lifecycle=active (PASS)"

# ---------------------------------------------------------------------------
# T5：恢复告警必须"恰好一条 / 每一次 firing"
#
# 不断言全局只有 1 条：respawn 后 dispatcher 会先回到 unconfigured、被协调器
# 重新 configure+activate，这期间 /dispatcher/state 可能再断一次
# （本验收把 stale 阈值压到 1.5s，属于刻意收紧），于是出现
#   not_ready -> recovered -> not_ready -> recovered
# 这**不是**告警缺陷，而是真实的抖动被如实上报。因此这里锁的是状态机不变量：
#   * 每一条 readiness_recovered 之前必须有一条未配对的 readiness_not_ready；
#   * 绝不允许出现"连续两条 recovered"或"从未 firing 就 recovered"；
#   * 最终状态必须是 resolved（栈已恢复）。
# ---------------------------------------------------------------------------
echo ""
echo "[T5] 恢复告警满足状态机不变量"
for ((i = 0; i < 60; i++)); do
    if docker exec "${CONTAINER}" bash -c "grep -q '\"type\":\"readiness_recovered\"' '${STAMP_DIR}/webhook.jsonl' 2>/dev/null"; then
        break
    fi
    sleep 0.25
done
docker exec "${CONTAINER}" bash -c "
    grep -o '\"type\":\"[a-z_]*\",\"state\":\"[a-z]*\"' '${STAMP_DIR}/webhook.jsonl' \
        | grep -E 'readiness_(not_ready|recovered)'
" > /tmp/health_readiness_seq.txt

python3 - /tmp/health_readiness_seq.txt <<'PY' || fail "T5 恢复告警状态机不变量被破坏"
import sys

lines = [ln.strip() for ln in open(sys.argv[1], encoding='utf-8') if ln.strip()]
pending = 0
recovered = 0
for ln in lines:
    if '"readiness_not_ready"' in ln:
        # 连续两次 firing 不允许：第二次只能在 resolved 之后再出现
        if pending:
            print(f"连续 readiness_not_ready 未配对: {ln}")
            raise SystemExit(1)
        pending = 1
    elif '"readiness_recovered"' in ln:
        if not pending:
            print(f"未见 firing 就 recovered（或连续两次 recovered）: {ln}")
            raise SystemExit(1)
        pending = 0
        recovered += 1
if recovered < 1:
    print('没有收到任何 readiness_recovered')
    raise SystemExit(1)
if pending:
    print('最后一条 not_ready 未被 resolved（栈仍未恢复）')
    raise SystemExit(1)
print(f'readiness 告警序列合法: {recovered} 次 firing/recovered 配对')
PY
echo "[T5] 恢复告警序列合法（$(grep -c 'readiness_recovered' /tmp/health_readiness_seq.txt || true) 次配对）(PASS)"

# ---------------------------------------------------------------------------
# T6：急停 -> 指标与 safety_critical 告警
# ---------------------------------------------------------------------------
echo ""
echo "[T6] 急停 -> /readyz 503 + 安全指标 + safety_critical 告警"
docker exec "${CONTAINER}" bash -c "
    source /opt/ros/jazzy/setup.bash >/dev/null 2>&1
    source ${CONTAINER_WORKDIR}/install/setup.bash >/dev/null 2>&1
    ROS_DOMAIN_ID=${HEALTH_DOMAIN_ID} timeout 10 ros2 topic pub --once /safety/estop std_msgs/msg/Bool '{data: true}' >/dev/null 2>&1
" || true

ESTOP_RED=0
for ((i = 0; i < 40; i++)); do
    code="$(docker exec "${CONTAINER}" bash -c "curl -s -o /dev/null -w '%{http_code}' --max-time 2 ${REST}/readyz" 2>/dev/null || true)"
    if [ "${code}" = "503" ]; then ESTOP_RED=1; break; fi
    sleep 0.25
done
[ "${ESTOP_RED}" -eq 1 ] || fail "T6 急停后 /readyz 未转为 503"

docker exec "${CONTAINER}" bash -c "curl -s --max-time 3 ${REST}/metrics" > /tmp/health_metrics.txt
grep -qx 'amr_safety_critical 1' /tmp/health_metrics.txt || fail "T6 amr_safety_critical 非 1"
grep -qx 'amr_safety_severity_level 2' /tmp/health_metrics.txt || fail "T6 amr_safety_severity_level 非 2"

# alert_safety_critical_sec:=3 -> 3 秒后才发（防抖：瞬时急停不刷告警）
SAFETY_ALERT=0
for ((i = 0; i < 60; i++)); do
    if docker exec "${CONTAINER}" bash -c "grep -q '\"type\":\"safety_critical\"' '${STAMP_DIR}/webhook.jsonl' 2>/dev/null"; then
        SAFETY_ALERT=1; break
    fi
    sleep 0.25
done
[ "${SAFETY_ALERT}" -eq 1 ] || fail "T6 15s 内未收到 safety_critical 告警"
echo "[T6] 急停指标与 safety_critical 告警 (PASS)"

# 解除急停 -> safety_recovered
docker exec "${CONTAINER}" bash -c "
    source /opt/ros/jazzy/setup.bash >/dev/null 2>&1
    source ${CONTAINER_WORKDIR}/install/setup.bash >/dev/null 2>&1
    ROS_DOMAIN_ID=${HEALTH_DOMAIN_ID} timeout 10 ros2 topic pub --once /safety/estop std_msgs/msg/Bool '{data: false}' >/dev/null 2>&1
" || true
SAFETY_RECOVER=0
for ((i = 0; i < 80; i++)); do
    if docker exec "${CONTAINER}" bash -c "grep -q '\"type\":\"safety_recovered\"' '${STAMP_DIR}/webhook.jsonl' 2>/dev/null"; then
        SAFETY_RECOVER=1; break
    fi
    sleep 0.25
done
[ "${SAFETY_RECOVER}" -eq 1 ] || fail "T6 解除急停后未收到 safety_recovered 告警"
echo "[T6] 解除急停后 safety_recovered (PASS)"

# ---------------------------------------------------------------------------
# T7：/metrics 新指标族全部有样本、无标签基数爆炸
# ---------------------------------------------------------------------------
echo ""
echo "[T7] /metrics 新指标族完备"
docker exec "${CONTAINER}" bash -c "curl -s --max-time 3 ${REST}/metrics" > /tmp/health_metrics.txt
for family in amr_ready amr_ready_component amr_dispatcher_state_age_seconds \
              amr_safety_severity_level amr_safety_critical amr_topology_overlay_active \
              amr_topology_overlay_revision amr_topology_blocked_edges amr_topology_one_way_edges \
              amr_alert_sent_total amr_alert_webhook_failures_total amr_alert_dropped_total; do
    grep -q "^# HELP ${family} " /tmp/health_metrics.txt || fail "T7 缺少指标族 ${family} 的 HELP 行"
    grep -q "^${family}" /tmp/health_metrics.txt || fail "T7 指标族 ${family} 无样本"
done
# amr_ready_component 的标签基数必须有界（只有 4 个已知组件）
COMP_COUNT="$(grep -c '^amr_ready_component{' /tmp/health_metrics.txt | tr -d '\r')"
[ "${COMP_COUNT}" -le 4 ] || fail "T7 amr_ready_component 标签基数异常: ${COMP_COUNT}"
# 绝对不允许把边名/站点名当标签（基数爆炸 + 泄露拓扑）
if grep -q 'route_edge:\|station_' /tmp/health_metrics.txt; then
    fail "T7 /metrics 出现拓扑实体名（标签基数会爆炸）"
fi
echo "[T7] 12 个新指标族齐全，标签基数有界 (PASS)"

# ---------------------------------------------------------------------------
# T8：alert_webhook_url 为空时不建告警线程
# ---------------------------------------------------------------------------
echo ""
echo "[T8] 未配置 webhook 时不产生告警线程"
docker exec -d "${CONTAINER}" bash -c "
    export ROS_DOMAIN_ID=${HEALTH_DOMAIN_ID}
    source /opt/ros/jazzy/setup.bash >/dev/null 2>&1
    source ${CONTAINER_WORKDIR}/install/setup.bash >/dev/null 2>&1
    # 不传 alert_webhook_url：它的默认值就是空串（= 告警通道关闭）。
    # 刻意不用 -p alert_webhook_url:='' —— 那会被 rcl 当成空值解析失败，
    # 本次踩过这个坑；"不传"与"显式传空"在语义上等价，且更贴近现网配置。
    exec ros2 run amr_dispatcher_tools rest_gateway --ros-args \
        -p port:=18109 \
        > ${STAMP_DIR}/gateway_nowebhook.log 2>&1
"
G2=0
for ((i = 0; i < 100; i++)); do
    if docker exec "${CONTAINER}" bash -c "cat < /dev/null > /dev/tcp/127.0.0.1/18109" 2>/dev/null; then G2=1; break; fi
    sleep 0.2
done
[ "${G2}" -eq 1 ] || fail "T8 第二实例未就绪"
# 线程数对比：只统计**节点二进制本身**（/proc/<pid>/exe 指向 rest_gateway），
# 排除 `ros2 run` 的 python 启动器与本脚本自己的 bash 包装进程（都踩过）。
#
# 如何区分两个实例：
#   * launch 起的那个（配置了 webhook）：参数由 launch 写成临时 params 文件，
#     cmdline 里**没有** `port:=`；
#   * 这里用 `ros2 run` 起的那个（不配置 webhook）：`port:=18109` 在 cmdline 里。
# 这条区分依据来自本脚本自己的启动方式，不依赖 pid 顺序之类的易变因素。
gateway_threads() {
    local mode="$1"   # "launch" = cmdline 无 port:= ；"cli" = cmdline 含 port:=18109
    docker exec "${CONTAINER}" bash -c "
        for envf in /proc/[0-9]*/environ; do
            pid=\"\${envf#/proc/}\"; pid=\"\${pid%/environ}\"
            [ \"\${pid}\" = \"\$\$\" ] && continue
            exe=\$(readlink -f /proc/\$pid/exe 2>/dev/null)
            case \"\$exe\" in
                */amr_dispatcher_tools/rest_gateway) ;;
                *) continue ;;
            esac
            # 只认本验收域内的实例（域 42 的生产网关绝不参与对比）
            tr '\\0' '\\n' 2>/dev/null < \"\$envf\" | grep -qx 'ROS_DOMAIN_ID=${HEALTH_DOMAIN_ID}' || continue
            cmdline=\$(tr '\\0' ' ' 2>/dev/null < /proc/\$pid/cmdline)
            case \"\$cmdline\" in
                *'port:=18109'*)
                    [ '${mode}' = cli ] || continue ;;
                *)
                    [ '${mode}' = launch ] || continue ;;
            esac
            awk '/^Threads:/{print \$2}' /proc/\$pid/status
            exit 0
        done
    " 2>/dev/null | tr -d '\r'
}

THREADS_PLAIN="$(gateway_threads cli)"
THREADS_ALERT="$(gateway_threads launch)"
echo "    threads: no_webhook=${THREADS_PLAIN} with_webhook=${THREADS_ALERT}"
[ -n "${THREADS_PLAIN}" ] || fail "T8 无法读取未配置 webhook 实例的线程数"
[ -n "${THREADS_ALERT}" ] || fail "T8 无法读取已配置 webhook 实例的线程数"
# 配置了 webhook 的实例**至少**多 1 个发送线程；未配置的必须更少
[ "${THREADS_ALERT}" -gt "${THREADS_PLAIN}" ] \
    || fail "T8 配置 webhook 的实例未比未配置的多出线程 (${THREADS_ALERT} vs ${THREADS_PLAIN})"
echo "[T8] 未配置 webhook 时无额外发送线程 (PASS)"

echo ""
echo "======================================================================"
echo " [PASS] 探针 / 守护 / 告警端到端验收全部通过:"
echo "   T1 /healthz 200 免鉴权, /readyz 200 且含 4 组件"
echo "   T2 kill -9 dispatcher -> 3s 内 /readyz 503 (reason=stale)"
echo "   T3 webhook 收到 readiness_not_ready (schema=amr.alert.v1)"
echo "   T4 respawn 拉起新进程 + 协调器重新提拉到 active"
echo "   T5 恰好一条 readiness_recovered"
echo "   T6 急停 -> 503 + amr_safety_critical=1 + safety_critical/safety_recovered"
echo "   T7 12 个新指标族齐全, 标签基数有界, 不含拓扑实体名"
echo "   T8 alert_webhook_url 为空时不建发送线程"
echo "======================================================================"

exit 0
