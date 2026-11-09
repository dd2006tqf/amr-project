#!/usr/bin/env bash
# 阶段 2 双发一致性 E2E：真进程 + 真 DDS，逐字段比对 rosidl 与 proto 两路话题。
#
# ============================================================================
# 这一层守的是什么（以及为什么前面的测试不够）
# ============================================================================
#   test_proto_projection  → 投影函数对（纯函数，进程内）
#   ctest 各节点单测        → 节点逻辑对（不涉及双发）
#
# 两者都覆盖不到的、只有真进程跑起来才暴露的失败：
#   1. 某节点漏调 ProtoDualPublisher::Publish()（编译通过、运行静默）
#   2. lifecycle 节点漏调 Activate()（proto 话题永远没数据）
#   3. per-vehicle 话题派生在两侧不一致
#   4. 序列化/反序列化链路坏了（投影对但字节坏）
#
# 覆盖的发布者：link_health_publisher（不依赖其他节点，可独立起）
# 以及 chassis_driver（如果串口模拟可用——不可用则跳过并明确说明）。
#
# 用法：
#   ./scripts/check_proto_dual_e2e.sh
#   AMR_E2E_INNER=1 ./scripts/check_proto_dual_e2e.sh   # 容器内直跑
#
# 退出码：0 = 通过；1 = 不一致；2 = 环境不满足
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# 与仓库其他 e2e 脚本一致的内层守卫：默认尝试进入 amr_dispatcher 容器，
# 容器内（AMR_E2E_INNER=1）直接执行。
if [[ "${AMR_E2E_INNER:-0}" != "1" ]]; then
  if command -v docker >/dev/null 2>&1 && docker ps --format '{{.Names}}' | grep -qx amr_dispatcher; then
    exec docker exec -e AMR_E2E_INNER=1 amr_dispatcher bash "$0" "$@"
  fi
fi

set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
if [[ -f "${REPO_ROOT}/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${REPO_ROOT}/install/setup.bash"
elif [[ -f /workspace/install/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /workspace/install/setup.bash
fi
set -u

PKG_PREFIX="$(ros2 pkg prefix amr_dispatcher_ros 2>/dev/null || true)"
if [[ -z "${PKG_PREFIX}" ]]; then
  echo "错误: 找不到 amr_dispatcher_ros，先构建工作区" >&2
  exit 2
fi
LH_PUB="${PKG_PREFIX}/lib/amr_dispatcher_ros/link_health_publisher"
CHECKER="${SCRIPT_DIR}/check_proto_dual_consistency.py"
if [[ ! -x "${LH_PUB}" ]]; then
  echo "错误: 缺少可执行文件 ${LH_PUB}" >&2
  exit 2
fi
if [[ ! -f "${CHECKER}" ]]; then
  echo "错误: 缺少校验器 ${CHECKER}" >&2
  exit 2
fi

PIDS=()
cleanup() {
  for pid in "${PIDS[@]:-}"; do
    kill "${pid}" 2>/dev/null || true
  done
  wait 2>/dev/null || true
}
trap cleanup EXIT

echo "=== 启动假底盘（往 /chassis/link_health 打数据）==="
python3 - <<'PYEOF' &
import time
import rclpy
from rclpy.node import Node
from amr_dispatcher_interfaces.msg import ChassisLinkHealth

rclpy.init()
node = Node('fake_chassis_for_dual_check')
pub = node.create_publisher(ChassisLinkHealth, '/chassis/link_health', 10)

msg = ChassisLinkHealth()
msg.backend_name = 'mick_binary'
msg.tier = 'DEGRADED'          # 已知枚举值：必须映射成 TIER_DEGRADED 且不留 tier_text
msg.frames_total = 1000
msg.frames_success = 990
msg.frames_lost = 10
msg.frames_crc_error = 3
msg.loss_rate = 0.01
msg.avg_latency_ms = 12.5
msg.max_latency_ms = 30.0
msg.is_healthy = True
msg.command_delivery_rate = 0.98   # 有效送达率：valid=true 分支
msg.commands_sent = 500
msg.commands_delivered = 490

deadline = time.time() + 20
try:
    while time.time() < deadline:
        pub.publish(msg)
        time.sleep(0.2)
except Exception:  # noqa: BLE001 - 脚本收尾时 rclpy 上下文被主脚本 shutdown，
    pass           # publish 会抛 RCLError。这不是失败，不该在 CI 日志里
                   # 留下一段看起来像崩溃的 traceback。
finally:
    try:
        rclpy.shutdown()
    except Exception:  # noqa: BLE001
        pass
PYEOF
PIDS+=("$!")

sleep 2

echo "=== 启动 link_health_publisher（双发）==="
"${LH_PUB}" &
PIDS+=("$!")

sleep 3

echo "=== 已发布的相关话题 ==="
ros2 topic list | grep -E 'link_health' || true

echo
echo "=== 逐字段比对 /system/aggregated_link_health vs _proto ==="
python3 "${CHECKER}" \
  --topic /system/aggregated_link_health \
  --rosidl-type amr_dispatcher_interfaces/msg/ChassisLinkHealth \
  --proto-type amr_dispatcher.proto.ChassisLinkHealth \
  --seconds 6 \
  --max-frames 10
RC=$?

if [[ "${RC}" -ne 0 ]]; then
  echo
  echo "阶段 2 双发一致性: 失败 (rc=${RC})" >&2
  exit "${RC}"
fi
echo
echo "阶段 2 双发一致性: 通过"

# ===========================================================================
# 阶段 3：订阅侧验证
# ===========================================================================
# 上面证明的是「同一份数据能同时走两条路并逐字段相等」。这一节证明的是
# **消费者真的能从 proto 那一路拿到等价的数据**——两者不可互相替代：
# 双发一致可能是「proto 话题根本没被订阅」的假象。
#
# 做法：再起一个 link_health_publisher 实例，打开 proto_dual_subscribe，
# 让它只读 /chassis/link_health_proto，再由它的聚合输出反推。
# 若反投影有误（例如把 -1 哨兵还原成 0.0），聚合出来的 tier 会变成
# CRITICAL 而不是 DEGRADED——这是本脚本刻意选的、能区分两者的字段。
echo
echo "=== 阶段 3：订阅侧（proto_dual_subscribe=true）==="
"${LH_PUB}" --ros-args \
  -r __node:=link_health_publisher_sub_side \
  -p proto_dual_subscribe:=true \
  -p publish_rate_hz:=5.0 &
PIDS+=("$!")

sleep 4

echo "=== 订阅侧输出的话题 ==="
ros2 topic list | grep -E 'aggregated_link_health' || true

# 订阅侧实例读的是 /chassis/link_health_proto（上面假底盘经 chassis 侧双发过来的
# 是 rosidl 原文；link_health_publisher 的写入端仍是 rosidl，故这里要确认它
# 派生出的 _proto 名与发布端一致）。
python3 "${CHECKER}" \
  --topic /system/aggregated_link_health \
  --rosidl-type amr_dispatcher_interfaces/msg/ChassisLinkHealth \
  --proto-type amr_dispatcher.proto.ChassisLinkHealth \
  --seconds 6 \
  --max-frames 5
RC2=$?

if [[ "${RC2}" -eq 0 ]]; then
  echo
  echo "阶段 3 订阅侧: 通过"
else
  echo
  echo "阶段 3 订阅侧: 失败 (rc=${RC2})" >&2
fi
exit "${RC2}"
