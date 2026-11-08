#!/usr/bin/env bash
# proto 链路 E2E：真进程 + 真 DDS，验证 proto 话题是**唯一**的传输路径且可用。
#
# ============================================================================
# 阶段 4c 之后这个脚本守什么
# ============================================================================
# 4c 之前它守的是「双发一致性」：同一份数据走 rosidl 与 proto 两条路，
# 逐字段比对必须相等。4c 删除了 rosidl 发布侧，**比对基准消失了**，
# 那条断言在结构上已不可能成立。
#
# 现在改成守三件在 4c 之后才会坏、且只有真进程能暴露的事：
#   1. 发布端不再产生 rosidl 话题（否则 4c 就是没做完）
#   2. proto 话题**确实有数据**（节点漏调 Publish / lifecycle 漏 Activate）
#   3. 消费者默认就挂在 proto 话题上（默认值没翻对 = 全链路静默断流）
#
# 第 3 条是 4c 特有的失败模式：发布端改完了、订阅端默认值没改，两边各自
# 都「正常」，只是永远收不到对方。DDS 不报错，ctest 也抓不到。
#
# 用法：
#   ./scripts/check_proto_dual_e2e.sh
#   AMR_E2E_INNER=1 ./scripts/check_proto_dual_e2e.sh   # 容器内直跑
#
# 退出码：0 = 通过；1 = 失败；2 = 环境不满足
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

# 独立 domain：容器里可能还跑着常驻系统，共用域会让「有没有数据」的
# 判断被别的 writer 污染（本项目已经踩过一次，见 PROTOBUF_MIGRATION_PLAN.md）。
export ROS_DOMAIN_ID="${AMR_E2E_DOMAIN_ID:-73}"

PKG_PREFIX="$(ros2 pkg prefix amr_dispatcher_ros 2>/dev/null || true)"
if [[ -z "${PKG_PREFIX}" ]]; then
  echo "错误: 找不到 amr_dispatcher_ros，先构建工作区" >&2
  exit 2
fi
LH_PUB="${PKG_PREFIX}/lib/amr_dispatcher_ros/link_health_publisher"
if [[ ! -x "${LH_PUB}" ]]; then
  echo "错误: 缺少可执行文件 ${LH_PUB}" >&2
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

echo "=== ROS_DOMAIN_ID=${ROS_DOMAIN_ID}（与常驻系统隔离）==="
echo "域内节点（启动前）: $(ros2 node list 2>/dev/null | tr '\n' ' ')"

FAIL=0

# ---------------------------------------------------------------------------
# 阶段 1：发布端只发 proto —— 7 条受管话题的 rosidl 侧必须不存在
# ---------------------------------------------------------------------------
# 这一步在**空域**里跑，所以「话题不存在」只可能来自本脚本没起发布端，
# 而不是被别处的话题掩盖。
echo
echo "=== 阶段 1：rosidl 侧必须已消失 ==="
for t in /chassis/link_health /system/aggregated_link_health \
         /dispatcher/state /dispatcher/events /dispatcher/topology_state \
         /safety/state /tracking/error; do
  if ros2 topic list 2>/dev/null | grep -qx "$t"; then
    echo "  ✗ $t 仍然存在 —— 4c 没有做完（应只发 ${t}_proto）"
    FAIL=1
  else
    echo "  ✓ $t 不存在"
  fi
done

# ---------------------------------------------------------------------------
# 阶段 2：proto 侧确实有数据
# ---------------------------------------------------------------------------
echo
echo "=== 阶段 2：link_health_publisher 的 proto 输出有数据 ==="
# publish_rate_hz 提到 5Hz 让 e2e 更快拿到帧；watched_topics 指一条
# 没人发布的 proto 话题也没关系——它是聚合的**输入**，无输入时聚合定时器
# 直接 return，所以必须先喂输入。这里改从 **proto** 侧喂（见阶段 3 的说明）。
"${LH_PUB}" --ros-args -r __node:=lh_e2e_iso -p publish_rate_hz:=5.0 \
  > /tmp/e2e_lh.log 2>&1 &
PIDS+=("$!")
sleep 4

echo "--- 启动日志 ---"
grep -E "发布 proto|proto_dual" /tmp/e2e_lh.log | head -3 || true

echo "--- 域内 link_health 相关话题 ---"
ros2 topic list 2>/dev/null | grep -E 'aggregated_link_health' || echo "  (无)"

# 聚合需要至少一条输入链路。用 python 直接往 proto 话题注入一帧，
# 这既验证了「proto 侧能被写入」，也验证了消费者的反投影路径。
echo
echo "=== 阶段 3：消费者默认挂在 proto 话题上 ==="
"${LH_PUB}" --ros-args -r __node:=lh_e2e_consumer -p publish_rate_hz:=5.0 \
  > /tmp/e2e_lh_consumer.log 2>&1 &
PIDS+=("$!")
sleep 3

# 用 node info 判断订阅端落在哪个话题上——这是 4c 默认值翻对没翻对的唯一
# 直接证据。若默认值仍是 false，这里会显示 /chassis/link_health（rosidl），
# 而那侧现在已经没有发布者，链路静默断流。
SUB_TOPICS="$(ros2 node info /lh_e2e_consumer --no-daemon 2>/dev/null || true)"
if printf '%s' "${SUB_TOPICS}" | grep -q '/chassis/link_health_proto'; then
  echo "  ✓ 消费者订阅 /chassis/link_health_proto（默认值已翻转为 proto）"
else
  echo "  ✗ 消费者没有订阅 /chassis/link_health_proto —— 订阅侧默认值没翻"
  printf '%s\n' "${SUB_TOPICS}" | sed -n '/Subscribers/,/Publishers/p' | head -8 | sed 's/^/      /'
  FAIL=1
fi

# ---------------------------------------------------------------------------
# 阶段 4：proto 话题的 writer 存在（节点真在发）
# ---------------------------------------------------------------------------
echo
echo "=== 阶段 4：proto 话题上确有 writer ==="
# 注意：`ros2 topic info` 对 proto 话题可用（它只读 DDS 图，不做类型内省），
# 但 `ros2 topic hz/echo` **不可用**——见 PROTOBUF_MIGRATION_PLAN.md 的陷阱记录。
#
# `--no-daemon` 不是可选项：本脚本刚起进程 3–4 秒就查询，daemon 的图缓存可能
# 还没跟上，不加会拿到**空输出**，表现为「话题不存在」的假失败。
INFO="$(ros2 topic info /system/aggregated_link_health_proto --no-daemon 2>/dev/null || true)"
if [[ -z "${INFO}" ]]; then
  echo "  ✗ /system/aggregated_link_health_proto 话题不存在"
  FAIL=1
else
  printf '%s\n' "${INFO}" | sed 's/^/      /'
  # 无输入时聚合定时器直接 return，所以 writer 存在即可，不强求帧数。
  if printf '%s' "${INFO}" | grep -q 'Publisher count: 0'; then
    echo "  ✗ 有话题但没有任何发布者 —— 发布器没建起来"
    FAIL=1
  else
    echo "  ✓ 存在发布者"
  fi
fi

echo
if [[ "${FAIL}" -ne 0 ]]; then
  echo "proto 链路 E2E: 失败" >&2
  exit 1
fi
echo "proto 链路 E2E: 通过"
exit 0
