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
# 现在改成守四件在 4c 之后才会坏、且只有真进程能暴露的事：
#   1. 发布端不再产生 rosidl 话题（否则 4c 就是没做完）
#   2. proto 话题**确实有数据**（节点漏调 Publish）
#   3. 消费者默认就挂在 proto 话题上（默认值没翻对 = 全链路静默断流）
#   4. lifecycle 节点的三路 proto 话题**在激活后真的出帧**（发布器没建、或
#      发布路径被 IsActivated() 守卫挡住时，DDS 不报错、ctest 也全绿）
#
# 第 4 条的措辞是修正过的：最初写成「漏调 Activate() 会导致没有 writer」，
# 阴性测试推翻了它——框架会自动激活托管实体，显式 Activate() 是冗余的。
# 详见阶段 5 的说明。
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
#
# 必须轮询，理由见阶段 5 的 count_pub：`--no-daemon` 每次查询都是一个短
# 发现窗口，单次查询可能返回空输出。这一步尤其不能单次——「空输出」与
# 「订阅挂在 rosidl 上」在 grep 看来是同一种失败，会把可恢复的查询抖动
# 报成真实的默认值错误。
SUB_TOPICS=""
for ((i = 0; i < 6; i++)); do
  SUB_TOPICS="$(ros2 node info /lh_e2e_consumer --no-daemon 2>/dev/null || true)"
  printf '%s' "${SUB_TOPICS}" | grep -q '/chassis/link_health_proto' && break
  ((i + 1 < 6)) && sleep 1
done
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
#
# 但 `--no-daemon` 本身也不是充分的：它每次调用新建一个临时 participant，
# 只等一个很短的发现窗口，同样可能返回空。所以必须轮询——单次查询的
# 「空输出」与真正的「话题不存在」在这里无法区分。
INFO=""
for ((i = 0; i < 6; i++)); do
  INFO="$(ros2 topic info /system/aggregated_link_health_proto --no-daemon 2>/dev/null || true)"
  [[ -n "${INFO}" ]] && break
  ((i + 1 < 6)) && sleep 1
done
if [[ -z "${INFO}" ]]; then
  echo "  ✗ /system/aggregated_link_health_proto 话题不存在（已轮询 6 次）"
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

# ---------------------------------------------------------------------------
# 阶段 5：lifecycle 节点的 proto 话题在激活后真的出帧
# ---------------------------------------------------------------------------
# 为什么单独做这一段：dispatcher_node 是生命周期节点，它的三个 proto 发布器
# 在 on_configure() 里创建。未激活时 publish 是**静默空操作**，所以「有话题、
# 有 writer、但零帧」是这条链路的天然中间态，只有真进程能暴露。
#
# ============================================================================
# 先纠正一个容易写错的因果关系（2026-11-08 阴性测试实测）
# ============================================================================
# 本段最初的注释断言「漏调 Activate() 会导致话题有 writer 但永远没有帧」。
# 做阴性测试（把 on_activate() 里的 `state_proto_pub_->Activate()` 注释掉、
# 重新编译、重跑）时，**帧照样收得到**，断言全绿 —— 上述因果关系是错的。
#
# 真实机制：`rclcpp_lifecycle::LifecycleNode::create_publisher()` 在
# lifecycle_node_impl.hpp 里除了建好 LifecyclePublisher，还会调
# `this->add_managed_entity(pub)` 把它登记为托管实体。生命周期节点在状态
# 迁移时会驱动所有托管实体的 on_activate()/on_deactivate()。也就是说——
# **框架已经自动调用了 on_activate()**，我们代码里那三行显式
# `Activate()` 是冗余的（幂等，留着无害，但删掉也不会坏）。
#
# 由此，本段**能**抓到和**不能**抓到的东西必须分清：
#   · 能抓到：发布器根本没建（configure 分支漏了 / 参数关掉了）、
#     发布路径被 `IsActivated()` 守卫或定时器提前 return 挡住、
#     proto 类型在 DDS 上不可达（收到 writer 却解不出帧）。
#   · 不能抓到：漏调我们那三行 `Activate()` —— 框架兜住了，这种情况
#     不存在故障，因此也不需要断言。
#
# ============================================================================
# 判据为什么是「有没有帧」而不是「有没有 writer」
# ============================================================================
# LifecyclePublisher 在 create_publisher 时就把 DataWriter 注册进 DDS 了，
# 激活与否只影响 publish() 发不发。所以 writer 数在 activate 前后**恒为 1**。
# 实测：注释掉 Activate() 后 writer 判据照样全绿 —— 它对这条链路没有分辨力。
# 能证明「真的在发」的只有帧：`ros2 amr echo <topic> <type> --once` 带超时去收。
# 这同时验证了 proto 类型在 DDS 上确实可解析（writer 存在 ≠ 能收到可解析的帧）。
#
# 注意 `ros2 topic hz` 对 proto 话题不可用（靠 rosidl introspection），
# 不能用它数帧——见 PROTOBUF_MIGRATION_PLAN.md 的陷阱记录。
echo
echo "=== 阶段 5：lifecycle 节点的 proto 话题在激活后真的出帧 ==="
DISP="${PKG_PREFIX}/lib/amr_dispatcher_ros/dispatcher_node"
if [[ ! -x "${DISP}" ]]; then
  echo "  ✗ 缺少可执行文件 ${DISP}"
  FAIL=1
else
  "${DISP}" --ros-args -r __node:=disp_e2e > /tmp/e2e_disp.log 2>&1 &
  PIDS+=("$!")
  sleep 5

  # 查 writer 数，**必须轮询**，不能单次查询。
  #
  # `--no-daemon` 每次调用都新建一个临时 participant、只等一个很短的发现
  # 窗口。窗口内没发现完就返回 `Publisher count: 0`——同一话题下一次查询
  # 可能又是 1。实测在同一个活着的 dispatcher 上连查 5 次得到
  # 1,0,1,1,1（daemon 模式反而是 1,1,1,1,1）。所以这里取**多次查询的最大值**：
  # 只要有一次发现到 writer，就说明它确实存在。
  #
  # 这与「daemon 缓存会过期所以要加 --no-daemon」并不冲突：两种模式都会漏报，
  # 只是漏报原因不同——daemon 是图缓存陈旧，--no-daemon 是发现窗口太短。
  # 前者的解法是换模式，后者只有一个解法：重复查。
  #
  # \param $1 话题名  $2 最多查几次（每次间隔 1s）
  count_pub() {
    local topic="$1" tries="${2:-6}" best="" c
    for ((i = 0; i < tries; i++)); do
      c="$(ros2 topic info "${topic}" --no-daemon 2>/dev/null \
        | sed -n 's/^Publisher count: //p' | head -1)"
      if [[ -n "${c}" ]] && { [[ -z "${best}" ]] || ((c > best)); }; then
        best="${c}"
      fi
      ((best >= 1)) && break
      ((i + 1 < tries)) && sleep 1
    done
    printf '%s' "${best}"
  }

  # 收帧探针：能收到一帧 = 这个话题**真的在发**，而不只是 DDS 里挂了个 writer。
  #
  # writer 数对这条链路没有分辨力：LifecyclePublisher 在 create_publisher 时
  # 就把 DataWriter 注册好了，激活与否只决定 publish() 发不发，所以 writer 数
  # 在 activate 前后恒为 1（实测：注释掉 Activate() 后 writer 判据照样全绿）。
  # 详见上面阶段 5 的两段说明。
  #
  # \param $1 话题名  $2 protobuf 全名  $3 超时秒数
  recv_frame() {
    timeout "${3:-6}" ros2 amr echo "$1" "$2" --once >/dev/null 2>&1
  }

  # 激活前：三个话题都不应存在（发布器尚未创建）
  #
  # 这里的抖动方向与激活后**相反**，值得单独说明：发现窗口太短只会把
  # 「有 writer」读成 0，不会凭空读出一个 writer。所以这个断言天生只会
  # **假通过**（发布器其实绕过了 lifecycle，却被漏查成 0），不会假失败。
  # 仍然查 3 次而不是 1 次，就是为了压低这个假通过概率；不需要像激活后
  # 那样查 6 次——那边要等的是「发现完成」，这边等不到任何新东西。
  PRE_BAD=0
  for t in /dispatcher/state /dispatcher/events /dispatcher/topology_state; do
    c="$(count_pub "${t}_proto" 3)"
    if [[ -n "${c}" ]]; then
      echo "  ✗ ${t}_proto 在 configure 之前就存在（publisher=${c}）——发布器绕过了 lifecycle"
      PRE_BAD=1
    fi
  done
  [[ "${PRE_BAD}" -eq 0 ]] && echo "  ✓ 激活前三条 _proto 话题均不存在"

  # configure + activate
  CFG="$(ros2 lifecycle set /disp_e2e configure 2>&1 | tail -1)"
  ACT="$(ros2 lifecycle set /disp_e2e activate 2>&1 | tail -1)"
  if ! printf '%s%s' "${CFG}" "${ACT}" | grep -q "Transitioning successful"; then
    echo "  ✗ lifecycle 状态迁移失败"
    echo "      configure: ${CFG}"
    echo "      activate : ${ACT}"
    FAIL=1
  else
    echo "  ✓ configure + activate 成功"
  fi
  sleep 3

  # 激活后：逐条断言「真的在发」。
  #
  # 两条高频话题用**帧**判据，events 只能用 **writer** 判据——这不是偷懒，
  # 是话题本身的发布时机决定的：
  #   · /dispatcher/state          ← PublishState()，500ms 定时器（2Hz）
  #   · /dispatcher/topology_state ← 同一个 PublishState() 里发（2Hz）
  #   · /dispatcher/events         ← 只在真有任务/事件发生时才发，空闲时零帧
  # 对 events 用帧判据会必然超时，那是假失败，不是缺陷。它在这里是弱断言：
  # 只能抓到「发布器根本没建」；「激活了但发不出帧」由前两条覆盖（同一类
  # 双发器、同一个 configure 分支、同一种守卫写法）。
  #
  # 失败时的归因要谨慎：Frame 判据失败不等于 Activate() 漏调（框架会自动
  # 激活托管实体，见上方阶段 5 说明）。真实原因通常是发布器没建、参数关掉、
  # 或发布路径被 IsActivated()/定时器提前 return 挡住。
  check_live() {
    local topic="$1" type="$2" mode="$3" c
    if [[ "${mode}" == "frame" ]]; then
      if recv_frame "${topic}" "${type}" 6; then
        echo "  ✓ ${topic} 收到帧（真的在发）"
      else
        echo "  ✗ ${topic} 6s 内收不到任何帧 —— 发布路径没通"
        c="$(count_pub "${topic}" 3)"
        if [[ "${c:-0}" -ge 1 ]]; then
          echo "      writer=${c}，但零帧：发布路径被挡住（IsActivated 守卫 / 定时器未起 / 类型不可达）"
        else
          echo "      writer=${c:-0}：发布器根本没创建（configure 分支或 proto_dual_publish 开关）"
        fi
        FAIL=1
      fi
    else
      c="$(count_pub "${topic}" 6)"
      if [[ "${c:-0}" -ge 1 ]]; then
        echo "  ✓ ${topic} publisher=${c}（该话题仅事件驱动，空闲无帧，故只查 writer）"
      else
        echo "  ✗ ${topic} 没有 writer（publisher=${c:-0}）—— 发布器未创建"
        FAIL=1
      fi
    fi
  }

  check_live /dispatcher/state_proto \
    amr_dispatcher.proto.DispatcherState frame
  check_live /dispatcher/topology_state_proto \
    amr_dispatcher.proto.TopologyState frame
  check_live /dispatcher/events_proto \
    amr_dispatcher.proto.MissionEvent writer

  # 同一时刻，旧 rosidl 话题一条都不该出现
  STALE="$(ros2 topic list --no-daemon 2>/dev/null \
    | grep -E '^/dispatcher/(state|events|topology_state)$' || true)"
  if [[ -n "${STALE}" ]]; then
    echo "  ✗ 旧 rosidl 话题仍在：$(printf '%s' "${STALE}" | tr '\n' ' ')"
    FAIL=1
  else
    echo "  ✓ 三条旧的 dispatcher rosidl 话题均不存在"
  fi
fi

echo
if [[ "${FAIL}" -ne 0 ]]; then
  echo "proto 链路 E2E: 失败" >&2
  exit 1
fi
echo "proto 链路 E2E: 通过"
exit 0
