#!/usr/bin/env bash
# ============================================================
# NavigateSequence 桥接节点容器内端到端冒烟（fallback 模式）
#
# 断言链：
#   1) bridge 起来且 /navigate_sequence action server 可见
#   2) 2 点 goal：伪造 arrived 两次（覆盖第 1、2 点）→ goal SUCCEEDED
#   3) loop goal：伪造 arrived 后序列回绕持续运行 → kill -INT 取消 → goal CANCELED
# 任何一步失败退出非 0。全部用超时 + grep 结果文本判定，不降低断言硬凑通过。
#
# 手工等价步骤（若镜像缺 ros2 CLI 无法自动执行，按此降级并在报告中记录）：
#   source /opt/ros/jazzy/setup.bash && source install/setup.bash
#   ros2 run amr_dispatcher_ros navigate_sequence_bridge_node &
#   ros2 action send_goal /navigate_sequence amr_dispatcher_interfaces/action/NavigateSequence \
#     "{waypoints: [{header: {frame_id: map}, pose: {position: {x: 1.0, y: 2.0}}},
#                   {header: {frame_id: map}, pose: {position: {x: 3.0, y: 4.0}}}],
#       station_names: [s1, s2], loop: false}" -t 25
#   # 每当桥日志出现 "Publishing waypoint" 后伪造一次到达：
#   ros2 topic pub -1 /tracking/error amr_dispatcher_interfaces/msg/TrackingError "{arrived: true}"
#   # 断言输出 "Goal finished with status: SUCCEEDED"
#   # loop goal（loop:=true）推进一次后取消（零 UUID=取消全部）：
#   ros2 service call /navigate_sequence/_action/cancel_goal action_msgs/srv/CancelGoal \
#     "{goal_info: {goal_id: {uuid: [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]}, stamp: {sec: 0, nanosec: 0}}}"
#   # 断言输出 "Goal finished with status: CANCELED"
# ============================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

# 环境：宿主无 ROS；若未 source 则补齐（Step 6 的 docker run 已 source，此处兜底）
if [[ -z "${ROS_DISTRO:-}" && -f /opt/ros/jazzy/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
fi
if [[ -z "${AMENT_PREFIX_PATH:-}" && -f "${WORKSPACE_DIR}/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${WORKSPACE_DIR}/install/setup.bash"
fi

command -v ros2 >/dev/null 2>&1 || {
  echo "[FAIL] ros2 CLI not found; cannot smoke-test (record degradation, do not lower assertions)"
  exit 1
}
if ! ros2 action list -h >/dev/null 2>&1; then
  echo "[FAIL] ros2 action CLI missing; cannot smoke-test (record degradation, do not lower assertions)"
  exit 1
fi

# python CLI 输出不缓冲、C++ 节点 stdout 行缓冲：保证日志门控 grep 即时可见
export PYTHONUNBUFFERED=1

TMP_DIR="$(mktemp -d)"
BRIDGE_LOG="${TMP_DIR}/bridge.log"
GOAL1_LOG="${TMP_DIR}/goal1.log"
GOAL2_LOG="${TMP_DIR}/goal2.log"
BRIDGE_PID=""
G1_PID=""
G2_PID=""

cleanup() {
  local pid
  for pid in "$G1_PID" "$G2_PID" "$BRIDGE_PID"; do
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
      kill -9 "$pid" 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT

fail() {
  echo "[FAIL] $*"
  echo "-------- bridge log (tail 60) --------"
  tail -60 "$BRIDGE_LOG" 2>/dev/null || true
  echo "-------- goal1 log (tail 30) --------"
  tail -30 "$GOAL1_LOG" 2>/dev/null || true
  echo "-------- goal2 log (tail 30) --------"
  tail -30 "$GOAL2_LOG" 2>/dev/null || true
  exit 1
}

# 轮询直到文件出现 pattern（100ms 步进，超时返回非 0）
wait_log() {
  local pattern="$1" file="$2" timeout_sec="${3:-20}"
  local i
  for ((i = 0; i < timeout_sec * 10; i++)); do
    if grep -q "$pattern" "$file" 2>/dev/null; then
      return 0
    fi
    sleep 0.1
  done
  return 1
}

# 轮询直到 pattern 在文件中至少出现 min_count 次
wait_log_count() {
  local pattern="$1" file="$2" min_count="$3" timeout_sec="${4:-20}"
  local i count
  for ((i = 0; i < timeout_sec * 10; i++)); do
    count="$(grep -c "$pattern" "$file" 2>/dev/null || true)"
    if [[ "${count:-0}" -ge "$min_count" ]]; then
      return 0
    fi
    sleep 0.1
  done
  return 1
}

# 有界等待进程退出
wait_pid_timeout() {
  local pid="$1" timeout_sec="${2:-30}"
  timeout "$timeout_sec" tail --pid="$pid" -f /dev/null >/dev/null 2>&1
}

echo "=== [Smoke] NavigateSequence 桥接节点端到端冒烟（fallback 模式） ==="

# ---------- 1. 启动 bridge 并等待 action server ----------
stdbuf -oL -eL ros2 run amr_dispatcher_ros navigate_sequence_bridge_node \
  --ros-args -p use_nav2:=false >"$BRIDGE_LOG" 2>&1 &
BRIDGE_PID=$!
server_up=0
for ((i = 0; i < 100; i++)); do
  if ros2 action list 2>/dev/null | grep -qx /navigate_sequence; then
    server_up=1
    break
  fi
  kill -0 "$BRIDGE_PID" 2>/dev/null || fail "bridge exited during startup"
  sleep 0.2
done
[[ "$server_up" -eq 1 ]] || fail "/navigate_sequence action server not visible within 20s"
echo "[OK] bridge up, /navigate_sequence server visible"

# ---------- 2. 2 点 goal：两次 arrived → SUCCEEDED ----------
GOAL1='{waypoints: [{header: {frame_id: map}, pose: {position: {x: 1.0, y: 2.0}}},
                    {header: {frame_id: map}, pose: {position: {x: 3.0, y: 4.0}}}],
        station_names: [s1, s2], loop: false}'
timeout 40 ros2 action send_goal /navigate_sequence amr_dispatcher_interfaces/action/NavigateSequence \
  "$GOAL1" -t 30 >"$GOAL1_LOG" 2>&1 &
G1_PID=$!

wait_log "Publishing waypoint 1/2" "$BRIDGE_LOG" 20 ||
  fail "bridge did not publish waypoint 1/2 after goal1"
sleep 0.5
ros2 topic pub -1 /tracking/error amr_dispatcher_interfaces/msg/TrackingError \
  "{arrived: true}" >/dev/null 2>&1 || true
wait_log "Publishing waypoint 2/2" "$BRIDGE_LOG" 20 ||
  fail "bridge did not advance to waypoint 2/2 after first arrived"
sleep 0.5
ros2 topic pub -1 /tracking/error amr_dispatcher_interfaces/msg/TrackingError \
  "{arrived: true}" >/dev/null 2>&1 || true
wait_pid_timeout "$G1_PID" 35 || fail "goal1 client did not finish within timeout"
wait "$G1_PID" 2>/dev/null || true
G1_PID=""
grep -q "Goal finished with status: SUCCEEDED" "$GOAL1_LOG" ||
  fail "goal1 expected SUCCEEDED"
grep -q "NavigateSequence goal succeeded" "$BRIDGE_LOG" ||
  fail "bridge did not log goal success"
echo "[OK] 2-waypoint goal finished with SUCCEEDED"

# ---------- 3. loop goal：回绕持续运行 → 取消 → CANCELED ----------
GOAL2='{waypoints: [{header: {frame_id: map}, pose: {position: {x: 5.0, y: 6.0}}}],
        station_names: [l1], loop: true}'
timeout 50 ros2 action send_goal /navigate_sequence amr_dispatcher_interfaces/action/NavigateSequence \
  "$GOAL2" -t 45 >"$GOAL2_LOG" 2>&1 &
G2_PID=$!

wait_log "Publishing waypoint 1/1" "$BRIDGE_LOG" 20 ||
  fail "bridge did not publish waypoint for loop goal"
sleep 0.5
ros2 topic pub -1 /tracking/error amr_dispatcher_interfaces/msg/TrackingError \
  "{arrived: true}" >/dev/null 2>&1 || true
# loop：到达后从头重发 → 同一 waypoint 至少第二次发布
wait_log_count "Publishing waypoint 1/1" "$BRIDGE_LOG" 2 20 ||
  fail "loop goal did not restart sequence after arrival"
sleep 2
kill -0 "$G2_PID" 2>/dev/null || fail "loop goal client exited early (goal not kept running)"
if grep -q "Goal finished with status" "$GOAL2_LOG"; then
  fail "loop goal finished on its own; expected to keep running until cancel"
fi
echo "[OK] loop goal still running after restart"

# 用 cancel_goal 服务取消（零 UUID=取消全部 goal）。
# 注意不用 kill -INT：Jazzy CLI 的 SIGINT 处理器在 spin 中重入 spin 会抛
# "Executor is already spinning"，取消结果不可靠。
# 服务发现偶有延迟：有界重试，最终以客户端结果 CANCELED 为端到端断言。
cancel_ok=0
cancel_resp=""
for ((try = 0; try < 3; try++)); do
  cancel_resp="$(timeout 30 ros2 service call /navigate_sequence/_action/cancel_goal action_msgs/srv/CancelGoal \
    "{goal_info: {goal_id: {uuid: [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]}, stamp: {sec: 0, nanosec: 0}}}" 2>&1 || true)"
  if echo "${cancel_resp}" | grep -qE "return_code[:=] *0"; then
    cancel_ok=1
    break
  fi
  sleep 1
done
[[ "$cancel_ok" -eq 1 ]] || fail "cancel_goal not accepted by server: ${cancel_resp}"
wait_pid_timeout "$G2_PID" 30 || fail "loop goal client did not exit after cancel"
wait "$G2_PID" 2>/dev/null || true
G2_PID=""
grep -q "Goal finished with status: CANCELED" "$GOAL2_LOG" ||
  fail "loop goal expected CANCELED after cancel"
grep -q "NavigateSequence goal canceled" "$BRIDGE_LOG" ||
  fail "bridge did not log goal canceled"
echo "[OK] loop goal canceled with CANCELED"

echo "=== check_navigate_sequence_bridge.sh PASSED ==="
