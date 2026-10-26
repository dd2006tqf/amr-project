#include "amr_dispatcher_core/workflow/traffic_coordination_model.hpp"

namespace amr_dispatcher_core::workflow {

const char* TrafficCoordinationStateLabel(TrafficCoordinationState state) {
  switch (state) {
    case TrafficCoordinationState::kRunning:
      return "RUNNING";
    case TrafficCoordinationState::kBlocked:
      return "BLOCKED";
    case TrafficCoordinationState::kReplanning:
      return "REPLANNING";
    case TrafficCoordinationState::kWaitingForResource:
      return "WAITING_FOR_RESOURCE";
    case TrafficCoordinationState::kRecoveryFailed:
      return "RECOVERY_FAILED";
  }
  return "UNKNOWN";
}

bool IsValidTrafficStateTransition(TrafficCoordinationState from, TrafficCoordinationState to) {
  if (from == to) {
    return true;  // 幂等维持
  }
  switch (from) {
    case TrafficCoordinationState::kRunning:
      // 运行中可遇到阻断进入 BLOCKED，或直接由调度触发重规划 REPLANNING
      return to == TrafficCoordinationState::kBlocked ||
             to == TrafficCoordinationState::kReplanning;

    case TrafficCoordinationState::kBlocked:
      // 阻断后可尝试重规划，或直接进入安全等待，或重规划失败直接判定 RECOVERY_FAILED
      return to == TrafficCoordinationState::kReplanning ||
             to == TrafficCoordinationState::kWaitingForResource ||
             to == TrafficCoordinationState::kRecoveryFailed;

    case TrafficCoordinationState::kReplanning:
      // 重规划成功后恢复 RUNNING；或规划出的新路线也需等待 WAITING_FOR_RESOURCE；或无路可走 RECOVERY_FAILED
      return to == TrafficCoordinationState::kRunning ||
             to == TrafficCoordinationState::kWaitingForResource ||
             to == TrafficCoordinationState::kRecoveryFailed;

    case TrafficCoordinationState::kWaitingForResource:
      // 等待唤醒后重新推进 RUNNING；或等待超时/再遇冲突触发 REPLANNING；或无解 RECOVERY_FAILED
      return to == TrafficCoordinationState::kRunning ||
             to == TrafficCoordinationState::kReplanning ||
             to == TrafficCoordinationState::kRecoveryFailed;

    case TrafficCoordinationState::kRecoveryFailed:
      // 失败态只能通过人工恢复或整体重置返回 REPLANNING / RUNNING
      return to == TrafficCoordinationState::kReplanning ||
             to == TrafficCoordinationState::kRunning;
  }
  return false;
}

}  // namespace amr_dispatcher_core::workflow
