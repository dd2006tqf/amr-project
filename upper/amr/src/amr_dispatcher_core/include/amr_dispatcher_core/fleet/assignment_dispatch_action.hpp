// 选车结果 -> 调度动作的映射（纯函数）。
//
// 核心库只回答"能不能选到车"（kAssigned / kDeferred / kIndeterminate /
// kRejected），但"选车结果 -> 调度动作"的映射是**调度策略**，必须显式写死
// 并可测：
//
//   kAssigned      -> kDispatch              派发
//   kDeferred      -> kStayQueued            任务留队等待，下个 tick 重试（不判负）
//   kIndeterminate -> kPauseAutoAssignment   暂停自动分配，等状态恢复
//   kRejected      -> kReject                永久判负，附明确原因（不静默改派）
//
// 评审问题 4 的落点："暂时不可用"与"永久不满足"必须分开。
// 该判定被 dispatcher_node 的排队派发链与直接派发链共用。
//
// 历史：本逻辑最早以 header-only 形式放在 amr_dispatcher_ros 包里，
// 但它不依赖任何 ROS 类型 —— 下沉到 core 后既能被宿主 ctest 覆盖，
// 也能被以后的非 ROS 客户端复用。ROS 侧保留同名类型别名，调用点零改动。
#pragma once

#include <string>

#include "amr_dispatcher_core/fleet/vehicle_assignment.hpp"

namespace amr_dispatcher_core::fleet {

// 选车结果对应的调度动作。
enum class AssignmentDispatchAction {
  kDispatch,             // 继续派发
  kStayQueued,           // 暂时不可用 -> 留队等待
  kPauseAutoAssignment,  // 状态不确定 -> 暂停自动分配
  kReject,               // 永久不满足 -> 该单判负
};

struct AssignmentDispatchDecision {
  AssignmentDispatchAction action = AssignmentDispatchAction::kReject;
  // 判负/留队的原因（审计与日志用）
  std::string reason;
  // 决策依据（"explicit" / "nearest" / ...），仅 kDispatch 时有意义
  std::string strategy;
};

// 纯函数：三态 -> 调度动作。
inline AssignmentDispatchDecision DecideAssignmentDispatch(
    const VehicleAssignmentDecision& decision) {
  AssignmentDispatchDecision result;
  result.reason = decision.reject_reason;
  result.strategy = decision.strategy;
  switch (decision.outcome) {
    case AssignmentOutcome::kAssigned:
      result.action = AssignmentDispatchAction::kDispatch;
      result.reason.clear();
      break;
    case AssignmentOutcome::kDeferred:
      result.action = AssignmentDispatchAction::kStayQueued;
      break;
    case AssignmentOutcome::kIndeterminate:
      result.action = AssignmentDispatchAction::kPauseAutoAssignment;
      break;
    case AssignmentOutcome::kRejected:
      result.action = AssignmentDispatchAction::kReject;
      break;
  }
  return result;
}

}  // namespace amr_dispatcher_core::fleet
