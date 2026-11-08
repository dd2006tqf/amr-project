// A7：派发侧三态决策单测（ROS 侧，纯函数可脱离 rclcpp）。
//
// 评审问题 4 的落点："选车失败"与"任务排队"不能矛盾：
//   暂时不可用 -> 任务留队等待（kStayQueued）
//   状态不确定 -> 暂停自动分配（kPauseAutoAssignment）
//   永久不满足 -> 该单判负（kReject）
// 该映射写死在 DecideAssignmentDispatch 里，排队派发链与直接派发链共用。

#include "amr_dispatcher_ros/assignment_dispatch_decision.hpp"

#include <gtest/gtest.h>

#include <string>

using namespace amr_dispatcher_ros;
using amr_dispatcher_core::fleet::AssignmentOutcome;
using amr_dispatcher_core::fleet::VehicleAssignmentDecision;

TEST(AssignmentDispatchDecisionTest, AssignedDispatches) {
  VehicleAssignmentDecision decision;
  decision.outcome = AssignmentOutcome::kAssigned;
  decision.vehicle_id = "amr_01";
  decision.strategy = "nearest";

  const auto result = DecideAssignmentDispatch(decision);

  EXPECT_EQ(result.action, AssignmentDispatchAction::kDispatch);
  EXPECT_EQ(result.strategy, "nearest");
  EXPECT_TRUE(result.reason.empty());
}

TEST(AssignmentDispatchDecisionTest, DeferredStaysQueuedAndKeepsReason) {
  VehicleAssignmentDecision decision;
  decision.outcome = AssignmentOutcome::kDeferred;
  decision.reject_reason = "all vehicles busy";

  const auto result = DecideAssignmentDispatch(decision);

  EXPECT_EQ(result.action, AssignmentDispatchAction::kStayQueued);
  EXPECT_EQ(result.reason, "all vehicles busy");
}

TEST(AssignmentDispatchDecisionTest, IndeterminatePausesAutoAssignment) {
  VehicleAssignmentDecision decision;
  decision.outcome = AssignmentOutcome::kIndeterminate;
  decision.reject_reason = "no state report ever received";

  const auto result = DecideAssignmentDispatch(decision);

  EXPECT_EQ(result.action, AssignmentDispatchAction::kPauseAutoAssignment);
  EXPECT_EQ(result.reason, "no state report ever received");
}

TEST(AssignmentDispatchDecisionTest, RejectedFailsPermanentlyWithReason) {
  VehicleAssignmentDecision decision;
  decision.outcome = AssignmentOutcome::kRejected;
  decision.reject_reason = "requested vehicle not in managed list";

  const auto result = DecideAssignmentDispatch(decision);

  EXPECT_EQ(result.action, AssignmentDispatchAction::kReject);
  EXPECT_EQ(result.reason, "requested vehicle not in managed list");
}

TEST(AssignmentDispatchDecisionTest, DeferredAndRejectedAreDistinctActions) {
  // 这条断言锁住"失败 vs 排队"的语义分界：两者绝不能塌缩成同一动作
  VehicleAssignmentDecision deferred;
  deferred.outcome = AssignmentOutcome::kDeferred;
  VehicleAssignmentDecision rejected;
  rejected.outcome = AssignmentOutcome::kRejected;
  VehicleAssignmentDecision indeterminate;
  indeterminate.outcome = AssignmentOutcome::kIndeterminate;

  EXPECT_NE(DecideAssignmentDispatch(deferred).action,
            DecideAssignmentDispatch(rejected).action);
  EXPECT_NE(DecideAssignmentDispatch(deferred).action,
            DecideAssignmentDispatch(indeterminate).action);
  EXPECT_NE(DecideAssignmentDispatch(rejected).action,
            DecideAssignmentDispatch(indeterminate).action);
}
