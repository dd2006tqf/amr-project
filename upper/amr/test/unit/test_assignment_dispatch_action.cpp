#include "amr_dispatcher_core/fleet/assignment_dispatch_action.hpp"

#include <gtest/gtest.h>

using namespace amr_dispatcher_core::fleet;

namespace {

VehicleAssignmentDecision MakeDecision(AssignmentOutcome outcome,
                                       const std::string& reason = "",
                                       const std::string& strategy = "nearest") {
  VehicleAssignmentDecision d;
  d.outcome = outcome;
  d.reject_reason = reason;
  d.strategy = strategy;
  return d;
}

}  // namespace

TEST(AssignmentDispatchActionTest, AssignedDispatchesAndClearsReason) {
  const auto decision = DecideAssignmentDispatch(
      MakeDecision(AssignmentOutcome::kAssigned, "stale residual reason"));
  EXPECT_EQ(decision.action, AssignmentDispatchAction::kDispatch);
  EXPECT_TRUE(decision.reason.empty());  // 派发时不得残留拒绝原因
  EXPECT_EQ(decision.strategy, "nearest");  // 策略透传（审计/日志用）
}

// 核心语义：暂时不可用留队（不判负），与永久不满足必须分开。
TEST(AssignmentDispatchActionTest, DeferredStaysQueuedWithReason) {
  const auto decision = DecideAssignmentDispatch(
      MakeDecision(AssignmentOutcome::kDeferred, "all vehicles busy"));
  EXPECT_EQ(decision.action, AssignmentDispatchAction::kStayQueued);
  EXPECT_EQ(decision.reason, "all vehicles busy");
}

TEST(AssignmentDispatchActionTest, IndeterminatePausesAutoAssignment) {
  const auto decision = DecideAssignmentDispatch(
      MakeDecision(AssignmentOutcome::kIndeterminate, "vehicle state never received"));
  EXPECT_EQ(decision.action, AssignmentDispatchAction::kPauseAutoAssignment);
  EXPECT_EQ(decision.reason, "vehicle state never received");
}

TEST(AssignmentDispatchActionTest, RejectedFailsWithReason) {
  const auto decision = DecideAssignmentDispatch(
      MakeDecision(AssignmentOutcome::kRejected, "requested vehicle not found"));
  EXPECT_EQ(decision.action, AssignmentDispatchAction::kReject);
  EXPECT_EQ(decision.reason, "requested vehicle not found");
}

TEST(AssignmentDispatchActionTest, EveryOutcomeMapsToDistinctAction) {
  // 四种结果必须映射到四种动作：任何合并都会让"暂时/永久"的区分失效
  const auto assigned = DecideAssignmentDispatch(MakeDecision(AssignmentOutcome::kAssigned));
  const auto deferred = DecideAssignmentDispatch(MakeDecision(AssignmentOutcome::kDeferred));
  const auto indeterminate =
      DecideAssignmentDispatch(MakeDecision(AssignmentOutcome::kIndeterminate));
  const auto rejected = DecideAssignmentDispatch(MakeDecision(AssignmentOutcome::kRejected));
  EXPECT_NE(assigned.action, deferred.action);
  EXPECT_NE(assigned.action, indeterminate.action);
  EXPECT_NE(assigned.action, rejected.action);
  EXPECT_NE(deferred.action, indeterminate.action);
  EXPECT_NE(deferred.action, rejected.action);
  EXPECT_NE(indeterminate.action, rejected.action);
}
