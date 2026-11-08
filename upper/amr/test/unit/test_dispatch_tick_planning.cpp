#include "amr_dispatcher_core/workflow/dispatch_tick_planning.hpp"

#include <gtest/gtest.h>

using amr_dispatcher_core::dispatcher::Mission;
using amr_dispatcher_core::fleet::AssignmentOutcome;
using amr_dispatcher_core::fleet::VehicleAssignmentDecision;
using amr_dispatcher_core::workflow::DispatchCandidate;
using amr_dispatcher_core::workflow::DispatchStep;
using amr_dispatcher_core::workflow::DispatchTickDecision;
using amr_dispatcher_core::workflow::DispatchTickRequest;
using amr_dispatcher_core::workflow::PlanDispatchTick;
using amr_dispatcher_core::workflow::ReservationAttempt;

namespace {

DispatchCandidate MakeCandidate(const std::string& id, AssignmentOutcome outcome,
                                const std::string& vehicle_id = "amr_01",
                                std::vector<std::string> locks = {}) {
  DispatchCandidate candidate;
  candidate.mission.id = id;
  candidate.mission.order_id = "order_" + id;
  candidate.assignment.outcome = outcome;
  candidate.assignment.vehicle_id = vehicle_id;
  if (outcome == AssignmentOutcome::kDeferred) {
    candidate.assignment.reject_reason = "all vehicles busy";
  } else if (outcome == AssignmentOutcome::kIndeterminate) {
    candidate.assignment.reject_reason = "vehicle state never received";
  } else if (outcome == AssignmentOutcome::kRejected) {
    candidate.assignment.reject_reason = "requested vehicle not found";
  }
  candidate.route_locks = std::move(locks);
  return candidate;
}

std::vector<DispatchStep::Kind> Kinds(const DispatchTickDecision& decision) {
  std::vector<DispatchStep::Kind> kinds;
  for (const auto& step : decision.steps) {
    kinds.push_back(step.kind);
  }
  return kinds;
}

}  // namespace

TEST(DispatchTickPlanningTest, AssignedCandidateIsDispatched) {
  DispatchTickRequest request;
  request.candidates = {MakeCandidate("m1", AssignmentOutcome::kAssigned, "amr_02",
                                      {"route_edge:a__b"})};
  const auto decision = PlanDispatchTick(request);
  ASSERT_EQ(decision.steps.size(), 1u);
  EXPECT_EQ(decision.steps[0].kind, DispatchStep::Kind::kDispatch);
  EXPECT_EQ(decision.steps[0].mission_id, "m1");
  EXPECT_EQ(decision.steps[0].vehicle_id, "amr_02");
  EXPECT_EQ(decision.steps[0].route_locks, (std::vector<std::string>{"route_edge:a__b"}));
  EXPECT_TRUE(decision.dispatched);
  EXPECT_EQ(decision.dispatched_mission_id, "m1");
}

TEST(DispatchTickPlanningTest, DeferredStaysQueuedWithoutStep) {
  DispatchTickRequest request;
  request.candidates = {MakeCandidate("m1", AssignmentOutcome::kDeferred)};
  const auto decision = PlanDispatchTick(request);
  EXPECT_TRUE(decision.steps.empty());  // 留队不产生任何落地动作
  EXPECT_TRUE(decision.saw_deferred);
  EXPECT_EQ(decision.deferred_reason, "all vehicles busy");
  EXPECT_FALSE(decision.dispatched);
}

TEST(DispatchTickPlanningTest, IndeterminatePausesAutoAssignment) {
  DispatchTickRequest request;
  request.candidates = {MakeCandidate("m1", AssignmentOutcome::kIndeterminate)};
  const auto decision = PlanDispatchTick(request);
  EXPECT_TRUE(decision.steps.empty());
  EXPECT_TRUE(decision.saw_indeterminate);
  EXPECT_EQ(decision.indeterminate_reason, "vehicle state never received");
  EXPECT_FALSE(decision.saw_deferred);
}

TEST(DispatchTickPlanningTest, RejectedCandidateProducesRejectStepThenContinues) {
  DispatchTickRequest request;
  request.candidates = {MakeCandidate("m1", AssignmentOutcome::kRejected),
                        MakeCandidate("m2", AssignmentOutcome::kAssigned)};
  request.try_reserve = [](const std::string&, const std::vector<std::string>&) {
    ReservationAttempt a;
    a.success = true;
    return a;
  };
  const auto decision = PlanDispatchTick(request);
  EXPECT_EQ(Kinds(decision), (std::vector<DispatchStep::Kind>{DispatchStep::Kind::kRejectCandidate,
                                                             DispatchStep::Kind::kDispatch}));
  EXPECT_EQ(decision.steps[0].reason, "requested vehicle not found");
  EXPECT_EQ(decision.dispatched_mission_id, "m2");
}

// 兼容红线：一 tick 只派一个 —— 命中后不再扫描后续候选。
TEST(DispatchTickPlanningTest, OnlyOneDispatchPerTick) {
  DispatchTickRequest request;
  request.candidates = {MakeCandidate("m1", AssignmentOutcome::kAssigned),
                        MakeCandidate("m2", AssignmentOutcome::kAssigned)};
  int reserve_calls = 0;
  request.try_reserve = [&reserve_calls](const std::string&,
                                         const std::vector<std::string>&) {
    ++reserve_calls;
    ReservationAttempt a;
    a.success = true;
    return a;
  };
  const auto decision = PlanDispatchTick(request);
  EXPECT_TRUE(decision.dispatched);
  EXPECT_EQ(decision.dispatched_mission_id, "m1");
  EXPECT_EQ(reserve_calls, 1);  // 第二个候选根本没有被尝试
  EXPECT_EQ(Kinds(decision), (std::vector<DispatchStep::Kind>{DispatchStep::Kind::kDispatch}));
}

// 队首阻塞规避：队首约不上锁 -> 记账 wait，继续扫下一个。
TEST(DispatchTickPlanningTest, StaticDenialRecordsWaitAndTriesNext) {
  DispatchTickRequest request;
  request.candidates = {MakeCandidate("m1", AssignmentOutcome::kAssigned, "amr_01", {"lockX"}),
                        MakeCandidate("m2", AssignmentOutcome::kAssigned, "amr_02", {"lockY"})};
  request.try_reserve = [](const std::string& mission_id, const std::vector<std::string>&) {
    ReservationAttempt a;
    if (mission_id == "m1") {
      a.success = false;
      a.denial_kind = ReservationAttempt::DenialKind::kStatic;
      a.message = "resource locked: lockX by other";
    } else {
      a.success = true;
    }
    return a;
  };
  const auto decision = PlanDispatchTick(request);
  EXPECT_EQ(Kinds(decision),
            (std::vector<DispatchStep::Kind>{DispatchStep::Kind::kRecordWait,
                                             DispatchStep::Kind::kEmitReservationDenied,
                                             DispatchStep::Kind::kDispatch}));
  EXPECT_EQ(decision.steps[0].mission_id, "m1");
  EXPECT_EQ(decision.steps[0].denial_kind, ReservationAttempt::DenialKind::kStatic);
  EXPECT_EQ(decision.steps[0].route_locks, (std::vector<std::string>{"lockX"}));
  EXPECT_EQ(decision.steps[1].reason, "resource locked: lockX by other");
  EXPECT_EQ(decision.dispatched_mission_id, "m2");
}

// 时空窗被拒只告警 + 记账，**不发** reservation_denied（原实现的差异）。
TEST(DispatchTickPlanningTest, TimeSpaceDenialRecordsWaitWithoutDeniedEvent) {
  DispatchTickRequest request;
  request.candidates = {MakeCandidate("m1", AssignmentOutcome::kAssigned)};
  request.try_reserve = [](const std::string&, const std::vector<std::string>&) {
    ReservationAttempt a;
    a.success = false;
    a.denial_kind = ReservationAttempt::DenialKind::kTimeSpace;
    a.message = "no feasible time window";
    return a;
  };
  const auto decision = PlanDispatchTick(request);
  EXPECT_EQ(Kinds(decision), (std::vector<DispatchStep::Kind>{DispatchStep::Kind::kRecordWait}));
  EXPECT_EQ(decision.steps[0].denial_kind, ReservationAttempt::DenialKind::kTimeSpace);
  EXPECT_EQ(decision.steps[0].reason, "no feasible time window");
  EXPECT_FALSE(decision.dispatched);
}

TEST(DispatchTickPlanningTest, AllDeniedLeavesNothingDispatched) {
  DispatchTickRequest request;
  request.candidates = {MakeCandidate("m1", AssignmentOutcome::kAssigned),
                        MakeCandidate("m2", AssignmentOutcome::kAssigned)};
  request.try_reserve = [](const std::string&, const std::vector<std::string>&) {
    ReservationAttempt a;
    a.success = false;
    a.denial_kind = ReservationAttempt::DenialKind::kStatic;
    a.message = "locked";
    return a;
  };
  const auto decision = PlanDispatchTick(request);
  EXPECT_FALSE(decision.dispatched);
  EXPECT_EQ(Kinds(decision).size(), 4u);  // wait + denied × 2
  EXPECT_FALSE(decision.saw_deferred);
  EXPECT_FALSE(decision.saw_indeterminate);
}

TEST(DispatchTickPlanningTest, NoReserveCallbackMeansNoReservationNeeded) {
  DispatchTickRequest request;
  request.candidates = {MakeCandidate("m1", AssignmentOutcome::kAssigned)};
  const auto decision = PlanDispatchTick(request);
  EXPECT_TRUE(decision.dispatched);  // 回调为空 = 无需预留
}

TEST(DispatchTickPlanningTest, EmptyCandidateListDispatchesNothing) {
  const DispatchTickRequest request;
  const auto decision = PlanDispatchTick(request);
  EXPECT_TRUE(decision.steps.empty());
  EXPECT_FALSE(decision.dispatched);
}

// 混合场景：deferred 与 indeterminate 同时出现时两个标志都要置位，
// 由 ROS 侧按 indeterminate 优先告警（原实现 if/else if 的顺序）。
TEST(DispatchTickPlanningTest, DeferredAndIndeterminateBothReported) {
  DispatchTickRequest request;
  request.candidates = {MakeCandidate("m1", AssignmentOutcome::kDeferred),
                        MakeCandidate("m2", AssignmentOutcome::kIndeterminate)};
  const auto decision = PlanDispatchTick(request);
  EXPECT_TRUE(decision.saw_deferred);
  EXPECT_TRUE(decision.saw_indeterminate);
  EXPECT_TRUE(decision.steps.empty());
}

TEST(DispatchTickPlanningTest, DispatchClearsWaitRecordViaStep) {
  // 命中时 ROS 侧必须 erase pending_wait_：本函数把"要派发"作为唯一信号，
  // 由 kDispatch step 携带 mission_id 驱动该清理（不额外产出 step）。
  DispatchTickRequest request;
  request.candidates = {MakeCandidate("m1", AssignmentOutcome::kAssigned)};
  const auto decision = PlanDispatchTick(request);
  ASSERT_EQ(decision.steps.size(), 1u);
  EXPECT_EQ(decision.steps[0].kind, DispatchStep::Kind::kDispatch);
  EXPECT_EQ(decision.steps[0].mission_id, "m1");
}
