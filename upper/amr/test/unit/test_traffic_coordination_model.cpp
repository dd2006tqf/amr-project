#include "amr_dispatcher_core/workflow/traffic_coordination_model.hpp"
#include "amr_dispatcher_core/dispatcher/deadlock_snapshot_assembly.hpp"
#include "amr_dispatcher_core/dispatcher/deadlock_detector.hpp"

#include <gtest/gtest.h>

using amr_dispatcher_core::dispatcher::AssembleDeadlockSnapshot;
using amr_dispatcher_core::dispatcher::DeadlockDetector;
using amr_dispatcher_core::dispatcher::DeadlockKind;
using amr_dispatcher_core::dispatcher::DeadlockSnapshotSources;
using amr_dispatcher_core::dispatcher::Mission;
using amr_dispatcher_core::workflow::IsValidTrafficStateTransition;
using amr_dispatcher_core::workflow::TrafficCoordinationState;
using amr_dispatcher_core::workflow::TrafficCoordinationStateLabel;

TEST(TrafficCoordinationModelTest, StateLabels) {
  EXPECT_STREQ(TrafficCoordinationStateLabel(TrafficCoordinationState::kRunning), "RUNNING");
  EXPECT_STREQ(TrafficCoordinationStateLabel(TrafficCoordinationState::kBlocked), "BLOCKED");
  EXPECT_STREQ(TrafficCoordinationStateLabel(TrafficCoordinationState::kReplanning), "REPLANNING");
  EXPECT_STREQ(TrafficCoordinationStateLabel(TrafficCoordinationState::kWaitingForResource), "WAITING_FOR_RESOURCE");
  EXPECT_STREQ(TrafficCoordinationStateLabel(TrafficCoordinationState::kRecoveryFailed), "RECOVERY_FAILED");
}

TEST(TrafficCoordinationModelTest, StateTransitions) {
  // 正常行进 -> 遇到阻塞
  EXPECT_TRUE(IsValidTrafficStateTransition(TrafficCoordinationState::kRunning, TrafficCoordinationState::kBlocked));
  // 阻塞 -> 重规划
  EXPECT_TRUE(IsValidTrafficStateTransition(TrafficCoordinationState::kBlocked, TrafficCoordinationState::kReplanning));
  // 重规划 -> 成功恢复行进
  EXPECT_TRUE(IsValidTrafficStateTransition(TrafficCoordinationState::kReplanning, TrafficCoordinationState::kRunning));
  // 阻塞 -> 资源等待
  EXPECT_TRUE(IsValidTrafficStateTransition(TrafficCoordinationState::kBlocked, TrafficCoordinationState::kWaitingForResource));
  // 资源等待 -> 恢复行进
  EXPECT_TRUE(IsValidTrafficStateTransition(TrafficCoordinationState::kWaitingForResource, TrafficCoordinationState::kRunning));
  // 资源等待 -> 失败
  EXPECT_TRUE(IsValidTrafficStateTransition(TrafficCoordinationState::kWaitingForResource, TrafficCoordinationState::kRecoveryFailed));
  // 非法跳变：运行中直接跳到失败
  EXPECT_FALSE(IsValidTrafficStateTransition(TrafficCoordinationState::kRunning, TrafficCoordinationState::kRecoveryFailed));
}

TEST(TrafficCoordinationModelTest, InFlightWaitDetectsMutualDeadlockCycle) {
  // 两台在途运行中的车：
  // 车 1 (m1) 占有 route_edge:A__B，正在等待申请 route_edge:B__C
  // 车 2 (m2) 占有 route_edge:B__C，正在等待申请 route_edge:A__B
  DeadlockSnapshotSources sources;
  Mission m1; m1.id = "m1";
  Mission m2; m2.id = "m2";
  sources.active_missions["m1"] = m1;
  sources.active_missions["m2"] = m2;

  sources.reserved_resources = {
      {"route_edge:A__B", "m1"},
      {"route_edge:B__C", "m2"},
  };

  sources.in_flight_pending_wait["m1"] = {"route_edge:B__C"};
  sources.in_flight_pending_wait["m2"] = {"route_edge:A__B"};

  const auto assembly = AssembleDeadlockSnapshot(sources);
  DeadlockDetector detector;
  const auto findings = detector.Detect(assembly.snapshot);

  ASSERT_FALSE(findings.empty());
  bool found_mutual_cycle = false;
  for (const auto& finding : findings) {
    if (finding.kind == DeadlockKind::kMutualBlock) {
      found_mutual_cycle = true;
      EXPECT_EQ(finding.mission_ids.size(), 2u);
      break;
    }
  }
  EXPECT_TRUE(found_mutual_cycle);
}
