#include "amr_dispatcher_core/dispatcher/mission_state.hpp"

#include <gtest/gtest.h>

using namespace amr_dispatcher_core::dispatcher;

TEST(MissionStateMachineTest, AcceptsAllowedTransitions) {
  MissionStateMachine m;
  EXPECT_TRUE(m.Transition(MissionState::kActive, MissionResult::kNone));
  EXPECT_TRUE(m.Transition(MissionState::kPaused, MissionResult::kNone));
  EXPECT_TRUE(m.Transition(MissionState::kActive, MissionResult::kNone));
  EXPECT_TRUE(m.Finish(MissionResult::kSucceeded));
  EXPECT_EQ(m.state(), MissionState::kFinished);
  EXPECT_EQ(m.result(), MissionResult::kSucceeded);
}

TEST(MissionStateMachineTest, RejectsIllegalTransitions) {
  MissionStateMachine m;
  std::string reason;
  EXPECT_FALSE(m.Transition(MissionState::kFinished, MissionResult::kNone, &reason));
  EXPECT_FALSE(reason.empty());
}

TEST(MissionStateMachineTest, FinishRequiresExplicitResult) {
  MissionStateMachine m;
  EXPECT_TRUE(m.Transition(MissionState::kActive, MissionResult::kNone));
  std::string reason;
  EXPECT_FALSE(m.Finish(MissionResult::kNone, &reason));
  EXPECT_FALSE(reason.empty());
}

TEST(MissionStateMachineTest, TerminalClearsNonTerminalResult) {
  MissionStateMachine m;
  EXPECT_TRUE(m.Transition(MissionState::kActive, MissionResult::kNone));
  EXPECT_EQ(m.result(), MissionResult::kNone);
  EXPECT_TRUE(m.Finish(MissionResult::kFailed));
  EXPECT_EQ(m.result(), MissionResult::kFailed);
}

TEST(MissionStateMachineTest, NoTransitionOutOfFinished) {
  MissionStateMachine m;
  EXPECT_TRUE(m.Finish(MissionResult::kCanceled));
  std::string reason;
  EXPECT_FALSE(m.Transition(MissionState::kActive, MissionResult::kNone, &reason));
  EXPECT_FALSE(m.Finish(MissionResult::kSucceeded, &reason));
  EXPECT_FALSE(reason.empty());
}

TEST(MissionStateMachineTest, PausedToPendingAllowed) {
  MissionStateMachine m;
  EXPECT_TRUE(m.Transition(MissionState::kActive, MissionResult::kNone));
  EXPECT_TRUE(m.Transition(MissionState::kPaused, MissionResult::kNone));
  EXPECT_TRUE(m.Transition(MissionState::kPending, MissionResult::kNone));
}

TEST(MissionStateMachineTest, ParsesStrings) {
  EXPECT_EQ(ParseMissionState("ACTIVE").value(), MissionState::kActive);
  EXPECT_EQ(ParseMissionResult("FAILED").value(), MissionResult::kFailed);
  EXPECT_FALSE(ParseMissionState("UNKNOWN").has_value());
  EXPECT_EQ(ToString(MissionState::kPaused), "PAUSED");
}

// ---- A2：身份语义分离（fleet_id 保持原义 / assigned_vehicle_id 新增）----

TEST(MissionIdentityFieldsTest, FleetIdAndAssignedVehicleIdAreIndependent) {
  amr_dispatcher_core::dispatcher::Mission m;
  // 车队归属：V1.2 多机预留，语义不变
  m.fleet_id = "fleet_north";
  // 承接车辆：选车产出后才落定
  EXPECT_TRUE(m.assigned_vehicle_id.empty());

  m.assigned_vehicle_id = "amr_07";
  EXPECT_EQ(m.fleet_id, "fleet_north");
  EXPECT_EQ(m.assigned_vehicle_id, "amr_07");
}

TEST(MissionIdentityFieldsTest, AssignedVehicleIdDefaultsEmpty_MeaningAutoSelection) {
  amr_dispatcher_core::dispatcher::Mission m;
  EXPECT_TRUE(m.assigned_vehicle_id.empty());
  EXPECT_TRUE(m.fleet_id.empty());
}

TEST(MissionIdentityFieldsTest, StateMachineCarriesBothFieldsThroughTransitions) {
  amr_dispatcher_core::dispatcher::MissionStateMachine m;
  m.mutable_mission().fleet_id = "fleet_west";
  m.mutable_mission().assigned_vehicle_id = "amr_03";

  EXPECT_TRUE(m.Transition(amr_dispatcher_core::dispatcher::MissionState::kActive,
                           amr_dispatcher_core::dispatcher::MissionResult::kNone));
  EXPECT_EQ(m.mission().fleet_id, "fleet_west");
  EXPECT_EQ(m.mission().assigned_vehicle_id, "amr_03");

  EXPECT_TRUE(m.Finish(amr_dispatcher_core::dispatcher::MissionResult::kSucceeded));
  EXPECT_EQ(m.mission().assigned_vehicle_id, "amr_03");
}
