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
