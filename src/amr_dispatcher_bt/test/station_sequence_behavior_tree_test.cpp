
#include "amr_dispatcher_core/workflow/docking_workflow.hpp"
#include "amr_dispatcher_core/workflow/mission_recovery_policy.hpp"
#include "amr_dispatcher_core/workflow/mission_result_workflow.hpp"
#include "amr_dispatcher_core/workflow/mission_runtime_workflow.hpp"
#include "amr_dispatcher_core/workflow/queued_mission.hpp"
#include "amr_dispatcher_core/workflow/station_sequence_workflow.hpp"
#include "amr_dispatcher_core/workflow/submit_order_router.hpp"
#include "amr_dispatcher_core/workflow/mission_resource_cleanup.hpp"
#include "amr_dispatcher_core/facility/facility_workflow.hpp"
#include "amr_dispatcher_core/facility/facility_reservation.hpp"
#include "amr_dispatcher_core/facility/facility_catalog.hpp"
#include "amr_dispatcher_core/catalog/station_catalog.hpp"

#include "amr_dispatcher_bt/station_sequence_behavior_tree.hpp"

#include <gtest/gtest.h>

namespace amr_dispatcher_bt {

using namespace amr_dispatcher_core::workflow;
using namespace amr_dispatcher_core::facility;
using namespace amr_dispatcher_core::catalog;

namespace {

StationSequenceLeg MakeLeg(const std::string& mission_id) {
  StationSequenceLeg leg;
  leg.profile.mission_id = mission_id;
  leg.profile.frame_id = "map";
  leg.profile.waypoints = {
      MissionWaypoint{0.0, 0.0, 0.0}, MissionWaypoint{1.0, 0.0, 0.0}};
  leg.pickup_id = "pickup";
  leg.dropoff_id = "dropoff";
  return leg;
}

MissionPreflightResult MakePreflight(const bool allowed, const std::string& message) {
  MissionPreflightResult preflight;
  preflight.allowed = allowed;
  preflight.message = message;
  return preflight;
}

}  // namespace

TEST(StationSequenceBehaviorTreeTest, AcceptedSequence_BuildsQueuedMissionsThroughBtAction) {
  StationSequenceBehaviorTreeInput input;
  input.legs = {
      MakeLeg("station_sequence_order_1_leg_1"),
      MakeLeg("station_sequence_order_1_leg_2"),
  };
  input.priority = 6;
  input.first_sequence = 100;

  const auto result = TickStationSequenceBehaviorTree(input);

  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.branch, "station_sequence_queue");
  EXPECT_EQ(result.message, "queued station sequence with 2 leg(s)");
  ASSERT_EQ(result.mission_ids.size(), 2U);
  EXPECT_EQ(result.mission_ids[0], "station_sequence_order_1_leg_1");
  ASSERT_EQ(result.queued_missions.size(), 2U);
  EXPECT_EQ(result.queued_missions[0].priority, 6);
  EXPECT_EQ(result.queued_missions[0].sequence, 100U);
  EXPECT_EQ(result.queued_missions[1].sequence, 101U);
}

TEST(StationSequenceBehaviorTreeTest, Conflict_RejectsBeforeQueueBuild) {
  StationSequenceBehaviorTreeInput input;
  input.legs = {MakeLeg("station_sequence_order_1_leg_1")};
  input.active_or_queued_mission_ids = {"station_sequence_order_1_leg_1"};

  const auto result = TickStationSequenceBehaviorTree(input);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(
      result.message,
      "station sequence leg already active or queued: station_sequence_order_1_leg_1");
  EXPECT_TRUE(result.queued_missions.empty());
}

TEST(StationSequenceBehaviorTreeTest, RejectedPreflight_StopsQueueBuildWithContext) {
  StationSequenceBehaviorTreeInput input;
  input.legs = {MakeLeg("station_sequence_order_1_leg_1")};
  input.preflights = {MakePreflight(false, "battery below minimum")};

  const auto result = TickStationSequenceBehaviorTree(input);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(
      result.message,
      "station sequence preflight rejected for station_sequence_order_1_leg_1: "
      "battery below minimum");
  EXPECT_TRUE(result.queued_missions.empty());
}

}  // namespace amr_dispatcher_bt
