
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

#include "amr_dispatcher_bt/mission_resource_cleanup_behavior_tree.hpp"

#include <gtest/gtest.h>

namespace amr_dispatcher_bt {

using namespace amr_dispatcher_core::workflow;
using namespace amr_dispatcher_core::facility;
using namespace amr_dispatcher_core::catalog;


TEST(MissionResourceCleanupBehaviorTreeTest, ExistingResources_ReleaseThroughBtBranch) {
  FacilityReservationMap facility_reservations;
  MissionResourceMap resource_by_mission;
  facility_reservations["door_a"] = ResourceReservation{"holder", "mission_a", "reserved"};
  resource_by_mission["mission_a"] = "door_a";
  RouteLockMap route_locks;
  RouteLockReservationMap route_locks_by_mission;
  std::string message;
  ASSERT_TRUE(ReserveRouteLocksForMission(
      route_locks, route_locks_by_mission, "mission_a", {"route_node:receiving"}, &message));

  const auto result = TickMissionResourceCleanupBehaviorTree(
      MissionResourceCleanupBehaviorTreeInput{
          facility_reservations, resource_by_mission, route_locks, route_locks_by_mission,
          "mission_a"});

  EXPECT_TRUE(result.success);
  EXPECT_EQ("released", result.branch);
  EXPECT_TRUE(result.cleanup.facility_resource_released);
  EXPECT_TRUE(result.cleanup.route_locks_released);
  EXPECT_TRUE(facility_reservations.empty());
  EXPECT_TRUE(resource_by_mission.empty());
  EXPECT_TRUE(route_locks.empty());
  EXPECT_TRUE(route_locks_by_mission.empty());
}

TEST(MissionResourceCleanupBehaviorTreeTest, MissingMission_TakesNoopBranch) {
  FacilityReservationMap facility_reservations;
  MissionResourceMap resource_by_mission;
  RouteLockMap route_locks;
  RouteLockReservationMap route_locks_by_mission;

  const auto result = TickMissionResourceCleanupBehaviorTree(
      MissionResourceCleanupBehaviorTreeInput{
          facility_reservations, resource_by_mission, route_locks, route_locks_by_mission,
          "missing"});

  EXPECT_TRUE(result.success);
  EXPECT_EQ("noop", result.branch);
  EXPECT_FALSE(result.cleanup.facility_resource_released);
  EXPECT_FALSE(result.cleanup.route_locks_released);
}

}  // namespace amr_dispatcher_bt
