
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

#pragma once

#include <string>

#include "amr_dispatcher_core/workflow/mission_resource_cleanup.hpp"

namespace amr_dispatcher_bt {

using namespace amr_dispatcher_core::workflow;
using namespace amr_dispatcher_core::facility;
using namespace amr_dispatcher_core::catalog;


struct MissionResourceCleanupBehaviorTreeInput {
  FacilityReservationMap& facility_reservations;
  MissionResourceMap& facility_resource_by_mission;
  RouteLockMap& route_locks;
  RouteLockReservationMap& route_locks_by_mission;
  std::string mission_id;
};

struct MissionResourceCleanupBehaviorTreeResult {
  bool success = false;
  std::string branch;
  MissionResourceCleanupResult cleanup;
};

std::string DefaultMissionResourceCleanupBehaviorTreeXml();
MissionResourceCleanupBehaviorTreeResult TickMissionResourceCleanupBehaviorTree(
    MissionResourceCleanupBehaviorTreeInput input);

}  // namespace amr_dispatcher_bt
