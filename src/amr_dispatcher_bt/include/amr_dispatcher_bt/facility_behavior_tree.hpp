
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

#include "amr_dispatcher_core/facility/facility_reservation.hpp"
#include "amr_dispatcher_core/facility/facility_catalog.hpp"
#include "amr_dispatcher_core/facility/facility_workflow.hpp"

namespace amr_dispatcher_bt {

using namespace amr_dispatcher_core::workflow;
using namespace amr_dispatcher_core::facility;
using namespace amr_dispatcher_core::catalog;


struct FacilityActionBehaviorTreeInput {
  FacilityCatalog catalog;
  FacilityReservationMap reservations;
  MissionResourceMap resource_by_mission;
  std::string request_id;
  std::string resource_id;
  std::string resource_type;
  std::string action = "reserve";
  bool hold_after_action = true;
};

struct FacilityActionBehaviorTreeResult {
  bool success = false;
  std::string branch;
  std::string message;
  std::string mission_id;
  std::string resource_id;
  FacilityReservationMap reservations;
  MissionResourceMap resource_by_mission;
};

struct LiftSessionBehaviorTreeInput {
  FacilityReservationMap reservations;
  FacilityResource lift;
  std::string session_id;
  std::string requester_id;
  bool release_existing_for_holder = true;
  bool release = false;
};

struct LiftSessionBehaviorTreeResult {
  bool success = false;
  std::string branch;
  LiftSessionResult lift_result;
  FacilityReservationMap reservations;
};

std::string DefaultFacilityActionBehaviorTreeXml();
FacilityActionBehaviorTreeResult TickFacilityActionBehaviorTree(
    const FacilityActionBehaviorTreeInput& input);

std::string DefaultLiftSessionBehaviorTreeXml();
LiftSessionBehaviorTreeResult TickLiftSessionBehaviorTree(
    const LiftSessionBehaviorTreeInput& input);

}  // namespace amr_dispatcher_bt
