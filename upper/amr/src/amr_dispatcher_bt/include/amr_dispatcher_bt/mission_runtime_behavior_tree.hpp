
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

#include "amr_dispatcher_core/workflow/mission_runtime_workflow.hpp"

namespace amr_dispatcher_bt {

using namespace amr_dispatcher_core::workflow;
using namespace amr_dispatcher_core::facility;
using namespace amr_dispatcher_core::catalog;


struct MissionRuntimeBehaviorTreeInput {
  MissionRuntimeTickInput tick;
};

struct MissionRuntimeBehaviorTreeResult {
  bool success = false;
  std::string branch;
  MissionRuntimeTickDecision decision;
};

std::string DefaultMissionRuntimeBehaviorTreeXml();
MissionRuntimeBehaviorTreeResult TickMissionRuntimeBehaviorTree(
    const MissionRuntimeBehaviorTreeInput& input);

}  // namespace amr_dispatcher_bt
