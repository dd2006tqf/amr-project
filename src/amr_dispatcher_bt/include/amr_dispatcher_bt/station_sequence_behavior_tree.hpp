
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
#include <unordered_set>
#include <vector>

#include "amr_dispatcher_core/workflow/station_sequence_workflow.hpp"

namespace amr_dispatcher_bt {

using namespace amr_dispatcher_core::workflow;
using namespace amr_dispatcher_core::facility;
using namespace amr_dispatcher_core::catalog;


struct StationSequenceBehaviorTreeInput {
  std::vector<StationSequenceLeg> legs;
  std::unordered_set<std::string> active_or_queued_mission_ids;
  std::vector<MissionPreflightResult> preflights;
  int priority = 0;
  std::uint64_t first_sequence = 0;
};

struct StationSequenceBehaviorTreeResult {
  bool success = false;
  std::string branch;
  std::string message;
  std::vector<std::string> mission_ids;
  std::vector<QueuedMission> queued_missions;
};

std::string DefaultStationSequenceBehaviorTreeXml();
StationSequenceBehaviorTreeResult TickStationSequenceBehaviorTree(
    const StationSequenceBehaviorTreeInput& input);

}  // namespace amr_dispatcher_bt
