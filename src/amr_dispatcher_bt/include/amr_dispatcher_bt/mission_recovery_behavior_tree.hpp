
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

#include <cstdint>
#include <optional>
#include <string>

#include "amr_dispatcher_core/workflow/queued_mission.hpp"
#include "amr_dispatcher_core/workflow/mission_recovery_policy.hpp"

namespace amr_dispatcher_bt {

using namespace amr_dispatcher_core::workflow;
using namespace amr_dispatcher_core::facility;
using namespace amr_dispatcher_core::catalog;


struct MissionRecoveryBehaviorTreeInput {
  MissionRecoveryConfig config;
  int retry_count = 0;
  bool is_dock_mission = false;
  std::string mission_id;
  std::string failure_message;
  std::optional<MissionProfile> active_profile;
  std::string mission_file;
  int recovery_priority = 0;
  std::uint64_t next_queue_sequence = 0;
};

struct MissionRecoveryBehaviorTreeResult {
  bool success = false;
  std::string branch;
  MissionRecoveryDecision decision;
  std::string state;
  std::string state_message;
  bool recoverable = false;
  int retry_count = 0;
  std::uint64_t next_queue_sequence = 0;
  bool queue_running = false;
  bool queue_start_requested = false;
  bool request_dock_return = false;
  bool stop_queue = false;
  std::optional<QueuedMission> retry_mission;
};

std::string DefaultMissionRecoveryBehaviorTreeXml();
MissionRecoveryBehaviorTreeResult TickMissionRecoveryBehaviorTree(
    const MissionRecoveryBehaviorTreeInput& input);

}  // namespace amr_dispatcher_bt
