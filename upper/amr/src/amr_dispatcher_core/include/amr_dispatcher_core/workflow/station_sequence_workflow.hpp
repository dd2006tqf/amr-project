#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_set>
#include <vector>

#include "amr_dispatcher_core/workflow/mission_preflight.hpp"
#include "amr_dispatcher_core/workflow/mission_profile_builders.hpp"
#include "amr_dispatcher_core/workflow/queued_mission.hpp"

namespace amr_dispatcher_core::workflow {

struct StationSequenceAdmissionResult {
  bool accepted = true;
  std::string message = "station sequence accepted";
};

StationSequenceAdmissionResult CheckStationSequenceConflicts(
    const std::vector<StationSequenceLeg>& legs,
    const std::unordered_set<std::string>& active_or_queued_mission_ids);

StationSequenceAdmissionResult CheckStationSequenceLegPreflight(
    const StationSequenceLeg& leg, const MissionPreflightResult& preflight);

std::vector<std::string> StationSequenceMissionIds(
    const std::vector<StationSequenceLeg>& legs);

std::vector<QueuedMission> BuildStationSequenceQueuedMissions(
    const std::vector<StationSequenceLeg>& legs, int priority,
    std::uint64_t first_sequence);

std::string BuildStationSequenceQueuedMessage(std::size_t queued_count);

}  // namespace amr_dispatcher_core::workflow
