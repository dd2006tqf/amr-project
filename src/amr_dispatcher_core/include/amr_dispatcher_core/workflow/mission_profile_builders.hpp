#pragma once

#include <optional>
#include <string>
#include <vector>

#include "amr_dispatcher_core/workflow/mission_profile.hpp"
#include "amr_dispatcher_core/catalog/station_catalog.hpp"

namespace amr_dispatcher_core::workflow {

using amr_dispatcher_core::catalog::Station;
using amr_dispatcher_core::catalog::StationCatalog;

struct StationPair {
  Station pickup;
  Station dropoff;
};

struct StationSequenceLeg {
  MissionProfile profile;
  std::string pickup_id;
  std::string dropoff_id;
};

MissionProfile BuildTwoWaypointProfile(
    const std::string& mission_id, const std::string& frame_id, double pickup_x,
    double pickup_y, double pickup_yaw, double dropoff_x, double dropoff_y,
    double dropoff_yaw);

MissionProfile BuildStationWaypointProfile(const std::string& mission_id, const Station& station);

std::optional<StationPair> ResolveStationPair(
    const StationCatalog& catalog, const std::string& pickup_station,
    const std::string& dropoff_station, std::string* message);

std::optional<std::vector<StationSequenceLeg>> BuildStationSequenceProfiles(
    const StationCatalog& catalog, const std::vector<std::string>& station_ids,
    const std::string& base_id, std::string* message);

std::optional<MissionProfile> BuildStationSequenceEstimateProfile(
    const StationCatalog& catalog, const std::vector<std::string>& station_ids,
    std::string* message);

}  // namespace amr_dispatcher_core::workflow
