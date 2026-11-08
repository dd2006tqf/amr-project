#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace amr_dispatcher_core::workflow {

struct MissionWaypoint {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

struct MissionProfile {
  std::string mission_id;
  std::string frame_id = "map";
  bool loop = false;
  std::vector<MissionWaypoint> waypoints;
};

std::optional<MissionProfile> LoadMissionProfile(const std::filesystem::path& path);
bool ValidateMissionProfile(const MissionProfile& profile, std::string* message);

}  // namespace amr_dispatcher_core::workflow

