#include "amr_dispatcher_core/fleet/fleet_catalog.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <unordered_set>

namespace amr_dispatcher_core::fleet {
namespace {

using amr_dispatcher_core::catalog::EstimateRouteDistance;
using amr_dispatcher_core::catalog::FindStation;
using amr_dispatcher_core::catalog::Station;
using amr_dispatcher_core::catalog::StationCatalog;

std::string Trim(const std::string& value) {
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return "";
  }
  const auto last = value.find_last_not_of(" \t\r\n");
  return value.substr(first, last - first + 1);
}

std::optional<std::string> ValueAfterColon(const std::string& line, const std::string& key) {
  const std::string prefix = key + ":";
  if (line.rfind(prefix, 0) != 0) {
    return std::nullopt;
  }
  return Trim(line.substr(prefix.size()));
}

bool ParseBool(const std::string& value) {
  return value == "true" || value == "True" || value == "1";
}

std::optional<double> ParseDouble(const std::string& value) {
  std::istringstream parser(value);
  double result = 0.0;
  parser >> result;
  if (parser.fail() || !std::isfinite(result)) {
    return std::nullopt;
  }
  return result;
}

std::vector<std::string> SplitCommaList(const std::string& value) {
  std::vector<std::string> result;
  std::stringstream stream(value);
  std::string item;
  while (std::getline(stream, item, ',')) {
    item = Trim(item);
    if (!item.empty()) {
      result.push_back(item);
    }
  }
  return result;
}

bool RobotAvailableForTask(const FleetRobot& robot) {
  return robot.enabled && (robot.state == "available" || robot.state == "idle");
}

bool RobotUnavailable(
    const FleetRobot& robot, const std::vector<std::string>& unavailable_robot_ids) {
  return std::find(unavailable_robot_ids.begin(), unavailable_robot_ids.end(), robot.id) !=
         unavailable_robot_ids.end();
}

std::optional<double> RouteOrEuclideanDistance(
    const StationCatalog& catalog, const std::string& from, const std::string& to) {
  if (from.empty() || to.empty()) {
    return std::nullopt;
  }
  if (const auto route_distance = EstimateRouteDistance(catalog, from, to)) {
    return route_distance;
  }
  const auto* from_station = FindStation(catalog, from);
  const auto* to_station = FindStation(catalog, to);
  if (from_station == nullptr || to_station == nullptr ||
      from_station->frame_id != to_station->frame_id) {
    return std::nullopt;
  }
  return std::hypot(from_station->x - to_station->x, from_station->y - to_station->y);
}

}  // namespace

std::optional<FleetCatalog> LoadFleetCatalog(const std::filesystem::path& path) {
  std::ifstream input(path);
  if (!input) {
    return std::nullopt;
  }

  FleetCatalog catalog;
  FleetRobot current;
  bool in_robot = false;
  bool has_id = false;
  std::string section;
  std::string line;

  auto flush_robot = [&]() {
    if (in_robot && has_id) {
      catalog.robots.push_back(current);
    }
    current = FleetRobot{};
    in_robot = false;
    has_id = false;
  };

  while (std::getline(input, line)) {
    line = Trim(line);
    if (line.empty() || line.front() == '#') {
      continue;
    }
    if (line == "robots:" || line == "fleet:") {
      flush_robot();
      section = line.substr(0, line.size() - 1);
      continue;
    }
    if (line.rfind("- ", 0) == 0) {
      if (section != "robots" && section != "fleet") {
        continue;
      }
      flush_robot();
      in_robot = true;
      line = Trim(line.substr(2));
    }
    if (!in_robot) {
      continue;
    }
    if (auto value = ValueAfterColon(line, "id")) {
      current.id = *value;
      has_id = !current.id.empty();
      continue;
    }
    if (auto value = ValueAfterColon(line, "enabled")) {
      current.enabled = ParseBool(*value);
      continue;
    }
    if (auto value = ValueAfterColon(line, "state")) {
      current.state = value->empty() ? "available" : *value;
      continue;
    }
    if (auto value = ValueAfterColon(line, "battery_voltage")) {
      if (auto parsed = ParseDouble(*value)) {
        current.battery_voltage = *parsed;
      }
      continue;
    }
    if (auto value = ValueAfterColon(line, "max_payload_kg")) {
      if (auto parsed = ParseDouble(*value)) {
        current.max_payload_kg = *parsed;
      }
      continue;
    }
    if (auto value = ValueAfterColon(line, "current_station_id")) {
      current.current_station_id = *value;
      continue;
    }
    if (auto value = ValueAfterColon(line, "capabilities")) {
      current.capabilities = SplitCommaList(*value);
      continue;
    }
    if (auto value = ValueAfterColon(line, "backend")) {
      // 执行后端（P0.1）：只接受已知标签，未知值回落 local_sim 并由校验报错
      current.backend = value->empty() ? "local_sim" : *value;
      continue;
    }
  }
  flush_robot();

  std::string message;
  if (!ValidateFleetCatalog(catalog, &message)) {
    return std::nullopt;
  }
  return catalog;
}

const FleetRobot* FindFleetRobot(const FleetCatalog& catalog, const std::string& robot_id) {
  for (const auto& robot : catalog.robots) {
    if (robot.id == robot_id) {
      return &robot;
    }
  }
  return nullptr;
}

bool RobotHasCapability(const FleetRobot& robot, const std::string& capability) {
  if (capability.empty()) {
    return true;
  }
  return std::find(robot.capabilities.begin(), robot.capabilities.end(), capability) !=
         robot.capabilities.end();
}

const FleetRobot* SelectFleetRobot(
    const FleetCatalog& catalog, const std::string& required_capability,
    const std::vector<std::string>& unavailable_robot_ids) {
  const FleetRobot* selected = nullptr;
  for (const auto& robot : catalog.robots) {
    if (!RobotAvailableForTask(robot) || !RobotHasCapability(robot, required_capability)) {
      continue;
    }
    if (RobotUnavailable(robot, unavailable_robot_ids)) {
      continue;
    }
    if (selected == nullptr || robot.battery_voltage > selected->battery_voltage) {
      selected = &robot;
    }
  }
  return selected;
}

using amr_dispatcher_core::catalog::StationCatalog;

const FleetRobot* SelectFleetRobotForStationTask(
    const FleetCatalog& fleet_catalog, const StationCatalog& station_catalog,
    const std::string& required_capability, const std::string& pickup_station,
    const std::string& dropoff_station, const std::vector<std::string>& unavailable_robot_ids,
    FleetRobotTaskScore* selected_score) {
  const auto task_distance = RouteOrEuclideanDistance(station_catalog, pickup_station, dropoff_station);
  if (!task_distance.has_value()) {
    return SelectFleetRobot(fleet_catalog, required_capability, unavailable_robot_ids);
  }

  const FleetRobot* selected = nullptr;
  FleetRobotTaskScore best_score;
  for (const auto& robot : fleet_catalog.robots) {
    if (!RobotAvailableForTask(robot) || !RobotHasCapability(robot, required_capability) ||
        RobotUnavailable(robot, unavailable_robot_ids)) {
      continue;
    }
    const auto approach_distance =
        RouteOrEuclideanDistance(station_catalog, robot.current_station_id, pickup_station);
    if (!approach_distance.has_value()) {
      continue;
    }
    FleetRobotTaskScore score;
    score.robot_id = robot.id;
    score.approach_distance_m = *approach_distance;
    score.task_distance_m = *task_distance;
    score.total_distance_m = score.approach_distance_m + score.task_distance_m;
    score.battery_voltage = robot.battery_voltage;
    if (selected == nullptr || score.total_distance_m < best_score.total_distance_m ||
        (score.total_distance_m == best_score.total_distance_m &&
         score.battery_voltage > best_score.battery_voltage)) {
      selected = &robot;
      best_score = score;
    }
  }
  if (selected != nullptr && selected_score != nullptr) {
    *selected_score = best_score;
  }
  if (selected == nullptr) {
    return SelectFleetRobot(fleet_catalog, required_capability, unavailable_robot_ids);
  }
  return selected;
}

std::optional<double> EstimateVehicleApproachCost(
    const StationCatalog& catalog, const double vehicle_x, const double vehicle_y,
    const std::string& vehicle_station_id, const std::string& target_station_id) {
  if (target_station_id.empty()) {
    return std::nullopt;
  }
  const auto* target = FindStation(catalog, target_station_id);
  if (target == nullptr) {
    return std::nullopt;
  }
  // 规则 1：车挂在已知站点上 -> 直接算路网成本
  if (!vehicle_station_id.empty()) {
    return EstimateRouteDistance(catalog, vehicle_station_id, target_station_id);
  }
  // 规则 2：车在站点之间 -> 先直线接驳到最近站点，再叠路网成本
  const Station* nearest = nullptr;
  double nearest_dist = 0.0;
  for (const auto& station : catalog.stations) {
    if (station.frame_id != target->frame_id) {
      continue;  // 跨 frame 不比较
    }
    const double d = std::hypot(station.x - vehicle_x, station.y - vehicle_y);
    if (nearest == nullptr || d < nearest_dist) {
      nearest = &station;
      nearest_dist = d;
    }
  }
  if (nearest == nullptr) {
    return std::nullopt;
  }
  const auto route = EstimateRouteDistance(catalog, nearest->id, target_station_id);
  if (!route.has_value()) {
    return std::nullopt;
  }
  return nearest_dist + *route;
}

bool ValidateFleetCatalog(const FleetCatalog& catalog, std::string* message) {
  std::unordered_set<std::string> ids;
  for (const auto& robot : catalog.robots) {
    if (robot.id.empty()) {
      if (message != nullptr) {
        *message = "fleet robot id is empty";
      }
      return false;
    }
    if (!ids.insert(robot.id).second) {
      if (message != nullptr) {
        *message = "duplicate fleet robot id: " + robot.id;
      }
      return false;
    }
    if (!std::isfinite(robot.battery_voltage) || robot.battery_voltage < 0.0) {
      if (message != nullptr) {
        *message = "invalid battery voltage for fleet robot: " + robot.id;
      }
      return false;
    }
    if (!std::isfinite(robot.max_payload_kg) || robot.max_payload_kg < 0.0) {
      if (message != nullptr) {
        *message = "invalid payload for fleet robot: " + robot.id;
      }
      return false;
    }
    // 执行后端标签必须是已知值（local_sim / vda5050）——不猜、不静默回落
    if (robot.backend != "local_sim" && robot.backend != "vda5050") {
      if (message != nullptr) {
        *message = "unknown execution backend for fleet robot " + robot.id + ": " + robot.backend;
      }
      return false;
    }
  }
  if (message != nullptr) {
    *message = "fleet catalog ok";
  }
  return true;
}

}  // namespace amr_dispatcher_core::fleet
