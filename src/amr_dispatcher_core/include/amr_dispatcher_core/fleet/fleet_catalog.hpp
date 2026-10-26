#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include "amr_dispatcher_core/catalog/station_catalog.hpp"

namespace amr_dispatcher_core::fleet {

using amr_dispatcher_core::catalog::StationCatalog;

struct FleetRobot {
  std::string id;
  bool enabled = true;
  std::string state = "available";
  double battery_voltage = 0.0;
  double max_payload_kg = 0.0;
  std::string current_station_id;
  std::vector<std::string> capabilities;
  // 执行后端（P0.1）：local_sim（本地模拟车，调度发 /plan/{id}）|
  // vda5050（外部标准协议车，调度只发 MQTT order）。空 = local_sim。
  // 该字段决定"谁在跑这条路径"，不影响选车阶段的硬约束判定。
  std::string backend = "local_sim";
};

struct FleetCatalog {
  std::vector<FleetRobot> robots;
};

struct FleetRobotTaskScore {
  std::string robot_id;
  double approach_distance_m = 0.0;
  double task_distance_m = 0.0;
  double total_distance_m = 0.0;
  double battery_voltage = 0.0;
};

std::optional<FleetCatalog> LoadFleetCatalog(const std::filesystem::path& path);
const FleetRobot* FindFleetRobot(const FleetCatalog& catalog, const std::string& robot_id);
bool RobotHasCapability(const FleetRobot& robot, const std::string& capability);
const FleetRobot* SelectFleetRobot(
    const FleetCatalog& catalog, const std::string& required_capability,
    const std::vector<std::string>& unavailable_robot_ids);
const FleetRobot* SelectFleetRobotForStationTask(
    const FleetCatalog& fleet_catalog, const StationCatalog& station_catalog,
    const std::string& required_capability, const std::string& pickup_station,
    const std::string& dropoff_station, const std::vector<std::string>& unavailable_robot_ids,
    FleetRobotTaskScore* selected_score);
bool ValidateFleetCatalog(const FleetCatalog& catalog, std::string* message);

// 车辆 -> 目标站点的**路网成本**（米），用于选车排序（评审问题 3：不用欧氏距离）。
// 规则：
//   1. 车辆当前挂在已知站点上（vehicle_station_id 非空）-> 该站点到目标站点的
//      路网路径成本（EstimateRouteDistance）；
//   2. 车辆不在已知站点上 -> 取距车辆坐标最近的站点，先算"直线接驳距离"
//      再叠加"该站点到目标站点的路网成本"；
//   3. 目标站点不存在 / 目录里找不到任何站点 -> 返回 nullopt（距离不可算，
//      该车在自动选车时**不合格**，绝不盲派）。
std::optional<double> EstimateVehicleApproachCost(
    const StationCatalog& catalog, double vehicle_x, double vehicle_y,
    const std::string& vehicle_station_id, const std::string& target_station_id);

}  // namespace amr_dispatcher_core::fleet
