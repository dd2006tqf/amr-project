// 选车请求构建（纯函数）。
//
// 原实现内联在 DispatcherNode::BuildAssignmentRequest 里，混了三件事：
//   1. 刷新车队运行时状态（IO：读订阅、读里程计）
//   2. 按 id 排序保证裁决确定性
//   3. 调 EstimateVehicleApproachCost 补"到取货点的路网成本"
// 其中 2、3 是纯逻辑，但因为没有独立入口，只能通过起 ROS 图来间接验证
// （"同分时谁先被选中"这种确定性属性尤其难以回归）。
//
// 这里把 2、3 抽出来；1 仍留在 ROS 侧（它必须读订阅）。
#pragma once

#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "amr_dispatcher_core/catalog/station_catalog.hpp"
#include "amr_dispatcher_core/fleet/vehicle_assignment.hpp"

namespace amr_dispatcher_core::fleet {

struct VehicleAssignmentRequestInputs {
  // 待选车辆（顺序不限：本函数会重排，保证同分裁决确定）
  std::vector<VehicleRuntimeState> vehicles;
  // 任务侧
  std::string requested_vehicle_id;
  std::string pickup_station_id;
  std::string dropoff_station_id;
  std::string required_capability;
  double min_battery_percent = 0.0;
  int state_max_age_ms = 5000;
  // 自动选车打分策略（透传给核心；显式指定车辆时不起作用）
  VehicleSelectionStrategy strategy = VehicleSelectionStrategy::kNearest;
  // 路网成本来源；nullptr 或未加载 -> 不填 distance_to_pickup_m
  const catalog::StationCatalog* catalog = nullptr;
  bool catalog_loaded = false;
};

VehicleAssignmentRequest BuildVehicleAssignmentRequest(const VehicleAssignmentRequestInputs& inputs);

// 排序规则单列出来，供调用方复用（例如把候选按同一顺序打日志）。
// 返回按 vehicle_id 字典序排列的下标。
std::vector<std::size_t> StableVehicleOrder(const std::vector<VehicleRuntimeState>& vehicles);

}  // namespace amr_dispatcher_core::fleet
