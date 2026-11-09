#include "amr_dispatcher_core/fleet/vehicle_assignment_request_builder.hpp"

#include <algorithm>

#include "amr_dispatcher_core/fleet/fleet_catalog.hpp"

namespace amr_dispatcher_core::fleet {

std::vector<std::size_t> StableVehicleOrder(const std::vector<VehicleRuntimeState>& vehicles) {
  std::vector<std::size_t> order(vehicles.size());
  for (std::size_t i = 0; i < order.size(); ++i) {
    order[i] = i;
  }
  // 按 vehicle_id 字典序；同 id（不应出现）时用原始下标兜底，保证全序
  std::stable_sort(order.begin(), order.end(), [&vehicles](std::size_t a, std::size_t b) {
    if (vehicles[a].vehicle_id != vehicles[b].vehicle_id) {
      return vehicles[a].vehicle_id < vehicles[b].vehicle_id;
    }
    return a < b;
  });
  return order;
}

VehicleAssignmentRequest BuildVehicleAssignmentRequest(
    const VehicleAssignmentRequestInputs& inputs) {
  VehicleAssignmentRequest request;
  request.requested_vehicle_id = inputs.requested_vehicle_id;
  request.pickup_station_id = inputs.pickup_station_id;
  request.dropoff_station_id = inputs.dropoff_station_id;
  request.required_capability = inputs.required_capability;
  request.min_battery_percent = inputs.min_battery_percent;
  request.state_max_age_ms = inputs.state_max_age_ms;

  // 保序：调用方给的是无序容器导出的列表，遍历序不稳定；先按 id 排序，
  // 保证同分时裁决顺序确定（核心按输入顺序 tiebreak）。
  const auto order = StableVehicleOrder(inputs.vehicles);
  request.vehicles.reserve(order.size());
  for (const auto idx : order) {
    request.vehicles.push_back(inputs.vehicles[idx]);
  }

  // 到取货点的路网成本（评审问题 3：路网路径成本，非欧氏距离）
  const bool can_estimate = inputs.catalog != nullptr && inputs.catalog_loaded &&
                            !inputs.pickup_station_id.empty();
  if (!can_estimate) {
    return request;
  }
  for (const auto& state : request.vehicles) {
    const auto cost = EstimateVehicleApproachCost(
        *inputs.catalog, state.position_x, state.position_y, state.current_station_id,
        inputs.pickup_station_id);
    if (cost.has_value()) {
      request.distance_to_pickup_m[state.vehicle_id] = *cost;
    }
  }
  return request;
}

}  // namespace amr_dispatcher_core::fleet
