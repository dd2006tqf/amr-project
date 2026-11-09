#include "amr_dispatcher_core/fleet/vehicle_assignment.hpp"

#include <algorithm>
#include <limits>
#include <string>
#include <vector>

namespace amr_dispatcher_core::fleet {

namespace {

std::string JoinReasons(const std::vector<std::string>& parts) {
  std::string joined;
  for (std::size_t i = 0; i < parts.size(); ++i) {
    if (i) joined += ", ";
    joined += parts[i];
  }
  return joined;
}

}  // namespace

bool VehicleHasCapability(const VehicleRuntimeState& vehicle, const std::string& capability) {
  // capability 为空 = 不要求任何能力
  if (capability.empty()) {
    return true;
  }
  // capabilities 为空 = 该车未声明能力限制（单车最小配置仍可选中）
  if (vehicle.capabilities.empty()) {
    return true;
  }
  return std::find(vehicle.capabilities.begin(), vehicle.capabilities.end(), capability) !=
         vehicle.capabilities.end();
}

VehicleAssignmentDecision PlanVehicleAssignment(const VehicleAssignmentRequest& request) {
  VehicleAssignmentDecision decision;

  if (request.vehicles.empty()) {
    decision.reject_reason = "no managed vehicles configured";
    decision.strategy = "rejected";
    return decision;
  }

  const std::string& capability = request.required_capability;

  // ---- 规则 1：显式指定车辆 ----
  if (!request.requested_vehicle_id.empty()) {
    const auto it =
        std::find_if(request.vehicles.begin(), request.vehicles.end(),
                     [&](const VehicleRuntimeState& v) { return v.vehicle_id == request.requested_vehicle_id; });
    if (it == request.vehicles.end()) {
      // 显式指定的车不在受管清单内 —— 不静默改派
      decision.reject_reason =
          "vehicle '" + request.requested_vehicle_id + "' is not in the managed vehicle list";
      decision.strategy = "rejected";
      return decision;
    }
    const auto& vehicle = *it;
    if (!vehicle.online) {
      decision.reject_reason = "vehicle '" + vehicle.vehicle_id + "' is offline";
      decision.strategy = "rejected";
      return decision;
    }
    if (!vehicle.enabled) {
      decision.reject_reason = "vehicle '" + vehicle.vehicle_id + "' is disabled";
      decision.strategy = "rejected";
      return decision;
    }
    if (vehicle.busy) {
      decision.reject_reason = "vehicle '" + vehicle.vehicle_id + "' is busy";
      decision.strategy = "rejected";
      return decision;
    }
    if (!VehicleHasCapability(vehicle, capability)) {
      decision.reject_reason = "vehicle '" + vehicle.vehicle_id + "' lacks required capability '" +
                              capability + "'";
      decision.strategy = "rejected";
      return decision;
    }
    decision.assigned = true;
    decision.vehicle_id = vehicle.vehicle_id;
    decision.strategy = "explicit";
    return decision;
  }

  // ---- 规则 2：过滤候选 ----
  std::vector<const VehicleRuntimeState*> eligible;
  eligible.reserve(request.vehicles.size());
  for (const auto& vehicle : request.vehicles) {
    if (!vehicle.online || !vehicle.enabled || vehicle.busy) {
      continue;
    }
    if (!VehicleHasCapability(vehicle, capability)) {
      continue;
    }
    if (vehicle.battery_percent < request.min_battery_percent) {
      continue;
    }
    eligible.push_back(&vehicle);
  }

  if (eligible.empty()) {
    // ---- 规则 4：无候选，拒绝并说明主因 ----
    std::vector<std::string> blockers;
    bool capable_exists = false;
    for (const auto& vehicle : request.vehicles) {
      if (!VehicleHasCapability(vehicle, capability)) {
        continue;
      }
      capable_exists = true;
      if (!vehicle.online) {
        blockers.push_back("offline");
      }
      if (!vehicle.enabled) {
        blockers.push_back("disabled");
      }
      if (vehicle.busy) {
        blockers.push_back("busy");
      }
      if (vehicle.battery_percent < request.min_battery_percent) {
        blockers.push_back("battery below threshold");
      }
    }
    std::string reason;
    if (capable_exists) {
      reason = "no eligible vehicle with capability '" + capability + "'";
      if (!blockers.empty()) {
        reason += ": " + JoinReasons(blockers);
      }
    } else {
      reason = "no vehicle with capability '" + capability + "'";
    }
    decision.reject_reason = reason;
    decision.strategy = "rejected";
    return decision;
  }

  // ---- 规则 3：最近优先（距离不可算者排在后面；全不可算则取清单首个） ----
  const VehicleRuntimeState* best = nullptr;
  double best_distance = std::numeric_limits<double>::max();
  for (const auto* vehicle : eligible) {
    const auto it = request.distance_to_pickup_m.find(vehicle->vehicle_id);
    if (it == request.distance_to_pickup_m.end()) {
      continue;  // 距离不可算 —— 排在有距离的候选之后
    }
    if (best == nullptr || it->second < best_distance) {
      best = vehicle;
      best_distance = it->second;
    }
  }

  if (best == nullptr) {
    best = eligible.front();  // 全部距离不可算 -> 清单序首个可用
    decision.strategy = "first_available";
  } else {
    decision.strategy = "nearest";
  }

  decision.assigned = true;
  decision.vehicle_id = best->vehicle_id;
  return decision;
}

}  // namespace amr_dispatcher_core::fleet
