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

const char* VehicleBackendLabel(VehicleExecutionBackend backend) {
  switch (backend) {
    case VehicleExecutionBackend::kLocalSim:
      return "local_sim";
    case VehicleExecutionBackend::kVda5050:
      return "vda5050";
  }
  return "local_sim";
}

bool ParseVehicleBackend(const std::string& label, VehicleExecutionBackend* out) {
  if (out == nullptr) {
    return false;
  }
  if (label == "local_sim" || label.empty()) {
    *out = VehicleExecutionBackend::kLocalSim;
    return true;
  }
  if (label == "vda5050") {
    *out = VehicleExecutionBackend::kVda5050;
    return true;
  }
  return false;
}

bool VehicleHasCapability(const VehicleRuntimeState& vehicle, const std::string& capability) {
  if (capability.empty()) {
    return true;
  }
  if (vehicle.capabilities.empty()) {
    return true;  // 未声明能力限制
  }
  return std::find(vehicle.capabilities.begin(), vehicle.capabilities.end(), capability) !=
         vehicle.capabilities.end();
}

bool VehicleStateIsFresh(const VehicleRuntimeState& vehicle, int64_t state_max_age_ms) {
  // 从未收到过状态（state_time 为默认构造值）-> 状态不确定，不新鲜
  if (vehicle.state_time == std::chrono::steady_clock::time_point{}) {
    return false;
  }
  const auto age = std::chrono::duration_cast<std::chrono::milliseconds>(
                       std::chrono::steady_clock::now() - vehicle.state_time)
                       .count();
  return age <= state_max_age_ms;
}

VehicleAssignmentDecision PlanVehicleAssignment(const VehicleAssignmentRequest& request) {
  VehicleAssignmentDecision decision;

  if (request.vehicles.empty()) {
    decision.outcome = AssignmentOutcome::kRejected;
    decision.strategy = "rejected";
    decision.reject_reason = "no managed vehicles configured";
    return decision;
  }

  const std::string& capability = request.required_capability;

  // ---- 规则 2：显式指定车辆（点名不豁免硬约束，但豁免位置/新鲜度）----
  if (!request.requested_vehicle_id.empty()) {
    const auto it = std::find_if(
        request.vehicles.begin(), request.vehicles.end(),
        [&](const VehicleRuntimeState& v) { return v.vehicle_id == request.requested_vehicle_id; });
    if (it == request.vehicles.end()) {
      decision.outcome = AssignmentOutcome::kRejected;
      decision.strategy = "rejected";
      decision.reject_reason =
          "vehicle '" + request.requested_vehicle_id + "' is not in the managed vehicle list";
      return decision;
    }
    const auto& vehicle = *it;
    decision.blocking_vehicle_id = vehicle.vehicle_id;
    // 永久不满足（不会自行恢复，需人或运维改配置）-> 拒绝
    if (!vehicle.enabled) {
      decision.outcome = AssignmentOutcome::kRejected;
      decision.strategy = "rejected";
      decision.reject_reason = "vehicle '" + vehicle.vehicle_id + "' is disabled";
      return decision;
    }
    if (!VehicleHasCapability(vehicle, capability)) {
      decision.outcome = AssignmentOutcome::kRejected;
      decision.strategy = "rejected";
      decision.reject_reason = "vehicle '" + vehicle.vehicle_id + "' lacks required capability '" +
                              capability + "'";
      return decision;
    }
    if (vehicle.battery_percent < request.min_battery_percent) {
      decision.outcome = AssignmentOutcome::kRejected;
      decision.strategy = "rejected";
      decision.reject_reason = "vehicle '" + vehicle.vehicle_id + "' battery below threshold";
      return decision;
    }
    // 暂时不可用（会自行恢复）-> 留队等待，不判负
    if (vehicle.quarantined) {
      decision.outcome = AssignmentOutcome::kDeferred;
      decision.strategy = "deferred";
      decision.reject_reason = "vehicle '" + vehicle.vehicle_id +
                              "' is quarantined — restore it before assigning tasks";
      return decision;
    }
    if (!vehicle.online) {
      decision.outcome = AssignmentOutcome::kDeferred;
      decision.strategy = "deferred";
      decision.reject_reason = "vehicle '" + vehicle.vehicle_id + "' is offline";
      return decision;
    }
    if (vehicle.busy) {
      // 车忙是**暂时**状态：任务留队等待该车空闲，不判负
      decision.outcome = AssignmentOutcome::kDeferred;
      decision.strategy = "deferred";
      decision.reject_reason = "vehicle '" + vehicle.vehicle_id + "' is busy";
      return decision;
    }
    decision.outcome = AssignmentOutcome::kAssigned;
    decision.strategy = "explicit";
    decision.vehicle_id = vehicle.vehicle_id;
    return decision;
  }

  // ---- 规则 3：自动选车 ----
  std::vector<const VehicleRuntimeState*> eligible;
  eligible.reserve(request.vehicles.size());
  bool saw_never_seen = false;   // 从未收到状态的车
  bool saw_stale = false;        // 状态超龄的车
  bool saw_busy = false;
  bool saw_offline = false;      // 链路离线（暂时）
  bool saw_quarantined = false;  // 隔离态（暂时，待人工恢复）
  bool saw_no_position = false;
  bool saw_capable = false;
  std::vector<std::string> blockers;

  for (const auto& vehicle : request.vehicles) {
    if (!VehicleHasCapability(vehicle, capability)) {
      continue;
    }
    saw_capable = true;
    if (!vehicle.enabled) {
      blockers.push_back(vehicle.vehicle_id + ":disabled");
      continue;
    }
    if (vehicle.quarantined) {
      saw_quarantined = true;
      blockers.push_back(vehicle.vehicle_id + ":quarantined");
      continue;
    }
    if (!vehicle.online) {
      saw_offline = true;
      blockers.push_back(vehicle.vehicle_id + ":offline");
      continue;
    }
    if (vehicle.busy) {
      saw_busy = true;
      blockers.push_back(vehicle.vehicle_id + ":busy");
      continue;
    }
    if (vehicle.battery_percent < request.min_battery_percent) {
      blockers.push_back(vehicle.vehicle_id + ":battery");
      continue;
    }
    if (vehicle.state_time == std::chrono::steady_clock::time_point{}) {
      saw_never_seen = true;
      blockers.push_back(vehicle.vehicle_id + ":no_state");
      continue;
    }
    if (!VehicleStateIsFresh(vehicle, request.state_max_age_ms)) {
      saw_stale = true;
      blockers.push_back(vehicle.vehicle_id + ":stale_state");
      continue;
    }
    if (!vehicle.position_valid) {
      saw_no_position = true;
      blockers.push_back(vehicle.vehicle_id + ":no_position");
      continue;
    }
    eligible.push_back(&vehicle);
  }

  if (eligible.empty()) {
    decision.strategy = "rejected";
    if (saw_never_seen) {
      // 从未收到状态 -> 状态不确定：暂停自动分配，等状态恢复，而不是乱派
      decision.outcome = AssignmentOutcome::kIndeterminate;
      decision.strategy = "indeterminate";
      decision.reject_reason = "vehicle state unavailable (no state report yet)";
    } else if (saw_busy) {
      // 车忙 -> 暂时不可用：任务留队等待
      decision.outcome = AssignmentOutcome::kDeferred;
      decision.strategy = "deferred";
      decision.reject_reason = "all eligible vehicles are busy";
    } else if (saw_offline) {
      // 全部车离线 -> 暂时不可用：任务留队等待链路恢复（不是永久失败）
      decision.outcome = AssignmentOutcome::kDeferred;
      decision.strategy = "deferred";
      decision.reject_reason = "all eligible vehicles are offline";
    } else if (saw_quarantined) {
      // 全部车被隔离 -> 暂时不可用：等操作员恢复
      decision.outcome = AssignmentOutcome::kDeferred;
      decision.strategy = "deferred";
      decision.reject_reason = "all eligible vehicles are quarantined";
    } else if (saw_stale) {
      decision.outcome = AssignmentOutcome::kDeferred;
      decision.strategy = "deferred";
      decision.reject_reason = "vehicle state is stale";
    } else if (saw_no_position) {
      decision.outcome = AssignmentOutcome::kDeferred;
      decision.strategy = "deferred";
      decision.reject_reason = "no vehicle has a valid position";
    } else if (!saw_capable) {
      decision.outcome = AssignmentOutcome::kRejected;
      decision.reject_reason = "no vehicle with capability '" + capability + "'";
    } else {
      decision.outcome = AssignmentOutcome::kRejected;
      decision.reject_reason = "no eligible vehicle with capability '" + capability + "'";
    }
    if (!blockers.empty()) {
      decision.reject_reason += ": " + JoinReasons(blockers);
    }
    return decision;
  }

  // ---- 合格候选：按路径成本升序（距离不可算者排在后面）----
  const VehicleRuntimeState* best = nullptr;
  double best_cost = std::numeric_limits<double>::max();
  for (const auto* vehicle : eligible) {
    const auto it = request.distance_to_pickup_m.find(vehicle->vehicle_id);
    if (it == request.distance_to_pickup_m.end()) {
      continue;
    }
    if (best == nullptr || it->second < best_cost) {
      best = vehicle;
      best_cost = it->second;
    }
  }
  if (best == nullptr) {
    // 全部合格车距离都不可算 -> 不盲派（宁可不动也不把任务派给到不了取货点的车）
    decision.outcome = AssignmentOutcome::kRejected;
    decision.strategy = "rejected";
    decision.reject_reason = "no reachable pickup cost for any eligible vehicle";
    return decision;
  }

  decision.outcome = AssignmentOutcome::kAssigned;
  decision.strategy = "nearest";
  decision.vehicle_id = best->vehicle_id;
  return decision;
}

}  // namespace amr_dispatcher_core::fleet
