#include "amr_dispatcher_core/fleet/force_clear_link.hpp"

namespace amr_dispatcher_core::fleet {

ForceClearLinkDecision PlanForceClearLink(const ForceClearLinkRequest& request) {
  ForceClearLinkDecision decision;
  if (request.vehicle_id.empty()) {
    decision.reject_reason = "vehicle_id is empty";
    return decision;
  }
  if (!request.vehicle_in_managed_list) {
    decision.reject_reason = "vehicle " + request.vehicle_id + " not in managed list";
    return decision;
  }
  if (request.vehicle_online) {
    // 活车强清 = 误杀在线链路，物理车仍可能在跑，会撞车。
    decision.reject_reason =
        "vehicle " + request.vehicle_id + " is ONLINE — refusing to force-clear a live link";
    return decision;
  }

  // 收集该车承接的 mission（锁归属的唯一依据）。
  for (const auto& [mission_id, vehicle] : request.mission_vehicle_bindings) {
    if (vehicle == request.vehicle_id) {
      decision.missions_to_abort.push_back(mission_id);
    }
  }

  // 按 mission 归集该车名下的锁；owner 不在绑定表中的锁一律不动。
  for (const auto& [resource_id, owner] : request.held_locks) {
    const auto bind_it = request.mission_vehicle_bindings.find(owner);
    if (bind_it == request.mission_vehicle_bindings.end() ||
        bind_it->second != request.vehicle_id) {
      continue;
    }
    decision.locks_by_mission[owner].push_back(resource_id);
  }

  decision.allowed = true;
  return decision;
}

}  // namespace amr_dispatcher_core::fleet
