#include "amr_dispatcher_core/fleet/force_clear_link.hpp"

namespace amr_dispatcher_core::fleet {

ForceClearLinkDecision PlanForceClearLink(const ForceClearLinkRequest& request) {
  ForceClearLinkDecision decision;
  if (request.vehicle_id.empty()) {
    decision.reject_reason = "vehicle_id is empty";
    return decision;
  }
  // 防误操作：确认车 id 必须与目标车 id 完全一致
  if (request.confirm_vehicle_id != request.vehicle_id) {
    decision.reject_reason = "confirm_vehicle_id '" + request.confirm_vehicle_id +
                             "' does not match vehicle_id '" + request.vehicle_id + "'";
    return decision;
  }
  // 审计必填
  if (request.operator_id.empty()) {
    decision.reject_reason = "operator_id is required for audit";
    return decision;
  }
  if (request.reason.empty()) {
    decision.reject_reason = "reason is required for audit";
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
  if (request.vehicle_quarantined) {
    decision.reject_reason =
        "vehicle " + request.vehicle_id + " is already quarantined — nothing to force-clear";
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

RestoreLinkDecision PlanRestoreLink(const RestoreLinkRequest& request) {
  RestoreLinkDecision decision;
  if (request.vehicle_id.empty()) {
    decision.reject_reason = "vehicle_id is empty";
    return decision;
  }
  if (request.confirm_vehicle_id != request.vehicle_id) {
    decision.reject_reason = "confirm_vehicle_id '" + request.confirm_vehicle_id +
                             "' does not match vehicle_id '" + request.vehicle_id + "'";
    return decision;
  }
  if (request.operator_id.empty()) {
    decision.reject_reason = "operator_id is required for audit";
    return decision;
  }
  if (request.reason.empty()) {
    decision.reject_reason = "reason is required for audit";
    return decision;
  }
  if (!request.vehicle_in_managed_list) {
    decision.reject_reason = "vehicle " + request.vehicle_id + " not in managed list";
    return decision;
  }
  if (!request.vehicle_quarantined) {
    decision.reject_reason = "vehicle " + request.vehicle_id + " is not quarantined";
    return decision;
  }
  if (!request.vehicle_online) {
    // 离线车恢复后仍不可用：它连状态都报不上，回到调度池只会再次触发挂起
    decision.reject_reason =
        "vehicle " + request.vehicle_id + " is still offline — cannot rejoin the dispatch pool";
    return decision;
  }
  if (!request.position_verified) {
    // 位置未核实 -> 车可能仍占着通道，恢复它进调度池会撞车
    decision.reject_reason =
        "position_verified must be confirmed by the operator before restoring a vehicle";
    return decision;
  }
  decision.allowed = true;
  return decision;
}

}  // namespace amr_dispatcher_core::fleet
