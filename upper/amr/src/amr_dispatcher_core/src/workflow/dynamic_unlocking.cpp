#include "amr_dispatcher_core/workflow/dynamic_unlocking.hpp"

#include <optional>
#include <string>

#include "amr_dispatcher_core/dispatcher/traffic_reservation.hpp"

namespace amr_dispatcher_core::workflow {
namespace {

using dispatcher::ResourceTable;

// 在订单路径中定位确认节点：命中且 sequence 与订单一致才返回下标。
// 节点 i 的 sequence = i*2（与 Vda5050OrderBuilder 的偶数序列一致）。
std::optional<std::size_t> LocateConfirmedNode(const std::vector<std::string>& order_path_node_ids,
                                               const std::string& confirmed_last_node_id,
                                               uint32_t confirmed_last_node_sequence_id) {
  if (confirmed_last_node_id.empty()) {
    return std::nullopt;
  }
  for (std::size_t i = 0; i < order_path_node_ids.size(); ++i) {
    if (order_path_node_ids[i] != confirmed_last_node_id) {
      continue;
    }
    if (confirmed_last_node_sequence_id != static_cast<uint32_t>(i * 2)) {
      return std::nullopt;  // 命中但 sequence 与订单不一致：保守不确认
    }
    return i;
  }
  return std::nullopt;  // 不在路径上（订单更新后旧 state / 未知节点）
}

// 只把「理论释放集 ∩ 实际持有 ∩ owner 归属本任务」的锁纳入释放列表。
// 保证：他人锁不放、未持有不放（幂等）、traffic_block 不可见。
void PushIfOwned(const std::string& lock_id, const std::string& owner_mission_id,
                 const std::unordered_map<std::string, std::string>& held_locks,
                 std::vector<std::string>& out) {
  const auto it = held_locks.find(lock_id);
  if (it != held_locks.end() && it->second == owner_mission_id) {
    out.push_back(lock_id);
  }
}

}  // namespace

TrailingUnlockDecision PlanTrailingUnlock(
    const std::vector<std::string>& order_path_node_ids,
    const std::string& confirmed_last_node_id,
    const uint32_t confirmed_last_node_sequence_id,
    const std::string& owner_mission_id,
    const std::unordered_map<std::string, std::string>& held_locks) {
  TrailingUnlockDecision decision;

  const auto index = LocateConfirmedNode(order_path_node_ids, confirmed_last_node_id,
                                         confirmed_last_node_sequence_id);
  if (!index.has_value()) {
    decision.message =
        "dynamic_unlock skipped: lastNodeId '" + confirmed_last_node_id +
        "' (sequence " + std::to_string(confirmed_last_node_sequence_id) +
        ") does not match any node/sequence of the dispatched order; retain ALL locks. "
        "Verify the vehicle's reported orderId/lastNodeId against the order, or wait for "
        "a fresh state report after the vehicle reconfirms its position.";
    return decision;
  }

  const std::size_t k = *index;
  decision.confirmed = true;

  // 节点锁：释放 path[0..k-1]（车已离开的节点）；path[k] 为当前足迹，保留。
  for (std::size_t i = 0; i < k; ++i) {
    PushIfOwned(ResourceTable::RouteNodeId(order_path_node_ids[i]), owner_mission_id,
                held_locks, decision.lock_ids_to_release);
  }
  // 边锁：释放 path[0..k-2]（(i,i+1) 两端均已越过）；
  // (k-1,k) 为刚驶离的进入边（车尾可能仍悬跨）、(k,k+1) 为正在行驶的边，均保留。
  for (std::size_t i = 0; i + 1 < k && i + 1 < order_path_node_ids.size(); ++i) {
    PushIfOwned(ResourceTable::RouteEdgeId(order_path_node_ids[i], order_path_node_ids[i + 1]),
                owner_mission_id, held_locks, decision.lock_ids_to_release);
  }

  return decision;
}

DisconnectGuardDecision EvaluateVehicleLinkGuard(const bool vehicle_online,
                                                 const bool state_stale) {
  DisconnectGuardDecision guard;
  // retain_all_locks 恒为 true（初始化值即铁律）：锁只由 Rule 1 的显式位置推进释放，
  // 链路状态（离线/陈旧/在线）绝不产生释放动作——Spec §5.2 断链锁定保护。
  if (!vehicle_online) {
    guard.block_new_dispatch = true;
    guard.emit_watchdog_warning = true;
    guard.message =
        "vehicle link broken (LWT CONNECTIONBROKEN or heartbeat timeout): retaining ALL "
        "route locks held by this vehicle. Per Spec 5.2, locks are kept until the vehicle "
        "reconnects and its position is re-verified, or an operator manually forces "
        "release; dispatch to this vehicle is suspended while online=false.";
    return guard;
  }
  if (state_stale) {
    guard.emit_watchdog_warning = true;
    guard.message =
        "vehicle online but state report stale: retaining ALL locks (no release without a "
        "fresh, order-consistent lastNodeId per Spec 5.3); dispatch is not blocked by "
        "staleness alone.";
    return guard;
  }
  guard.message = "vehicle online with fresh state; locks released only via explicit "
                  "lastNodeId advancement (Spec 5.3).";
  return guard;
}

}  // namespace amr_dispatcher_core::workflow
