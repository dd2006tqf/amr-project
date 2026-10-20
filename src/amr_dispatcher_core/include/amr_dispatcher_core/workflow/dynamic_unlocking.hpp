#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace amr_dispatcher_core::workflow {

// Rule 1（Spec §5.3 逐段释放）纯核心计算结果。
struct TrailingUnlockDecision {
  // lastNodeId 命中订单路径且 lastNodeSequenceId 与订单一致（节点 i 的 sequence = i*2）。
  // false 时一律不释放（保守处置：宁可少放，不可错放）。
  bool confirmed = false;
  // 严格位于确认位置身后、且当前确由 owner_mission_id 持有的路由锁 id 列表。
  std::vector<std::string> lock_ids_to_release;
  // 未确认时的处置指引（调度侧据此打 WARN 日志；确认时可为空）。
  std::string message;
};

// Rule 1：根据小车确认的 lastNodeId/lastNodeSequenceId 计算应释放的身后路由锁。
//
// 参数:
//   order_path_node_ids: 本单行驶路径（travel order，如 station_A, rn_1, station_B）
//   confirmed_last_node_id: 小车 state 上报的 lastNodeId
//   confirmed_last_node_sequence_id: 小车 state 上报的 lastNodeSequenceId
//       （VDA 5050 订单节点 sequence = 下标 * 2，边占用两节点间的奇数槽位）
//   owner_mission_id: 锁的归属任务 id（= 订单 order_id = ResourceTable 预留 owner）
//   held_locks: resource_id -> owner 映射（通常由 ResourceTable::Snapshot() 构造）
//
// 足迹规则（保守）：确认位置 = path[k] 时
//   释放：path[0..k-1] 的节点锁 + path[0..k-2] 的边锁（整段越过的部分）
//   保留：path[k] 节点锁（当前所在节点）、(k-1,k) 边锁（刚驶离，车尾可能悬跨）、
//         (k,k+1) 边锁（正在行驶）以及前方全部锁（物理足迹不放）
//   lastNodeId 未命中路径 / sequence 与订单不符 / 空 lastNodeId
//     -> 不确认，释放集为空，message 附处置指引
// 所有权: 仅返回 held_locks 中 owner == owner_mission_id 的锁；
//   他人锁、快照中不存在（未持有）的 id 一律不返回（撞单/幂等安全）。
TrailingUnlockDecision PlanTrailingUnlock(
    const std::vector<std::string>& order_path_node_ids,
    const std::string& confirmed_last_node_id,
    uint32_t confirmed_last_node_sequence_id,
    const std::string& owner_mission_id,
    const std::unordered_map<std::string, std::string>& held_locks);

// Rule 2（Spec §5.2 断链锁定保护）链路守卫决策（纯函数）。
struct DisconnectGuardDecision {
  // 铁律：任何链路状态下守卫都不产生释放动作——锁仅可由 Rule 1 的显式位置推进释放。
  bool retain_all_locks = true;
  bool block_new_dispatch = false;    // 离线：挂起向该车的新派发，直至 online=true
  bool emit_watchdog_warning = false; // 离线或状态陈旧：软看门狗告警
  std::string message;                // 告警文案（含 Spec 指引引用）
};

// Rule 2：小车链路状态 -> 守卫决策。
//   离线（LWT CONNECTIONBROKEN / 心跳超时）：保留全部锁 + 挂起派发 + 告警；
//   在线但 state 陈旧：保留全部锁 + 告警（陈旧本身不是释放依据，也不阻塞派发）；
//   在线且新鲜：保留锁（释放只来自 Rule 1），放行派发，不告警。
DisconnectGuardDecision EvaluateVehicleLinkGuard(bool vehicle_online, bool state_stale);

}  // namespace amr_dispatcher_core::workflow
