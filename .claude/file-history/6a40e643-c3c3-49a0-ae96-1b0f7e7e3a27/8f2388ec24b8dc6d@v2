#pragma once

#include <string>
#include <unordered_map>
#include <vector>

namespace amr_dispatcher_core::fleet {

// 操作员人工强清离线 VDA 5050 链路的**纯核心决策**（Spec §5.2 人工强解半边）。
//
// 设计约束：核心库零 ROS/零 MQTT 依赖——本模块只吃纯数据、只出纯数据；
// ROS 侧的 service 回调只负责采集上下文 + 落地执行（解绑/释锁/收口任务）。
//
// 安全铁律：
//   1. 只有"受管清单内"且"当前离线"的车辆可被强清——在线车强清会误杀活车。
//   2. 锁归属必须经 mission->vehicle 绑定反查。**严禁**用"路径存档存在"之类的
//      间接证据推断归属——那会把其他车辆 mission 的锁误判为本车名下并误释放。
//   3. 锁归属不可判定时保守不动（宁可少放，不可错放）。

// 强清决策的输入快照。
struct ForceClearLinkRequest {
  // 目标车辆 id（受管清单中的 serialNumber）。
  std::string vehicle_id;
  // 目标车辆是否在受管清单内。
  bool vehicle_in_managed_list = false;
  // 目标车辆当前链路是否在线（true = 活车，拒绝强清）。
  bool vehicle_online = true;
  // mission_id -> 承接该单的车辆 id（下发订单时登记的绑定）。
  // 缺项表示该 mission 未由受管车辆承接（锁不归本车）。
  std::unordered_map<std::string, std::string> mission_vehicle_bindings;
  // resource_id -> 锁 owner（= mission_id）。由 ResourceTable::Snapshot() 采集。
  std::unordered_map<std::string, std::string> held_locks;
};

// 强清决策结果。
struct ForceClearLinkDecision {
  // 是否受理本次强清。
  bool allowed = false;
  // 拒绝原因（allowed == false 时有效）。
  std::string reject_reason;
  // 归属该车名下、需释放的锁（按 mission 分组：mission_id -> resource_id 列表）。
  // ReleaseResources(mission_id, ids) 按组释放，owner 校验在核心内完成。
  std::unordered_map<std::string, std::vector<std::string>> locks_by_mission;
  // 需收口（作废）的在途 mission id——车辆已死，这些任务永远不会到达，
  // 必须显式收口释放单车执行槽位，否则后续任务永远排不上。
  std::vector<std::string> missions_to_abort;
};

// 纯函数：给定强清上下文，产出强清决策。
// 判定顺序（任一不满足即拒绝，且不产生任何副作用）：
//   1. vehicle_id 非空；
//   2. 车辆在受管清单内；
//   3. 车辆当前离线（在线 = 误杀活车，拒绝）。
// 受理后：
//   - 收集绑定到该车的 mission（missions_to_abort）；
//   - 对每个绑定 mission，收集 held_locks 中 owner == 该 mission 的锁（locks_by_mission）。
//     绑定缺失的 owner（如 traffic_block: 前缀的封路、其它车辆的任务）一律不动。
ForceClearLinkDecision PlanForceClearLink(const ForceClearLinkRequest& request);

}  // namespace amr_dispatcher_core::fleet
