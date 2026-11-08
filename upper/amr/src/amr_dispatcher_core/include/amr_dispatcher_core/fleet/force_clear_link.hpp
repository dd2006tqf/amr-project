#pragma once

#include <string>
#include <unordered_map>
#include <vector>

namespace amr_dispatcher_core::fleet {

// 操作员人工强清离线 VDA 5050 链路的**纯核心决策**（Spec §5.2 人工强解半边）。
//
// 设计约束：核心库零 ROS/零 MQTT 依赖——本模块只吃纯数据、只出纯数据；
// ROS 侧的 service 回调只负责采集上下文 + 落地执行。
//
// 安全铁律（v2 修订）：
//   1. **通信失联 ≠ 可以释放路权**。离线只说明通信异常，不证明车辆已停止，
//      也不证明它没有占着被预约的通道。因此强清是**操作员声明**（人工承担
//      物理安全责任），服务端只校验"操作是否被授权、目标是否一致"。
//   2. 只有"受管清单内"且"当前离线"的车辆可被强清——在线车强清会误杀活车。
//   3. 强清后车辆进入**隔离态**（quarantined），不摘除出受管清单：
//      它仍然可见、可审计，但**不再参与自动分配**（见 PlanVehicleAssignment）。
//      恢复需显式调用 RestoreVda5050Link（并声明已核实车辆位置）。
//   4. 锁归属必须经 mission->vehicle 绑定反查。**严禁**用"路径存档存在"之类的
//      间接证据推断归属——那会把其他车辆 mission 的锁误判为本车名下并误释放。
//   5. 锁归属不可判定时保守不动（宁可少放，不可错放）。
//   6. 审计必填：operator_id + reason + confirm_vehicle_id 三者缺一即拒绝。

// 强清决策的输入快照。
struct ForceClearLinkRequest {
  // 目标车辆 id（受管清单中的 vehicle_id）。
  std::string vehicle_id;
  // 操作员确认的车辆 id：必须与 vehicle_id 完全一致（防误操作打错车）。
  std::string confirm_vehicle_id;
  // 操作员标识（必填，写入审计）。
  std::string operator_id;
  // 操作原因（必填，写入审计）。
  std::string reason;
  // 目标车辆是否在受管清单内。
  bool vehicle_in_managed_list = false;
  // 目标车辆当前链路是否在线（true = 活车，拒绝强清）。
  bool vehicle_online = true;
  // 目标车辆是否已处于隔离态（已隔离则拒绝重复强清）。
  bool vehicle_quarantined = false;
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
  // 必须显式收口释放执行槽位，否则后续任务永远排不上。
  std::vector<std::string> missions_to_abort;
};

// 纯函数：给定强清上下文，产出强清决策。
// 判定顺序（任一不满足即拒绝，且不产生任何副作用）：
//   1. vehicle_id 非空；
//   2. confirm_vehicle_id 与 vehicle_id 完全一致（防打错车）；
//   3. operator_id 与 reason 非空（审计必填）；
//   4. 车辆在受管清单内；
//   5. 车辆当前离线（在线 = 误杀活车，拒绝）；
//   6. 车辆未处于隔离态（已隔离则拒绝重复强清）。
// 受理后：
//   - 收集绑定到该车的 mission（missions_to_abort）；
//   - 对每个绑定 mission，收集 held_locks 中 owner == 该 mission 的锁（locks_by_mission）。
//     绑定缺失的 owner（如 traffic_block: 前缀的封路、其它车辆的任务）一律不动。
ForceClearLinkDecision PlanForceClearLink(const ForceClearLinkRequest& request);

// 恢复（解除隔离）决策的输入快照。
struct RestoreLinkRequest {
  std::string vehicle_id;
  std::string confirm_vehicle_id;
  std::string operator_id;
  std::string reason;
  bool vehicle_in_managed_list = false;
  bool vehicle_quarantined = false;   // 必须处于隔离态才可恢复
  bool vehicle_online = true;         // 在线才可恢复（离线车恢复后仍不可用）
  // 操作员是否已核实车辆物理位置（必填 true）：位置未核实的车不得回到调度池。
  bool position_verified = false;
};

// 恢复决策结果。
struct RestoreLinkDecision {
  bool allowed = false;
  std::string reject_reason;
};

RestoreLinkDecision PlanRestoreLink(const RestoreLinkRequest& request);

}  // namespace amr_dispatcher_core::fleet
