#pragma once

// A1：Mission → Vehicle 选车策略的**纯核心决策**。
//
// 设计约束：核心库零 ROS / 零 MQTT 依赖——本模块只吃纯数据、只出纯数据；
// ROS 侧只负责采集上下文（受管车在线态 / 在途占用 / 能力 / 电量）与落地执行。
//
// 语义约定：
//   * `capabilities` 为空 = 该车**未声明能力限制**（视为"无限制"），
//     这样单车最小配置（只配了一台车、没写 capabilities）仍可被选中；
//     非空时按精确匹配判定。
//   * `requested_vehicle_id` 非空 = 操作员/上游**显式指定**，此时
//     **绝不静默改派**——指定车不满足条件就直接拒绝，由上层决定重试或改单。

#include <string>
#include <unordered_map>
#include <vector>

namespace amr_dispatcher_core::fleet {

// 单台受管车的运行时状态（轻量输入，不与 fleet.yaml 强耦合）。
struct VehicleRuntimeState {
  std::string vehicle_id;
  bool online = true;   // 链路在线（VDA 5050 online 态）
  bool busy = false;    // 已被在途任务占用
  bool enabled = true;  // 运营开关
  double battery_percent = 100.0;
  std::string current_station_id;       // 已知则用于距离估算，可空
  std::vector<std::string> capabilities;  // 如 {"transport","charging"}；空 = 无限制
};

// 选车请求。
struct VehicleAssignmentRequest {
  // 显式指定的车 id；可空（空 = 由选车策略决定）。
  std::string requested_vehicle_id;
  // 受管车清单（顺序即"同距/无距"时的裁决顺序）。
  std::vector<VehicleRuntimeState> vehicles;
  std::string pickup_station_id;
  std::string dropoff_station_id;
  // 任务所需能力；空 = 不要求任何能力。默认 "transport"。
  std::string required_capability = "transport";
  // 最低电量阈值（百分比）。默认 0（不过滤）。
  double min_battery_percent = 0.0;
  // vehicle_id -> 到 pickup 的直线距离（米）；缺项表示该车距离不可算。
  std::unordered_map<std::string, double> distance_to_pickup_m;
};

// 选车决策结果。
struct VehicleAssignmentDecision {
  bool assigned = false;
  std::string vehicle_id;
  std::string reject_reason;
  // 决策依据（审计用）："explicit" | "nearest" | "first_available" | "rejected"
  std::string strategy;
};

// 该车是否具备某能力。
//   * capability 为空 → 视为"不要求能力"，恒为 true；
//   * vehicle.capabilities 为空 → 视为"该车无能力限制"，恒为 true；
//   * 否则按精确匹配。
bool VehicleHasCapability(const VehicleRuntimeState& vehicle, const std::string& capability);

// 纯函数：给定选车上下文，产出选车决策。
//
// 判定顺序（固定、可单测）：
//   1. `requested_vehicle_id` 非空（显式指定）
//      → 该车必须存在于 vehicles，且 online ∧ enabled ∧ !busy ∧ 具备 required_capability；
//        任一不满足即**拒绝**（不静默改派）。命中则 strategy = "explicit"。
//        注意：显式指定不施加电量阈值——操作员点名即生效。
//   2. 未显式指定 → 过滤候选：online ∧ enabled ∧ !busy ∧ 具备 required_capability
//      ∧ battery_percent >= min_battery_percent。
//   3. 候选按"到 pickup 的距离"升序取最近；同距按清单输入序；
//      距离不可算的车排在所有"距离可算"的车之后；
//      若**全部**候选距离都不可算 → 取清单中首个可用，strategy = "first_available"；
//      否则 strategy = "nearest"。
//   4. 无可用候选 → 拒绝，reject_reason 说明主因（忙 / 电量 / 离线 / 能力）。
//   5. vehicles 为空 → 拒绝 "no managed vehicles configured"。
VehicleAssignmentDecision PlanVehicleAssignment(const VehicleAssignmentRequest& request);

}  // namespace amr_dispatcher_core::fleet
