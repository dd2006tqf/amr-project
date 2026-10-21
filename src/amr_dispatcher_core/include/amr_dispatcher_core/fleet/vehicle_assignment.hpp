#pragma once

// 多车车队化：Mission → Vehicle 选车策略的**纯核心决策**。
//
// 设计约束：核心库零 ROS / 零 MQTT 依赖——本模块只吃纯数据、只出纯数据；
// ROS 侧只负责采集上下文（受管车在线态 / 在途占用 / 能力 / 电量 / 位置）与落地执行。
//
// 语义约定（v2 修订版）：
//   * **执行后端分离**：`kLocalSim`（本地模拟车，调度发 /plan/{vid}）与
//     `kVda5050`（外部标准协议车，调度只发 MQTT order，车辆自带控制器）
//     共用任务/选车/路权，但不共用运动控制链。选车阶段不看后端。
//   * **身份语义分离**：`Mission.fleet_id` = 车队归属；本模块产出的
//     `vehicle_id` = 具体承接车辆（对应 fleet.yaml 的 robots[].id / VDA 5050 serial）。
//   * **三态结果**：
//       kAssigned      —— 已选车（全部硬约束满足）
//       kDeferred      —— 暂时不可用（车忙 / 位置未就绪 / 状态未到）→ 任务留队等待
//       kIndeterminate —— 状态不确定（无该车状态报文）→ 暂停自动分配，等状态恢复
//       kRejected      —— 永久不满足（显式指定车不存在 / 禁用 / 能力不符 / 已隔离）
//   * **位置不可用即不合格**：位置无效或状态超龄的车**不参与自动分配**，
//     绝不"退化到清单首台"（那可能把任务派给根本到不了取货点的车）。
//   * **显式指定也受全部硬约束**：点名不豁免在线/忙碌/能力/电量/隔离检查，
//     但**豁免位置有效性**（人工知道车在哪）。

#include <chrono>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace amr_dispatcher_core::fleet {

// 车辆执行后端：决定"谁在跑这条路径"。
enum class VehicleExecutionBackend {
  kLocalSim,  // 本地模拟车：调度发 /plan/{vid}，path_tracker 算 cmd_vel，chassis 执行
  kVda5050,   // 外部 VDA 5050 车：调度只发 MQTT order，车辆自带控制器执行路线
};

// 后端 -> 稳定的字符串标签（配置/消息/日志用）。
const char* VehicleBackendLabel(VehicleExecutionBackend backend);
// 字符串标签 -> 后端；未知标签返回 false（不猜）。
bool ParseVehicleBackend(const std::string& label, VehicleExecutionBackend* out);

// 单台受管车的运行时状态。
struct VehicleRuntimeState {
  std::string vehicle_id;
  VehicleExecutionBackend backend = VehicleExecutionBackend::kLocalSim;

  // —— 静态（来自 fleet.yaml）——
  bool enabled = true;                  // 运营开关
  std::vector<std::string> capabilities;  // 空 = 未声明能力限制（视为无限制）

  // —— 动态（来自最新状态报文）——
  bool online = true;   // 链路在线
  bool busy = false;    // 已被在途任务占用
  double battery_percent = 100.0;
  bool position_valid = false;  // 位置是否可信（来自新鲜定位状态）
  // 车辆当前位置（map 坐标，米）。current_station_id 为空（车不在已知站点上）
  // 时，用它 + 站点目录算"最近站点 -> 取货点"的路网成本。
  double position_x = 0.0;
  double position_y = 0.0;
  std::string current_station_id;
  std::chrono::steady_clock::time_point state_time{};  // 状态采集时刻（新鲜度判定）
  bool quarantined = false;  // 隔离态（人工强清后）——不合格但可恢复
};

// 选车请求。
struct VehicleAssignmentRequest {
  // 显式指定的车 id；可空（空 = 由选车策略决定）。
  std::string requested_vehicle_id;
  // 受管车清单（顺序 = 同分/退化时的裁决顺序）。
  std::vector<VehicleRuntimeState> vehicles;
  std::string pickup_station_id;
  std::string dropoff_station_id;
  // 任务所需能力；空 = 不要求任何能力。默认 "transport"。
  std::string required_capability = "transport";
  // 最低电量阈值（百分比）。默认 0（不过滤）。
  double min_battery_percent = 0.0;
  // 状态最大时延（毫秒）：超过则视为"状态不确定"。默认 5000。
  int64_t state_max_age_ms = 5000;
  // vehicle_id -> 到 pickup 的路径成本（米）。缺项 = 距离不可算。
  std::unordered_map<std::string, double> distance_to_pickup_m;
};

// 选车结果分类。
enum class AssignmentOutcome {
  kAssigned,        // 已选车
  kDeferred,        // 暂时不可用 -> 任务留队等待
  kIndeterminate,   // 状态不确定 -> 暂停自动分配，等待状态恢复
  kRejected,        // 永久不满足 -> 拒绝
};

// 选车决策结果。
struct VehicleAssignmentDecision {
  AssignmentOutcome outcome = AssignmentOutcome::kRejected;
  std::string vehicle_id;
  std::string reject_reason;
  // 决策依据（审计用）："explicit" | "nearest" | "deferred" | "indeterminate" | "rejected"
  std::string strategy;
  // 触发 Deferred/Indeterminate 的车 id（可为空）。
  std::string blocking_vehicle_id;
};

// 该车是否具备某能力。
//   * capability 为空 -> 视为"不要求能力"，恒为 true；
//   * vehicle.capabilities 为空 -> 视为"该车无能力限制"，恒为 true；
//   * 否则按精确匹配。
bool VehicleHasCapability(const VehicleRuntimeState& vehicle, const std::string& capability);

// 状态是否新鲜（age <= state_max_age_ms）。state_time 为默认构造值（从未收到状态）视为**不新鲜**。
bool VehicleStateIsFresh(const VehicleRuntimeState& vehicle, int64_t state_max_age_ms);

// 纯函数：给定选车上下文，产出选车决策。
//
// 判定顺序（固定、可单测）：
//   1. `vehicles` 为空 -> kRejected("no managed vehicles configured")。
//   2. **显式指定**：
//      - 该车不在清单 -> kRejected（不静默改派）
//      - 该车 disabled / 缺能力 / 电量不足 -> kRejected（点名不豁免硬约束）
//      - 该车 offline / busy / quarantined -> **kDeferred**（这些都是**暂时**状态：
//        任务留队等它恢复，而不是判负——"暂时不可用"与"永久不满足"必须分开）
//      - 否则 -> kAssigned, strategy="explicit"
//        （显式指定豁免位置有效性与状态新鲜度：人工知道车在哪）
//   3. **自动选车**：逐车评估，硬约束全部满足才合格：
//        online ∧ enabled ∧ !busy ∧ !quarantined ∧ 具备能力 ∧ 电量达标
//        ∧ 位置有效 ∧ 状态新鲜
//      - 有"从未收到状态"的车（state_time 为默认构造值）-> 整轮 kIndeterminate
//        （状态不确定时暂停自动分配，等状态恢复，而不是乱派）
//      - 有"位置无效/状态超龄"的车 -> 这些车不合格（kDeferred 线索）
//      - 全部车都忙 -> kDeferred（任务留队）
//      - 有合格车但距离都不可算 -> kRejected（宁可不动也不盲派）
//      - 否则按路径成本升序取最近 -> kAssigned, strategy="nearest"
//
// **暂时 vs 永久的分类判据**（评审"失败 vs 排队矛盾"的落点）：
//   暂时（kDeferred，任务留队等待）：offline / busy / quarantined / 状态陈旧 /
//     位置未就绪 —— 这些都会随时间自行恢复。
//   永久（kRejected，任务判负）：车不在清单 / disabled / 缺能力 / 电量低于阈值 ——
//     这些不会自行恢复，需要人或运维改配置。
//   不确定（kIndeterminate，暂停自动分配）：从未收到状态报文。
VehicleAssignmentDecision PlanVehicleAssignment(const VehicleAssignmentRequest& request);

}  // namespace amr_dispatcher_core::fleet
