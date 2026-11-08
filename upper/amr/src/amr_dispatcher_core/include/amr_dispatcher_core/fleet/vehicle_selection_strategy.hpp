// 选车策略（可插拔，纯函数）。
//
// 原来"自动选车"只有一条规则：取到取货点路网成本最小的车（`nearest`）。
// 规则本身没问题，但它是写死的——运维想"电量低的先歇着、别总派给它"时
// 唯一能调的是 `min_battery_percent` 硬阈值，低于阈值直接不合格（判负风险），
// 而不是"软性降权"。
//
// 这里把"打分"独立出来，策略由参数 `vehicle_selection_strategy` 选择：
//   * `nearest`           —— 只看路网成本（默认，逐字保持改动前行为）
//   * `battery_preferred` —— 路网成本 + 低电量软惩罚
//
// 设计要点：
//   1. **硬约束不在这里**（在线/隔离/忙/能力/电量阈值/位置/新鲜度由
//      PlanVehicleAssignment 负责）——进来的一定是已经合格的候选。
//   2. **同分裁决必须有全序**：分数相同按 vehicle_id 字典序，否则
//      unordered_map 的遍历序会让"每次跑出来不一样"。
//   3. 惩罚上限 25%：足够在"两车距离接近"时偏向电量高的，不足以让
//      明显更近的车输给远处满电车（避免为了省电牺牲整体吞吐）。
#pragma once

#include <string>
#include <unordered_map>
#include <vector>

namespace amr_dispatcher_core::fleet {

enum class VehicleSelectionStrategy {
  kNearest,
  kBatteryPreferred,
};

// 参数标签 <-> 枚举。未知标签返回 false（不猜，调用方回落到默认策略）。
bool ParseVehicleSelectionStrategy(const std::string& label, VehicleSelectionStrategy* out);
const char* VehicleSelectionStrategyLabel(VehicleSelectionStrategy strategy);

struct ScoredVehicle {
  std::string vehicle_id;
  double cost_m = 0.0;         // 到取货点的路网成本（米）
  double battery_percent = 0.0;
  double score = 0.0;          // 越小越优先
};

// 低电量软惩罚的参考电量与权重：
//   penalty = max(0, reference - battery) / 100 * weight
//   score   = cost_m * (1 + penalty)
constexpr double kSelectionBatteryReferencePercent = 50.0;
constexpr double kSelectionBatteryPenaltyWeight = 0.25;

struct VehicleRankEntry {
  std::string vehicle_id;
  double cost_m = 0.0;
  double battery_percent = 0.0;
};

// 对**已经合格**的候选打分并排序（最优在前，同分按 id 字典序）。
// 成本缺失（`distance_to_pickup_m` 里没有）的候选会被丢弃——不盲派。
std::vector<ScoredVehicle> RankVehiclesForSelection(
    VehicleSelectionStrategy strategy, const std::vector<VehicleRankEntry>& eligible,
    const std::unordered_map<std::string, double>& distance_to_pickup_m);

}  // namespace amr_dispatcher_core::fleet
