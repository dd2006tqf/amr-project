#include "amr_dispatcher_core/fleet/vehicle_selection_strategy.hpp"

#include <algorithm>
#include <cmath>

namespace amr_dispatcher_core::fleet {

bool ParseVehicleSelectionStrategy(const std::string& label,
                                   VehicleSelectionStrategy* out) {
  if (out == nullptr) {
    return false;
  }
  if (label == "nearest") {
    *out = VehicleSelectionStrategy::kNearest;
    return true;
  }
  if (label == "battery_preferred") {
    *out = VehicleSelectionStrategy::kBatteryPreferred;
    return true;
  }
  return false;  // 未知标签不猜
}

const char* VehicleSelectionStrategyLabel(VehicleSelectionStrategy strategy) {
  switch (strategy) {
    case VehicleSelectionStrategy::kNearest:
      return "nearest";
    case VehicleSelectionStrategy::kBatteryPreferred:
      return "battery_preferred";
  }
  return "nearest";
}

std::vector<ScoredVehicle> RankVehiclesForSelection(
    VehicleSelectionStrategy strategy, const std::vector<VehicleRankEntry>& eligible,
    const std::unordered_map<std::string, double>& distance_to_pickup_m) {
  std::vector<ScoredVehicle> scored;
  scored.reserve(eligible.size());
  for (const auto& entry : eligible) {
    const auto cost_it = distance_to_pickup_m.find(entry.vehicle_id);
    if (cost_it == distance_to_pickup_m.end()) {
      continue;  // 距离不可算：不参与排序（不盲派）
    }
    ScoredVehicle item;
    item.vehicle_id = entry.vehicle_id;
    item.cost_m = cost_it->second;
    item.battery_percent = entry.battery_percent;
    if (strategy == VehicleSelectionStrategy::kBatteryPreferred &&
        entry.battery_percent < kSelectionBatteryReferencePercent) {
      const double shortfall = kSelectionBatteryReferencePercent - entry.battery_percent;
      const double penalty = (shortfall / 100.0) * kSelectionBatteryPenaltyWeight;
      item.score = item.cost_m * (1.0 + penalty);
    } else {
      item.score = item.cost_m;
    }
    scored.push_back(std::move(item));
  }
  // 全序：分数升序，同分按 vehicle_id 字典序（否则容器遍历序会让结果漂）
  std::stable_sort(scored.begin(), scored.end(),
                   [](const ScoredVehicle& a, const ScoredVehicle& b) {
                     if (std::fabs(a.score - b.score) > 1e-9) {
                       return a.score < b.score;
                     }
                     if (a.cost_m != b.cost_m) {
                       return a.cost_m < b.cost_m;
                     }
                     return a.vehicle_id < b.vehicle_id;
                   });
  return scored;
}

}  // namespace amr_dispatcher_core::fleet
