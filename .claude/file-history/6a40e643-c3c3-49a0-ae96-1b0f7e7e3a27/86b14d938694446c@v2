#include "amr_dispatcher_core/workflow/order_batch_bundling.hpp"

#include <algorithm>
#include <map>
#include <unordered_set>

namespace amr_dispatcher_core::workflow {

OrderBatchBundleResult PlanOrderBatchBundling(const OrderBatchBundleRequest& request) {
  OrderBatchBundleResult result;
  if (request.batch_id.empty()) {
    result.success = false;
    result.message = "batch_id cannot be empty";
    return result;
  }
  if (request.items.empty()) {
    result.message = "no items to bundle";
    return result;
  }

  // 1. 数据校验与过滤
  for (const auto& item : request.items) {
    if (item.order_id.empty()) {
      result.success = false;
      result.message = "order_id cannot be empty";
      return result;
    }
    if (item.pickup_station.empty() || item.dropoff_station.empty()) {
      result.success = false;
      result.message = "pickup and dropoff stations cannot be empty for order " + item.order_id;
      return result;
    }
    if (item.pickup_station == item.dropoff_station) {
      result.success = false;
      result.message = "pickup and dropoff stations cannot be identical for order " + item.order_id;
      return result;
    }
  }

  // 2. 第一阶段：同站起止归并 (Exact Match Aggregation)
  // key: "pickup->dropoff"
  std::map<std::pair<std::string, std::string>, std::vector<OrderBatchItem>> exact_groups;
  for (const auto& item : request.items) {
    exact_groups[{item.pickup_station, item.dropoff_station}].push_back(item);
  }

  std::vector<BundledOrderGroup> initial_groups;
  int group_idx = 0;
  for (auto& [pair, items] : exact_groups) {
    BundledOrderGroup group;
    group.group_id = request.batch_id + "_grp_" + std::to_string(group_idx++);
    int max_prio = items[0].priority;
    int64_t earliest_deadline = items[0].deadline_unix_ms;
    std::vector<std::string> order_ids;

    for (const auto& item : items) {
      order_ids.push_back(item.order_id);
      if (item.priority > max_prio) {
        max_prio = item.priority;
      }
      if (earliest_deadline == 0 || (item.deadline_unix_ms > 0 && item.deadline_unix_ms < earliest_deadline)) {
        earliest_deadline = item.deadline_unix_ms;
      }
    }

    group.priority = max_prio;
    group.earliest_deadline_unix_ms = earliest_deadline;
    group.order_ids = order_ids;
    group.station_sequence = {pair.first, pair.second};

    BundledOrderLeg leg;
    leg.pickup_station = pair.first;
    leg.dropoff_station = pair.second;
    leg.bundled_order_ids = order_ids;
    group.legs.push_back(leg);

    initial_groups.push_back(std::move(group));
  }

  // 3. 第二阶段：路线串联拼单 (Route Chaining: e.g. A->B and B->C into A->B->C)
  if (!request.enable_route_chaining || initial_groups.size() <= 1) {
    result.groups = std::move(initial_groups);
    result.message = "bundled into " + std::to_string(result.groups.size()) + " group(s)";
    return result;
  }

  // 贪心串联：寻找后一 group 的首站与前一 group 的末站相同的 group 进行合并
  std::vector<bool> consumed(initial_groups.size(), false);
  std::vector<BundledOrderGroup> chained_groups;

  for (size_t i = 0; i < initial_groups.size(); ++i) {
    if (consumed[i]) continue;
    consumed[i] = true;
    BundledOrderGroup chained = initial_groups[i];

    bool extended = true;
    while (extended) {
      extended = false;
      const std::string& current_last_station = chained.station_sequence.back();

      for (size_t j = 0; j < initial_groups.size(); ++j) {
        if (consumed[j]) continue;
        const auto& next_cand = initial_groups[j];
        if (next_cand.station_sequence.front() == current_last_station) {
          // 检查环路（避免无休止循环 A->B->A）
          if (std::find(chained.station_sequence.begin(), chained.station_sequence.end(),
                        next_cand.station_sequence.back()) != chained.station_sequence.end()) {
            continue;
          }
          // 串联合并
          consumed[j] = true;
          chained.priority = std::max(chained.priority, next_cand.priority);
          if (chained.earliest_deadline_unix_ms == 0 ||
              (next_cand.earliest_deadline_unix_ms > 0 &&
               next_cand.earliest_deadline_unix_ms < chained.earliest_deadline_unix_ms)) {
            chained.earliest_deadline_unix_ms = next_cand.earliest_deadline_unix_ms;
          }
          chained.order_ids.insert(chained.order_ids.end(), next_cand.order_ids.begin(),
                                   next_cand.order_ids.end());
          chained.legs.insert(chained.legs.end(), next_cand.legs.begin(), next_cand.legs.end());
          chained.station_sequence.push_back(next_cand.station_sequence.back());
          extended = true;
          break;
        }
      }
    }
    chained_groups.push_back(std::move(chained));
  }

  result.groups = std::move(chained_groups);
  result.message = "bundled into " + std::to_string(result.groups.size()) + " group(s)";
  return result;
}

}  // namespace amr_dispatcher_core::workflow
