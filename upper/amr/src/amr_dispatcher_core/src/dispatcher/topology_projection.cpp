#include "amr_dispatcher_core/dispatcher/topology_projection.hpp"

#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace amr_dispatcher_core::dispatcher {
namespace {

bool StartsWith(const std::string& value, const std::string& prefix) {
  return value.rfind(prefix, 0) == 0;
}

// 从锁 id 解析出站点 id 并去重追加（保序）：
//   route_edge:A__B -> A、B；route_node:X -> X；其余（mission id、畸形 id）跳过。
// 规则与 workflow::ParseRouteLock 等价，但本文件不 include workflow 层，保持 dispatcher 域内聚。
void AppendStationIds(const std::string& lock_id, std::vector<std::string>& out,
                      std::unordered_set<std::string>& seen) {
  auto append = [&out, &seen](const std::string& station_id) {
    if (station_id.empty()) {
      return;
    }
    if (seen.insert(station_id).second) {
      out.push_back(station_id);
    }
  };

  const std::string edge_prefix = "route_edge:";
  if (StartsWith(lock_id, edge_prefix)) {
    const auto separator = lock_id.find("__", edge_prefix.size());
    if (separator == std::string::npos) {
      return;
    }
    const auto first = lock_id.substr(edge_prefix.size(), separator - edge_prefix.size());
    const auto second = lock_id.substr(separator + 2);
    if (first.empty() || second.empty()) {
      return;
    }
    append(first);
    append(second);
    return;
  }
  const std::string node_prefix = "route_node:";
  if (StartsWith(lock_id, node_prefix)) {
    append(lock_id.substr(node_prefix.size()));
  }
}

}  // namespace

TopologyProjection BuildTopologyProjection(
    const std::vector<std::pair<std::string, std::string>>& resource_owner_snapshot,
    const std::unordered_set<std::string>& active_mission_ids,
    const std::vector<std::vector<std::string>>& deadlock_cycle_paths) {
  TopologyProjection projection;

  // locked_edges / edge_owners：Snapshot 全量透传（Snapshot 已排序，直接按序）
  for (const auto& [resource_id, owner_id] : resource_owner_snapshot) {
    projection.locked_edges.push_back(resource_id);
    projection.edge_owners.push_back(owner_id);
    // active_edges：仅任务持有且带 route_edge: 前缀的资源
    if (active_mission_ids.count(owner_id) != 0 && StartsWith(resource_id, "route_edge:")) {
      projection.active_edges.push_back(resource_id);
    }
  }

  // deadlocked_nodes：遍历环路径解析站点 id（去重有序），mission id 不得混入
  std::unordered_set<std::string> seen;
  for (const auto& cycle_path : deadlock_cycle_paths) {
    if (!cycle_path.empty()) {
      projection.deadlock_detected = true;
    }
    for (const auto& item : cycle_path) {
      AppendStationIds(item, projection.deadlocked_nodes, seen);
    }
  }

  return projection;
}

}  // namespace amr_dispatcher_core::dispatcher
