#include "amr_dispatcher_core/catalog/topology_connectivity.hpp"

#include <algorithm>
#include <functional>
#include <unordered_map>
#include <unordered_set>

namespace amr_dispatcher_core::catalog {
namespace {

// 迭代式 Tarjan：递归在大型骨架图上会爆栈（几千节点 × 深链），
// 而拓扑校验是在服务回调里跑的，栈溢出等于整节点崩溃。
class StrongComponents {
 public:
  explicit StrongComponents(const std::vector<std::vector<int>>& adjacency)
      : adjacency_(adjacency), index_(adjacency.size(), -1), low_(adjacency.size(), 0),
        on_stack_(adjacency.size(), false), component_of_(adjacency.size(), -1) {}

  // 返回每个节点所属分量 id（同一 SCC 的节点 id 相同）
  std::vector<int> Run() {
    for (std::size_t node = 0; node < adjacency_.size(); ++node) {
      if (index_[node] >= 0) {
        continue;
      }
      VisitFrom(static_cast<int>(node));
    }
    return component_of_;
  }

 private:
  struct Frame {
    int node = 0;
    std::size_t next_edge = 0;
  };

  void VisitFrom(int start) {
    std::vector<Frame> stack;
    stack.push_back(Frame{start, 0});
    index_[start] = low_[start] = next_index_++;
    scc_stack_.push_back(start);
    on_stack_[start] = true;

    while (!stack.empty()) {
      Frame& frame = stack.back();
      const auto& neighbours = adjacency_[frame.node];
      if (frame.next_edge < neighbours.size()) {
        const int next = neighbours[frame.next_edge++];
        if (index_[next] < 0) {
          index_[next] = low_[next] = next_index_++;
          scc_stack_.push_back(next);
          on_stack_[next] = true;
          stack.push_back(Frame{next, 0});
        } else if (on_stack_[next]) {
          low_[frame.node] = std::min(low_[frame.node], index_[next]);
        }
        continue;
      }
      // 该节点的所有邻居处理完毕：结算
      const int node = frame.node;
      stack.pop_back();
      if (!stack.empty()) {
        low_[stack.back().node] = std::min(low_[stack.back().node], low_[node]);
      }
      if (low_[node] == index_[node]) {
        while (!scc_stack_.empty()) {
          const int member = scc_stack_.back();
          scc_stack_.pop_back();
          on_stack_[member] = false;
          component_of_[member] = next_component_;
          if (member == node) {
            break;
          }
        }
        ++next_component_;
      }
    }
  }

  const std::vector<std::vector<int>>& adjacency_;
  std::vector<int> index_;
  std::vector<int> low_;
  std::vector<bool> on_stack_;
  std::vector<int> scc_stack_;
  std::vector<int> component_of_;
  int next_index_ = 0;
  int next_component_ = 0;
};

}  // namespace

ReachabilityReport AnalyzeTopologyConnectivity(const StationCatalog& catalog) {
  ReachabilityReport report;

  // 节点顺序：站点在前（分量结果按站点命名更稳定），路网节点在后
  std::vector<std::string> ids;
  std::unordered_set<std::string> station_ids;
  ids.reserve(catalog.stations.size() + catalog.road_nodes.size());
  for (const auto& station : catalog.stations) {
    ids.push_back(station.id);
    station_ids.insert(station.id);
  }
  for (const auto& node : catalog.road_nodes) {
    ids.push_back(node.id);
  }

  if (catalog.stations.empty()) {
    report.all_stations_reachable = false;
    report.message = "catalog has no stations";
    return report;
  }

  std::unordered_map<std::string, int> index_of;
  index_of.reserve(ids.size());
  for (std::size_t i = 0; i < ids.size(); ++i) {
    index_of[ids[i]] = static_cast<int>(i);
  }

  std::vector<std::vector<int>> adjacency(ids.size());
  for (const auto& edge : catalog.edges) {
    if (!edge.enabled) {
      continue;  // 被封的边不提供可达性（这正是叠加层生效后的形态）
    }
    const auto from = index_of.find(edge.from);
    const auto to = index_of.find(edge.to);
    if (from == index_of.end() || to == index_of.end()) {
      continue;
    }
    adjacency[from->second].push_back(to->second);
    if (edge.bidirectional) {
      adjacency[to->second].push_back(from->second);
    }
  }

  StrongComponents scc(adjacency);
  const std::vector<int> component_of = scc.Run();

  // 分量 -> 成员
  std::unordered_map<int, std::vector<std::string>> members;
  for (std::size_t i = 0; i < ids.size(); ++i) {
    members[component_of[i]].push_back(ids[i]);
  }
  for (auto& [_, ids_in_component] : members) {
    std::sort(ids_in_component.begin(), ids_in_component.end());
  }

  // "主分量" = 含站点最多的那个分量；若站点被拆到多个分量，多数站点仍在主分量里，
  // 少数成为孤岛 —— 这正是要拒绝的形态。
  int main_component = -1;
  std::size_t best_station_count = 0;
  for (const auto& [component, ids_in_component] : members) {
    std::size_t station_count = 0;
    for (const auto& id : ids_in_component) {
      if (station_ids.count(id) > 0) {
        ++station_count;
      }
    }
    if (station_count > best_station_count) {
      best_station_count = station_count;
      main_component = component;
    }
  }

  for (const auto& station : catalog.stations) {
    if (component_of[index_of[station.id]] != main_component) {
      report.unreachable_stations.push_back(station.id);
    }
  }
  std::sort(report.unreachable_stations.begin(), report.unreachable_stations.end());

  for (const auto& [component, ids_in_component] : members) {
    bool has_station = false;
    for (const auto& id : ids_in_component) {
      if (station_ids.count(id) > 0) {
        has_station = true;
        break;
      }
    }
    if (has_station) {
      report.components.push_back(ids_in_component);
    }
  }
  std::sort(report.components.begin(), report.components.end());

  // 路网节点不在主分量里：骨架图的短支路，可接受（只告警）
  for (const auto& node : catalog.road_nodes) {
    if (component_of[index_of[node.id]] != main_component) {
      report.dead_end_road_nodes.push_back(node.id);
    }
  }
  std::sort(report.dead_end_road_nodes.begin(), report.dead_end_road_nodes.end());

  report.all_stations_reachable = report.unreachable_stations.empty();
  if (report.all_stations_reachable) {
    report.message = "all " + std::to_string(catalog.stations.size()) + " station(s) reachable";
    if (!report.dead_end_road_nodes.empty()) {
      report.message += " (" + std::to_string(report.dead_end_road_nodes.size()) +
                        " dead-end road node(s), acceptable)";
    }
  } else {
    report.message = std::to_string(report.unreachable_stations.size()) +
                     " station(s) unreachable: ";
    for (std::size_t i = 0; i < report.unreachable_stations.size(); ++i) {
      if (i) report.message += ",";
      report.message += report.unreachable_stations[i];
    }
  }
  return report;
}

}  // namespace amr_dispatcher_core::catalog
