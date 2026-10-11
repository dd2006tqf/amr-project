#include "amr_dispatcher_core/dispatcher/deadlock_detector.hpp"

#include <algorithm>
#include <iterator>
#include <unordered_map>

namespace amr_dispatcher_core::dispatcher {

DeadlockDetector::DeadlockDetector(DeadlockDetectorConfig config)
    : config_(config) {}

std::vector<std::vector<std::string>> DeadlockDetector::DetectCycles(
    const ResourceAllocationModel& wfg_model) const {
  std::vector<std::vector<std::string>> cycles;

  // 1. 构建任务等待图 (Wait-For Graph)：
  // 任务 M1 等待资源 R，而 R 当前被 M2 持有 -> 有向边 M1 -> M2，并记下触发该边的资源
  //（环路径要以 waiter -> resource -> holder 交替记录）。
  // 同一 (等待者, 持有者) 只建一条边：多把锁指向同一持有者时不重复上报同一环，
  // 与改前 std::set 去重语义一致。
  struct WaitEdge {
    std::string holder;
    std::string resource_id;
  };
  std::unordered_map<std::string, std::vector<WaitEdge>> adj;
  for (const auto& [waiter_m, requested_resources] : wfg_model.waiting_for) {
    for (const auto& res_id : requested_resources) {
      auto it = wfg_model.held_by.find(res_id);
      if (it == wfg_model.held_by.end() || it->second == waiter_m) {
        continue;
      }
      auto& edges = adj[waiter_m];
      const bool duplicated =
          std::any_of(edges.begin(), edges.end(),
                      [&it](const WaitEdge& edge) { return edge.holder == it->second; });
      if (!duplicated) {
        edges.push_back(WaitEdge{it->second, res_id});
      }
    }
  }

  // 2. 使用 DFS 进行拓扑三色标记法或路径回溯检测有向图环
  // 0: 未访问, 1: 正在当前递归栈中 (在探索), 2: 已完成探索
  // path 只记任务节点；path_res[i] 记 path[i] -> path[i+1] 所等待的资源。
  std::unordered_map<std::string, int> visit_state;
  std::vector<std::string> path;
  std::vector<std::string> path_res;

  auto dfs = [&](auto& self, const std::string& u) -> void {
    visit_state[u] = 1;
    path.push_back(u);

    auto it = adj.find(u);
    if (it != adj.end()) {
      for (const auto& edge : it->second) {
        const std::string& v = edge.holder;
        if (visit_state[v] == 1) {
          // 找到环！自栈上 v 起提取到当前 u，段间用 path_res 的资源，
          // 末段用回边 (u -> v) 的资源并闭合回 v：
          //   [v, res, ..., u, res, v] —— 任务与资源交替，死锁可视化的
          //   deadlocked_nodes 由其中的 route_edge:/route_node: 项解析而来。
          const auto cycle_start = std::find(path.begin(), path.end(), v);
          if (cycle_start != path.end()) {
            const auto start_index =
                static_cast<std::size_t>(std::distance(path.begin(), cycle_start));
            std::vector<std::string> cycle;
            for (std::size_t index = start_index; index < path.size(); ++index) {
              cycle.push_back(path[index]);
              cycle.push_back(index + 1 < path.size() ? path_res[index]
                                                      : edge.resource_id);
            }
            cycle.push_back(path[start_index]);  // 闭合回起始任务
            cycles.push_back(cycle);
          }
        } else if (visit_state[v] == 0) {
          path_res.push_back(edge.resource_id);
          self(self, v);
          path_res.pop_back();
        }
      }
    }

    path.pop_back();
    visit_state[u] = 2;
  };

  // 起点按字典序遍历：同一张图的环报告次序可复现（unordered_map 顺序不稳定）
  std::vector<std::string> start_nodes;
  start_nodes.reserve(adj.size());
  for (const auto& [node, _] : adj) {
    start_nodes.push_back(node);
  }
  std::sort(start_nodes.begin(), start_nodes.end());
  for (const auto& node : start_nodes) {
    if (visit_state[node] == 0) {
      dfs(dfs, node);
    }
  }

  return cycles;
}

std::vector<DeadlockFinding> DeadlockDetector::Detect(
    const DeadlockSnapshot& snapshot) const {
  std::vector<DeadlockFinding> findings;
  const auto now = std::chrono::steady_clock::now();

  // 1. 【严密图论判定】通过 Wait-For Graph 进行 0 毫秒确定性死锁环检测
  const auto cycles = DetectCycles(snapshot.allocation_graph);
  for (const auto& cycle : cycles) {
    DeadlockFinding f;
    f.kind = DeadlockKind::kMutualBlock;
    f.cycle_path = cycle;
    // cycle_path 为任务/资源交替且首尾闭合：偶数位是任务，奇数位是资源，
    // mission_ids 只取任务（recovery 据此对任务下发暂停/取消，不能混入资源 id）
    for (std::size_t i = 0; i + 1 < cycle.size(); i += 2) {
      f.mission_ids.push_back(cycle[i]);
    }
    std::string desc = "Wait-For Graph Cycle detected: [";
    for (size_t i = 0; i < cycle.size(); ++i) {
      desc += cycle[i];
      if (i + 1 < cycle.size()) desc += " -> ";
    }
    desc += "]";  // cycle_path 自身闭合回起始任务
    f.description = desc;
    findings.push_back(std::move(f));
  }

  // 2. 启发式活跃任务过多检测
  if (config_.max_active_missions > 0 &&
      snapshot.active_missions.size() > config_.max_active_missions) {
    DeadlockFinding f;
    f.kind = DeadlockKind::kMutualBlock;
    f.description = "too many active missions: " +
                    std::to_string(snapshot.active_missions.size());
    for (const auto& m : snapshot.active_missions) {
      f.mission_ids.push_back(m.id);
    }
    findings.push_back(std::move(f));
  }

  // 3. 启发式：ACTIVE 但无 progress 超过阈值
  for (const auto& m : snapshot.active_missions) {
    auto it = snapshot.last_progress.find(m.id);
    if (it == snapshot.last_progress.end()) {
      continue;
    }
    if (now - it->second > config_.active_idle_threshold) {
      DeadlockFinding f;
      f.kind = DeadlockKind::kIdleActive;
      f.description = "mission idle too long: " + m.id;
      f.mission_ids.push_back(m.id);
      findings.push_back(std::move(f));
    }
  }

  // 4. 启发式：长期持锁检测
  std::unordered_map<std::string, std::vector<std::string>> resource_to_missions;
  for (const auto& m : snapshot.active_missions) {
    if (!m.reservation_id.empty()) {
      resource_to_missions[m.reservation_id].push_back(m.id);
    }
  }
  for (const auto& [resource, acquired_at] : snapshot.reservation_acquired_at) {
    if (now - acquired_at <= config_.reservation_hold_threshold) {
      continue;
    }
    auto it = resource_to_missions.find(resource);
    if (it == resource_to_missions.end() || it->second.empty()) {
      continue;
    }
    DeadlockFinding f;
    f.kind = DeadlockKind::kLongReservationHold;
    f.description = "resource held too long: " + resource;
    f.mission_ids = it->second;
    findings.push_back(std::move(f));
  }

  return findings;
}

}  // namespace amr_dispatcher_core::dispatcher
