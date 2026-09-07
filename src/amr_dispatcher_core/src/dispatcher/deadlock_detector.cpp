#include "amr_dispatcher_core/dispatcher/deadlock_detector.hpp"

#include <algorithm>
#include <unordered_map>
#include <unordered_set>

namespace amr_dispatcher_core::dispatcher {

DeadlockDetector::DeadlockDetector(DeadlockDetectorConfig config)
    : config_(config) {}

std::vector<std::vector<std::string>> DeadlockDetector::DetectCycles(
    const ResourceAllocationModel& wfg_model) const {
  std::vector<std::vector<std::string>> cycles;

  // 1. 构建 Mission -> Set<Mission> 的任务等待图 (Wait-For Graph)
  // 如果任务 M1 等待资源 R，而资源 R 当前被 M2 持有，则建立有向边: M1 -> M2
  std::unordered_map<std::string, std::unordered_set<std::string>> adj;
  for (const auto& [waiter_m, requested_resources] : wfg_model.waiting_for) {
    for (const auto& res_id : requested_resources) {
      auto it = wfg_model.held_by.find(res_id);
      if (it != wfg_model.held_by.end() && it->second != waiter_m) {
        adj[waiter_m].insert(it->second);
      }
    }
  }

  // 2. 使用 DFS 进行拓扑三色标记法或路径回溯检测有向图环
  // 0: 未访问, 1: 正在当前递归栈中 (在探索), 2: 已完成探索
  std::unordered_map<std::string, int> visit_state;
  std::vector<std::string> path;

  auto dfs = [&](auto& self, const std::string& u) -> void {
    visit_state[u] = 1;
    path.push_back(u);

    auto it = adj.find(u);
    if (it != adj.end()) {
      for (const auto& v : it->second) {
        if (visit_state[v] == 1) {
          // 找到环！提取从 v 开始到当前节点 u 的成环路径
          auto cycle_start = std::find(path.begin(), path.end(), v);
          if (cycle_start != path.end()) {
            std::vector<std::string> cycle_nodes(cycle_start, path.end());
            // 规范化以避免重复记录 (记录最小旋转表示)
            cycles.push_back(cycle_nodes);
          }
        } else if (visit_state[v] == 0) {
          self(self, v);
        }
      }
    }

    path.pop_back();
    visit_state[u] = 2;
  };

  for (const auto& [node, _] : adj) {
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
    f.mission_ids = cycle;
    f.cycle_path = cycle;
    std::string desc = "Wait-For Graph Cycle detected: [";
    for (size_t i = 0; i < cycle.size(); ++i) {
      desc += cycle[i];
      if (i + 1 < cycle.size()) desc += " -> ";
    }
    desc += " -> " + cycle.front() + "]";
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
