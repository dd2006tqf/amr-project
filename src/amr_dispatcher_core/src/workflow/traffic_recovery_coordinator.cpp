#include "amr_dispatcher_core/workflow/traffic_recovery_coordinator.hpp"

#include <algorithm>

namespace amr_dispatcher_core::workflow {

DeadlockRecoveryDecision PlanDeadlockRecovery(const DeadlockRecoveryRequest& request) {
  DeadlockRecoveryDecision decision;

  if (request.cycle_mission_ids.empty() || request.candidates.empty() || request.catalog == nullptr) {
    decision.action = DeadlockRecoveryDecision::Action::kNoActionNeeded;
    decision.message = "no deadlock cycle or missing catalog/candidates";
    return decision;
  }

  // 1. 筛选出处于死锁环中的候选任务，并按优先级升序排序（低优先级优先作为牺牲者去绕行）
  std::vector<DeadlockRecoveryCandidate> cycle_candidates;
  for (const auto& cand : request.candidates) {
    if (std::find(request.cycle_mission_ids.begin(), request.cycle_mission_ids.end(),
                  cand.mission_id) != request.cycle_mission_ids.end()) {
      cycle_candidates.push_back(cand);
    }
  }

  if (cycle_candidates.empty()) {
    decision.action = DeadlockRecoveryDecision::Action::kNoActionNeeded;
    decision.message = "candidates do not match cycle mission ids";
    return decision;
  }

  // 优先排序：优先级较低者先尝试绕行；若同优先级，按 mission_id 字典序保持确定性
  std::sort(cycle_candidates.begin(), cycle_candidates.end(),
            [](const DeadlockRecoveryCandidate& a, const DeadlockRecoveryCandidate& b) {
              if (a.priority != b.priority) {
                return a.priority < b.priority;
              }
              return a.mission_id < b.mission_id;
            });

  // 2. 依次尝试为候选任务计算避开冲突边的替代路线
  for (const auto& cand : cycle_candidates) {
    InFlightRerouteRequest reroute_req;
    reroute_req.mission_id = cand.mission_id;
    reroute_req.vehicle_id = cand.vehicle_id;
    reroute_req.current_safe_node = cand.current_safe_node;
    reroute_req.destination_node = cand.destination_node;
    reroute_req.current_held_locks = cand.current_held_locks;
    reroute_req.full_original_path = cand.full_original_path;
    reroute_req.blocked_edge_keys = request.cycle_blocked_edges;
    reroute_req.catalog = request.catalog;

    auto reroute_res = PlanInFlightReroute(reroute_req);
    if (reroute_res.outcome == InFlightRerouteDecision::Outcome::kRerouteReady) {
      decision.action = DeadlockRecoveryDecision::Action::kRerouteMission;
      decision.selected_mission_id = cand.mission_id;
      decision.reroute_decision = std::move(reroute_res);
      decision.message = "selected mission " + cand.mission_id + " to reroute around conflict";
      return decision;
    }
  }

  // 3. 所有涉案任务均无可用替代路线：判定为无法自动拓扑破环，转入受控安全等待/挂起
  decision.action = DeadlockRecoveryDecision::Action::kSafeStopSuspend;
  decision.message = "all cycle missions have no alternative bypass routes; fallback to safe stop wait";
  return decision;
}

}  // namespace amr_dispatcher_core::workflow
