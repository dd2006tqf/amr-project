#include "amr_dispatcher_core/planning/mapf_solver.hpp"

namespace amr_dispatcher_core::planning {

MAPFUnifiedResult SolveMAPF(
    const MAPFUnifiedRequest& request,
    const amr_dispatcher_core::catalog::StationCatalog& catalog) {
  MAPFUnifiedResult result;
  if (request.agents.empty()) {
    result.success = true;
    result.verified_conflict_free = true;
    result.solver_used = "none";
    result.message = "no agents";
    return result;
  }

  const bool prefer_prioritized =
      (request.policy.mode == MAPFSolverMode::kPrioritizedOnly) ||
      (request.policy.mode == MAPFSolverMode::kAuto &&
       static_cast<int>(request.agents.size()) > request.policy.cbs_agent_threshold);

  if (prefer_prioritized) {
    PrioritizedRequest p_req;
    p_req.agents = request.agents;
    p_req.travel_speed_mps = request.travel_speed_mps;
    p_req.time_step_duration_sec = request.time_step_duration_sec;
    p_req.initial_vertex_constraints = request.initial_vertex_constraints;
    p_req.initial_edge_constraints = request.initial_edge_constraints;
    p_req.max_time_steps = request.max_time_steps;
    p_req.low_level_max_nodes = request.low_level_max_nodes;
    p_req.options = request.policy.prioritized_options;

    auto p_res = PlanPrioritizedPaths(p_req, catalog);
    result.success = p_res.success;
    result.solution = std::move(p_res.solution);
    result.sum_of_costs = p_res.sum_of_costs;
    result.makespan = p_res.makespan;
    result.verified_conflict_free = p_res.verified_conflict_free;
    result.solver_used = "prioritized";
    result.message = p_res.message;
    return result;
  }

  // 尝试使用 CBS 搜索最优解
  CBSRequest cbs_req;
  cbs_req.agents = request.agents;
  cbs_req.travel_speed_mps = request.travel_speed_mps;
  cbs_req.time_step_duration_sec = request.time_step_duration_sec;
  cbs_req.initial_vertex_constraints = request.initial_vertex_constraints;
  cbs_req.initial_edge_constraints = request.initial_edge_constraints;
  cbs_req.max_ct_nodes = request.policy.cbs_max_ct_nodes;
  cbs_req.max_time_steps = request.max_time_steps;
  cbs_req.low_level_max_nodes = request.low_level_max_nodes;

  auto cbs_res = PlanConflictBasedSearch(cbs_req, catalog);

  if (cbs_res.success) {
    result.success = true;
    result.solution = std::move(cbs_res.solution);
    result.sum_of_costs = cbs_res.sum_of_costs;
    result.verified_conflict_free = true;
    result.solver_used = "cbs";
    result.message = "optimal solution found by CBS";

    int max_ms = 0;
    for (const auto& [_, p] : result.solution) {
      if (!p.empty()) {
        max_ms = std::max(max_ms, p.back().time_step);
      }
    }
    result.makespan = max_ms;
    return result;
  }

  // CBS 失败或超限，检查是否允许自适应降级至 Prioritized Planning
  if (request.policy.mode == MAPFSolverMode::kAuto && request.policy.fallback_to_prioritized) {
    result.fallback_reason = cbs_res.message;

    PrioritizedRequest p_req;
    p_req.agents = request.agents;
    p_req.travel_speed_mps = request.travel_speed_mps;
    p_req.time_step_duration_sec = request.time_step_duration_sec;
    p_req.initial_vertex_constraints = request.initial_vertex_constraints;
    p_req.initial_edge_constraints = request.initial_edge_constraints;
    p_req.max_time_steps = request.max_time_steps;
    p_req.low_level_max_nodes = request.low_level_max_nodes;
    p_req.options = request.policy.prioritized_options;

    auto p_res = PlanPrioritizedPaths(p_req, catalog);
    result.success = p_res.success;
    result.solution = std::move(p_res.solution);
    result.sum_of_costs = p_res.sum_of_costs;
    result.makespan = p_res.makespan;
    result.verified_conflict_free = p_res.verified_conflict_free;
    result.solver_used = "cbs->prioritized";
    result.message = "fallback to prioritized planning: " + p_res.message;
    return result;
  }

  result.success = false;
  result.solver_used = "cbs";
  result.message = "CBS failed and fallback disabled: " + cbs_res.message;
  return result;
}

}  // namespace amr_dispatcher_core::planning
