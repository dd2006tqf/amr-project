#include "amr_dispatcher_core/planning/prioritized_planning.hpp"

#include <algorithm>
#include <cmath>

namespace amr_dispatcher_core::planning {

namespace {

// 估计智能体无约束情况下的静态行程距离
double EstimateDirectDistance(const amr_dispatcher_core::catalog::StationCatalog& catalog,
                              const std::string& start, const std::string& goal) {
  auto d = amr_dispatcher_core::catalog::EstimateRouteDistance(catalog, start, goal);
  return d.has_value() ? *d : 1e6;
}

// 将单个已规划好的智能体轨迹转换为后续智能体的 Vertex/Edge 约束
void AppendPathToConstraints(
    const TimedPath& path,
    int max_horizon_ticks,
    bool hold_at_goal,
    std::vector<VertexConstraint>& out_vertex_constraints,
    std::vector<EdgeConstraint>& out_edge_constraints) {
  if (path.empty()) return;

  // 1. 沿途顶点与等待约束
  for (const auto& pt : path) {
    VertexConstraint vc;
    vc.agent_id = ""; // 对后续所有低优先级 agent 生效
    vc.station_id = pt.station_id;
    vc.time_step = pt.time_step;
    out_vertex_constraints.push_back(vc);
  }

  // 2. 沿途航段边约束 (对向互换与追尾拦截)
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const auto& p1 = path[i];
    const auto& p2 = path[i + 1];
    if (p1.station_id != p2.station_id) {
      EdgeConstraint ec;
      ec.agent_id = "";
      ec.from_station = p1.station_id;
      ec.to_station = p2.station_id;
      ec.t_start = p1.time_step;
      ec.t_end = p2.time_step;
      out_edge_constraints.push_back(ec);
    }
  }

  // 3. 终点驻留保护 (Goal-Hold)：到达终点后，禁止后续车辆占用终点站直至地平线
  if (hold_at_goal) {
    const std::string goal_station = path.back().station_id;
    const int arrival_tick = path.back().time_step;
    // 覆盖到达时刻及未来整个地平线 (如 max_horizon_ticks * 4，彻底保护)
    const int protect_horizon = std::max(max_horizon_ticks * 4, 1024);
    for (int t = arrival_tick; t <= protect_horizon; ++t) {
      VertexConstraint vc;
      vc.agent_id = "";
      vc.station_id = goal_station;
      vc.time_step = t;
      out_vertex_constraints.push_back(vc);
    }
  }
}

}  // namespace

PrioritizedResult PlanPrioritizedPaths(
    const PrioritizedRequest& request,
    const amr_dispatcher_core::catalog::StationCatalog& catalog) {
  PrioritizedResult result;
  if (request.agents.empty()) {
    result.success = true;
    result.verified_conflict_free = true;
    result.message = "no agents";
    return result;
  }

  // 1. 智能体优先级排序
  auto ordered_agents = request.agents;
  if (request.options.order == PriorityOrder::kShortestFirst) {
    std::sort(ordered_agents.begin(), ordered_agents.end(),
              [&](const CBSAgentSpec& a, const CBSAgentSpec& b) {
                double da = EstimateDirectDistance(catalog, a.start_station, a.goal_station);
                double db = EstimateDirectDistance(catalog, b.start_station, b.goal_station);
                if (da != db) return da < db;
                return a.agent_id < b.agent_id; // 全序 tie-break
              });
  } else if (request.options.order == PriorityOrder::kLongestFirst) {
    std::sort(ordered_agents.begin(), ordered_agents.end(),
              [&](const CBSAgentSpec& a, const CBSAgentSpec& b) {
                double da = EstimateDirectDistance(catalog, a.start_station, a.goal_station);
                double db = EstimateDirectDistance(catalog, b.start_station, b.goal_station);
                if (da != db) return da > db;
                return a.agent_id < b.agent_id;
              });
  }

  // 动态累积时空约束
  auto accumulated_vertex_constraints = request.initial_vertex_constraints;
  auto accumulated_edge_constraints = request.initial_edge_constraints;

  int current_horizon = request.max_time_steps;
  double total_cost = 0.0;
  int max_makespan = 0;

  // 2. 依次为各智能体规划时空轨迹
  for (const auto& agent_spec : ordered_agents) {
    bool agent_solved = false;
    TimedPath agent_path;

    int retries_left = request.options.horizon_retries;
    int search_horizon = current_horizon;

    while (!agent_solved && retries_left >= 0) {
      SpaceTimeAStarRequest req;
      req.agent_id = agent_spec.agent_id;
      req.start_station = agent_spec.start_station;
      req.goal_station = agent_spec.goal_station;
      req.start_time_step = agent_spec.start_time_step;
      req.start_time_sec = agent_spec.start_time_sec;
      req.travel_speed_mps = request.travel_speed_mps;
      req.time_step_duration_sec = request.time_step_duration_sec;
      req.max_time_steps = search_horizon;
      req.max_expanded_nodes = request.low_level_max_nodes;
      req.vertex_constraints = accumulated_vertex_constraints;
      req.edge_constraints = accumulated_edge_constraints;

      auto astar_res = PlanSpaceTimeAStar(req, catalog);
      if (astar_res.success) {
        agent_solved = true;
        agent_path = std::move(astar_res.path);
        total_cost += astar_res.total_cost;
        if (!agent_path.empty()) {
          max_makespan = std::max(max_makespan, agent_path.back().time_step);
        }
      } else {
        --retries_left;
        search_horizon *= 2; // 动态放宽时间地平线重试，允许更长时间的避让停留
      }
    }

    if (!agent_solved) {
      result.success = false;
      result.message = "prioritized planning failed for agent: " + agent_spec.agent_id;
      return result;
    }

    // 将已规划好的路径并入全局时空约束，阻断后续智能体碰撞
    AppendPathToConstraints(
        agent_path, search_horizon, request.options.hold_agents_at_goal,
        accumulated_vertex_constraints, accumulated_edge_constraints);

    result.solution[agent_spec.agent_id] = std::move(agent_path);
  }

  // 3. 终审安全门禁：执行全局冲突校验 (DetectFirstConflict)
  auto conflict = DetectFirstConflict(result.solution);
  if (conflict.has_value()) {
    result.success = false;
    result.verified_conflict_free = false;
    result.message = "post-planning conflict verification failed";
    return result;
  }

  result.success = true;
  result.verified_conflict_free = true;
  result.sum_of_costs = total_cost;
  result.makespan = max_makespan;
  result.message = "success";
  return result;
}

}  // namespace amr_dispatcher_core::planning
