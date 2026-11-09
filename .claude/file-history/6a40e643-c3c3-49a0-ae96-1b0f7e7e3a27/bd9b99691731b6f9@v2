#include "amr_dispatcher_core/planning/conflict_based_search.hpp"

#include <algorithm>
#include <queue>
#include <unordered_set>

namespace amr_dispatcher_core::planning {

namespace {

// 获取智能体在给定时间步所在的位置（或正穿行的边）
struct AgentStepState {
  std::string curr_station;
  std::string next_station;
  bool is_moving = false;
  bool is_active = false;
};

// 获取智能体在给定时间步所在的位置
// 若 tick 正好命中路径中某个离散到达点，则返回该点的 station_id (is_moving=false)
// 若 tick 处于两点之间行驶，则 is_moving=true
AgentStepState GetAgentStateAtTick(const TimedPath& path, int tick) {
  AgentStepState state;
  if (path.empty()) return state;

  if (tick < path.front().time_step || tick > path.back().time_step) {
    state.is_active = false;
    return state;
  }

  state.is_active = true;

  // 1. 先检查是否精确处于某个航路点或等待点
  for (const auto& pt : path) {
    if (pt.time_step == tick) {
      state.curr_station = pt.station_id;
      state.next_station = pt.station_id;
      state.is_moving = false;
      return state;
    }
  }

  // 2. 处于航段行驶过程中
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const auto& p1 = path[i];
    const auto& p2 = path[i + 1];
    if (tick > p1.time_step && tick < p2.time_step) {
      state.curr_station = p1.station_id;
      state.next_station = p2.station_id;
      state.is_moving = true;
      return state;
    }
  }

  state.curr_station = path.back().station_id;
  state.next_station = state.curr_station;
  state.is_moving = false;
  return state;
}

// 高层约束树节点 (CT Node)
struct CTNode {
  int id = 0;
  // 各智能体拥有的时空约束
  std::unordered_map<std::string, std::vector<VertexConstraint>> vertex_constraints;
  std::unordered_map<std::string, std::vector<EdgeConstraint>> edge_constraints;

  std::unordered_map<std::string, TimedPath> solution;
  double cost = 0.0; // Sum-of-Costs
  std::optional<AgentConflict> first_conflict;

  bool operator>(const CTNode& other) const {
    if (cost != other.cost) {
      return cost > other.cost;
    }
    return id > other.id; // tie-break
  }
};

}  // namespace

std::optional<AgentConflict> DetectFirstConflict(
    const std::unordered_map<std::string, TimedPath>& paths) {
  if (paths.size() < 2) {
    return std::nullopt;
  }

  // 计算全局最大时间步
  int max_t = 0;
  for (const auto& [agent, path] : paths) {
    if (!path.empty()) {
      max_t = std::max(max_t, path.back().time_step);
    }
  }

  // 转换为稳定顺序的向量
  std::vector<std::string> agent_ids;
  for (const auto& [agent, _] : paths) {
    agent_ids.push_back(agent);
  }
  std::sort(agent_ids.begin(), agent_ids.end());

  // 逐时间步扫描冲突（优先发现时间最早的冲突）
  for (int t = 0; t <= max_t + 1; ++t) {
    for (std::size_t i = 0; i < agent_ids.size(); ++i) {
      const auto& a1 = agent_ids[i];
      const auto& path1 = paths.at(a1);
      const auto state1 = GetAgentStateAtTick(path1, t);

      for (std::size_t j = i + 1; j < agent_ids.size(); ++j) {
        const auto& a2 = agent_ids[j];
        const auto& path2 = paths.at(a2);
        const auto state2 = GetAgentStateAtTick(path2, t);

        // 仅当两个智能体在时刻 t 均处于各自运行生命周期内时才触发时空冲突
        if (!state1.is_active || !state2.is_active) {
          continue;
        }

        // 1. 检查顶点冲突：在整点时间步 t，两智能体同时精确处于站 u（停顿或经过整点）
        // 若一方正在沿路段移动 (is_moving=true)，则其正在占用通道，不属于整点站点静止/过站冲突
        if (!state1.is_moving && !state2.is_moving) {
          if (state1.curr_station == state2.curr_station) {
            AgentConflict c;
            c.type = ConflictType::kVertex;
            c.agent_a = a1;
            c.agent_b = a2;
            c.station_u = state1.curr_station;
            c.time_step = t;
            return c;
          }
        }

        // 2. 检查边互换穿行对撞冲突 (Swap Conflict)
        // a1: u -> v, a2: v -> u
        if (state1.is_moving && state2.is_moving) {
          if (state1.curr_station == state2.next_station &&
              state1.next_station == state2.curr_station) {
            AgentConflict c;
            c.type = ConflictType::kEdgeSwap;
            c.agent_a = a1;
            c.agent_b = a2;
            c.station_u = state1.curr_station;
            c.station_v = state1.next_station;
            c.time_step = t;
            c.time_step_end = t + 10; // 保守阻断对撞区间
            return c;
          }

          // 3. 同向同段超车/重叠追尾 (Following Conflict)
          if (state1.curr_station == state2.curr_station &&
              state1.next_station == state2.next_station) {
            AgentConflict c;
            c.type = ConflictType::kEdgeFollow;
            c.agent_a = a1;
            c.agent_b = a2;
            c.station_u = state1.curr_station;
            c.station_v = state1.next_station;
            c.time_step = t;
            c.time_step_end = t + 5;
            return c;
          }
        }
      }
    }
  }

  return std::nullopt;
}

CBSResult PlanConflictBasedSearch(
    const CBSRequest& request,
    const amr_dispatcher_core::catalog::StationCatalog& catalog) {
  CBSResult result;

  if (request.agents.empty()) {
    result.success = true;
    result.message = "no agents to plan";
    return result;
  }

  auto cmp = [](const std::shared_ptr<CTNode>& a, const std::shared_ptr<CTNode>& b) {
    return *a > *b;
  };
  std::priority_queue<std::shared_ptr<CTNode>,
                      std::vector<std::shared_ptr<CTNode>>,
                      decltype(cmp)> open_list(cmp);

  int node_id_gen = 0;
  auto root = std::make_shared<CTNode>();
  root->id = ++node_id_gen;

  // 1. 初始化各智能体初始约束并运行独立 Low-Level A*
  double initial_cost = 0.0;
  for (const auto& agent_spec : request.agents) {
    root->vertex_constraints[agent_spec.agent_id] = request.initial_vertex_constraints;
    root->edge_constraints[agent_spec.agent_id] = request.initial_edge_constraints;

    SpaceTimeAStarRequest req;
    req.agent_id = agent_spec.agent_id;
    req.start_station = agent_spec.start_station;
    req.goal_station = agent_spec.goal_station;
    req.start_time_step = agent_spec.start_time_step;
    req.start_time_sec = agent_spec.start_time_sec;
    req.travel_speed_mps = request.travel_speed_mps;
    req.time_step_duration_sec = request.time_step_duration_sec;
    req.max_time_steps = request.max_time_steps;
    req.max_expanded_nodes = request.low_level_max_nodes;
    req.vertex_constraints = root->vertex_constraints[agent_spec.agent_id];
    req.edge_constraints = root->edge_constraints[agent_spec.agent_id];

    auto astar_res = PlanSpaceTimeAStar(req, catalog);
    if (!astar_res.success) {
      result.success = false;
      result.message = "initial low-level planning failed for agent " + agent_spec.agent_id +
                       ": " + astar_res.message;
      return result;
    }

    root->solution[agent_spec.agent_id] = std::move(astar_res.path);
    initial_cost += astar_res.total_cost;
  }

  root->cost = initial_cost;
  root->first_conflict = DetectFirstConflict(root->solution);

  open_list.push(root);

  while (!open_list.empty()) {
    auto current = open_list.top();
    open_list.pop();

    if (++result.expanded_ct_nodes > request.max_ct_nodes) {
      result.success = false;
      result.message = "exceeded max_ct_nodes limit (" +
                       std::to_string(request.max_ct_nodes) + ")";
      return result;
    }

    // 若无冲突，则找到了全局最优无冲突解
    if (!current->first_conflict.has_value()) {
      result.success = true;
      result.solution = std::move(current->solution);
      result.sum_of_costs = current->cost;
      result.message = "success";
      return result;
    }

    // 存在冲突，进行高层分支展开 (Branching)
    const auto& conflict = *current->first_conflict;

    // 对智能体 A 和智能体 B 分别生成子节点
    for (int branch = 0; branch < 2; ++branch) {
      const std::string target_agent = (branch == 0) ? conflict.agent_a : conflict.agent_b;
      auto child = std::make_shared<CTNode>();
      child->id = ++node_id_gen;
      child->vertex_constraints = current->vertex_constraints;
      child->edge_constraints = current->edge_constraints;
      child->solution = current->solution;

      // 注入对应的互斥约束
      if (conflict.type == ConflictType::kVertex) {
        VertexConstraint vc;
        vc.agent_id = target_agent;
        vc.station_id = conflict.station_u;
        vc.time_step = conflict.time_step;
        child->vertex_constraints[target_agent].push_back(vc);
      } else if (conflict.type == ConflictType::kEdgeSwap ||
                 conflict.type == ConflictType::kEdgeFollow) {
        EdgeConstraint ec;
        ec.agent_id = target_agent;
        ec.from_station = conflict.station_u;
        ec.to_station = conflict.station_v;
        ec.t_start = conflict.time_step;
        ec.t_end = (conflict.time_step_end > conflict.time_step) ? conflict.time_step_end
                                                                : conflict.time_step + 5;
        child->edge_constraints[target_agent].push_back(ec);
      }

      // 仅为该 target_agent 重新运行 Low-Level A* 更新路径
      const CBSAgentSpec* target_spec = nullptr;
      for (const auto& a : request.agents) {
        if (a.agent_id == target_agent) {
          target_spec = &a;
          break;
        }
      }
      if (target_spec == nullptr) continue;

      SpaceTimeAStarRequest req;
      req.agent_id = target_spec->agent_id;
      req.start_station = target_spec->start_station;
      req.goal_station = target_spec->goal_station;
      req.start_time_step = target_spec->start_time_step;
      req.start_time_sec = target_spec->start_time_sec;
      req.travel_speed_mps = request.travel_speed_mps;
      req.time_step_duration_sec = request.time_step_duration_sec;
      req.max_time_steps = request.max_time_steps;
      req.max_expanded_nodes = request.low_level_max_nodes;
      req.vertex_constraints = child->vertex_constraints[target_agent];
      req.edge_constraints = child->edge_constraints[target_agent];

      auto replan_res = PlanSpaceTimeAStar(req, catalog);
      if (!replan_res.success) {
        // 该分支在此约束下无可行解，剪枝
        continue;
      }

      child->solution[target_agent] = std::move(replan_res.path);

      // 重新计算 Sum-of-Costs
      double new_cost = 0.0;
      for (const auto& [_, p] : child->solution) {
        if (!p.empty()) {
          new_cost += (p.back().arrival_time_sec - p.front().arrival_time_sec);
        }
      }
      child->cost = new_cost;
      child->first_conflict = DetectFirstConflict(child->solution);

      open_list.push(child);
    }
  }

  result.success = false;
  result.message = "no feasible conflict-free solution found";
  return result;
}

}  // namespace amr_dispatcher_core::planning
