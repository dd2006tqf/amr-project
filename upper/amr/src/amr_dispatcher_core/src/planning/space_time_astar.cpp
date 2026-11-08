#include "amr_dispatcher_core/planning/space_time_astar.hpp"

#include <algorithm>
#include <cmath>
#include <queue>
#include <unordered_set>

namespace amr_dispatcher_core::planning {

namespace {

// 启发式：基于拓扑图的单源最短距离 (静态 Dijkstra 距离)
double ComputeHeuristic(const amr_dispatcher_core::catalog::StationCatalog& catalog,
                        const std::string& current, const std::string& goal,
                        double travel_speed_mps) {
  if (current == goal) {
    return 0.0;
  }
  const auto dist = amr_dispatcher_core::catalog::EstimateRouteDistance(catalog, current, goal);
  if (!dist.has_value()) {
    return 1e6;  // 不可达
  }
  const double speed = (travel_speed_mps > 0.05) ? travel_speed_mps : 1.0;
  return *dist / speed;
}

// 内部搜索节点
struct AStarNode {
  std::string station_id;
  int time_step = 0;
  double arrival_time_sec = 0.0;
  double g_cost = 0.0;
  double f_cost = 0.0;
  std::shared_ptr<AStarNode> parent = nullptr;
  bool is_wait = false;

  bool operator>(const AStarNode& other) const {
    return f_cost > other.f_cost;
  }
};

std::string StateKey(const std::string& station_id, int time_step) {
  return station_id + "@" + std::to_string(time_step);
}

}  // namespace

SpaceTimeAStarResult PlanSpaceTimeAStar(
    const SpaceTimeAStarRequest& request,
    const amr_dispatcher_core::catalog::StationCatalog& catalog) {
  SpaceTimeAStarResult result;

  if (request.start_station.empty() || request.goal_station.empty()) {
    result.success = false;
    result.message = "start or goal station is empty";
    return result;
  }

  // 构建约束快速查找索引
  std::unordered_set<std::string> vertex_constraint_set;
  for (const auto& vc : request.vertex_constraints) {
    if (vc.agent_id.empty() || vc.agent_id == request.agent_id) {
      vertex_constraint_set.insert(StateKey(vc.station_id, vc.time_step));
    }
  }

  std::vector<EdgeConstraint> active_edge_constraints;
  for (const auto& ec : request.edge_constraints) {
    if (ec.agent_id.empty() || ec.agent_id == request.agent_id) {
      active_edge_constraints.push_back(ec);
    }
  }

  // 起点自身是否存在顶点约束
  if (vertex_constraint_set.count(StateKey(request.start_station, request.start_time_step))) {
    result.success = false;
    result.message = "start station is blocked by vertex constraint at start_time_step";
    return result;
  }

  const double speed = (request.travel_speed_mps > 0.05) ? request.travel_speed_mps : 1.0;
  const double dt = (request.time_step_duration_sec > 0.05) ? request.time_step_duration_sec : 1.0;

  auto cmp = [](const std::shared_ptr<AStarNode>& a, const std::shared_ptr<AStarNode>& b) {
    return a->f_cost > b->f_cost;
  };
  std::priority_queue<std::shared_ptr<AStarNode>,
                      std::vector<std::shared_ptr<AStarNode>>,
                      decltype(cmp)> open_list(cmp);

  // Closed 表：StateKey -> 最小 g_cost
  std::unordered_map<std::string, double> closed_set;

  auto root = std::make_shared<AStarNode>();
  root->station_id = request.start_station;
  root->time_step = request.start_time_step;
  root->arrival_time_sec = request.start_time_sec;
  root->g_cost = 0.0;
  root->f_cost = ComputeHeuristic(catalog, request.start_station, request.goal_station, speed);
  root->parent = nullptr;
  root->is_wait = false;

  open_list.push(root);

  std::shared_ptr<AStarNode> goal_node = nullptr;

  while (!open_list.empty()) {
    auto current = open_list.top();
    open_list.pop();

    if (++result.expanded_nodes > request.max_expanded_nodes) {
      result.success = false;
      result.message = "exceeded max_expanded_nodes limit (" +
                       std::to_string(request.max_expanded_nodes) + ")";
      return result;
    }

    const std::string curr_key = StateKey(current->station_id, current->time_step);
    auto it = closed_set.find(curr_key);
    if (it != closed_set.end() && it->second <= current->g_cost) {
      continue;
    }
    closed_set[curr_key] = current->g_cost;

    // 到达目标
    if (current->station_id == request.goal_station) {
      goal_node = current;
      break;
    }

    if (current->time_step >= request.max_time_steps) {
      continue;
    }

    // 1. 动作一：移动至邻接站点 (Move)
    for (const auto& edge : catalog.edges) {
      if (!edge.enabled) continue;
      std::string next_station;
      if (edge.from == current->station_id) {
        next_station = edge.to;
      } else if (edge.bidirectional && edge.to == current->station_id) {
        next_station = edge.from;
      } else {
        continue;
      }

      // 计算行驶耗时与离散时步增量
      const double move_duration = std::max(edge.distance_m / speed, dt);
      const int steps = std::max(1, static_cast<int>(std::round(move_duration / dt)));
      const int next_time_step = current->time_step + steps;

      if (next_time_step > request.max_time_steps) {
        continue;
      }

      // 边约束检查 (禁止在 [current->time_step, next_time_step] 区间内与约束边重叠)
      bool edge_blocked = false;
      for (const auto& ec : active_edge_constraints) {
        if ((ec.from_station == current->station_id && ec.to_station == next_station) ||
            (ec.from_station == next_station && ec.to_station == current->station_id)) {
          // 区间重叠判定: [current->time_step, next_time_step] 与 [ec.t_start, ec.t_end]
          if (std::max(current->time_step, ec.t_start) <= std::min(next_time_step, ec.t_end)) {
            edge_blocked = true;
            break;
          }
        }
      }
      if (edge_blocked) {
        continue;
      }

      // 顶点约束检查 (目标站在到达时步不得被阻断)
      if (vertex_constraint_set.count(StateKey(next_station, next_time_step))) {
        continue;
      }

      auto neighbor = std::make_shared<AStarNode>();
      neighbor->station_id = next_station;
      neighbor->time_step = next_time_step;
      neighbor->arrival_time_sec = current->arrival_time_sec + move_duration;
      neighbor->g_cost = current->g_cost + move_duration;
      neighbor->f_cost = neighbor->g_cost +
                         ComputeHeuristic(catalog, next_station, request.goal_station, speed);
      neighbor->parent = current;
      neighbor->is_wait = false;

      const std::string n_key = StateKey(next_station, next_time_step);
      if (closed_set.find(n_key) == closed_set.end() || closed_set[n_key] > neighbor->g_cost) {
        open_list.push(neighbor);
      }
    }

    // 2. 动作二：在当前站点停留等待 (Wait In Place)
    const int wait_time_step = current->time_step + 1;
    if (wait_time_step <= request.max_time_steps) {
      if (!vertex_constraint_set.count(StateKey(current->station_id, wait_time_step))) {
        auto wait_node = std::make_shared<AStarNode>();
        wait_node->station_id = current->station_id;
        wait_node->time_step = wait_time_step;
        wait_node->arrival_time_sec = current->arrival_time_sec + dt;
        wait_node->g_cost = current->g_cost + dt;
        wait_node->f_cost = wait_node->g_cost +
                            ComputeHeuristic(catalog, current->station_id, request.goal_station, speed);
        wait_node->parent = current;
        wait_node->is_wait = true;

        const std::string w_key = StateKey(current->station_id, wait_time_step);
        if (closed_set.find(w_key) == closed_set.end() || closed_set[w_key] > wait_node->g_cost) {
          open_list.push(wait_node);
        }
      }
    }
  }

  if (goal_node == nullptr) {
    result.success = false;
    result.message = "no feasible space-time path found under given constraints";
    return result;
  }

  // 回溯构建路径
  std::vector<TimedPathPoint> reversed_path;
  for (auto curr = goal_node; curr != nullptr; curr = curr->parent) {
    TimedPathPoint pt;
    pt.station_id = curr->station_id;
    pt.time_step = curr->time_step;
    pt.arrival_time_sec = curr->arrival_time_sec;
    pt.is_wait = curr->is_wait;
    reversed_path.push_back(pt);
  }
  std::reverse(reversed_path.begin(), reversed_path.end());

  result.success = true;
  result.path = std::move(reversed_path);
  result.total_cost = goal_node->g_cost;
  result.message = "success";
  return result;
}

}  // namespace amr_dispatcher_core::planning
