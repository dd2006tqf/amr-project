// 优先级时空多智能体寻路算法 (Prioritized Planning / PBS)。
//
// 对应 Phase 10 (P0)：
// 在多智能体协同寻路中，按指定或计算出的优先级次序逐个规划智能体路径：
// 1. 高优先级智能体先规划无约束或仅受初始在途交通约束的最优路径；
// 2. 将高优先级智能体规划所得的时空轨迹转换为后续低优先级智能体的时空避障约束；
// 3. 【目标驻留保护 (Goal-Hold)】：智能体到达终点后，自动向后续低优先级智能体注入
//    持续至时间地平线 (Horizon) 的顶点约束，确保已停靠的车辆不被后来车辆穿身穿越；
// 4. 【零碰撞保证门禁】：所有路径规划完成后，执行全局冲突检测 (DetectFirstConflict)，
//    确保完全零碰撞；
// 5. 相比 CBS，Prioritized Planning 求解复杂度为 O(N * A*)，具有多车极速收敛与高可扩展性，
//    非常适合作为大规模车队调度或 CBS 搜索超限时的快速降级求解器。
#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include "amr_dispatcher_core/catalog/station_catalog.hpp"
#include "amr_dispatcher_core/planning/conflict_based_search.hpp"
#include "amr_dispatcher_core/planning/space_time_astar.hpp"

namespace amr_dispatcher_core::planning {

enum class PriorityOrder {
  kGiven,          // 按请求中提供的列表原始顺序
  kShortestFirst,  // 最短单车行程优先
  kLongestFirst,   // 最长单车行程优先
};

struct PrioritizedOptions {
  PriorityOrder order = PriorityOrder::kShortestFirst;
  bool hold_agents_at_goal = true;  // 开启终点驻留保护
  int horizon_retries = 2;          // 低层寻路超时时动态翻倍 max_time_steps 的重试次数
};

struct PrioritizedRequest {
  std::vector<CBSAgentSpec> agents;
  double travel_speed_mps = 1.0;
  double time_step_duration_sec = 1.0;

  // 初始外加环境约束 (如在途车辆占用的时空禁区)
  std::vector<VertexConstraint> initial_vertex_constraints;
  std::vector<EdgeConstraint> initial_edge_constraints;

  int max_time_steps = 128;
  int low_level_max_nodes = 2048;

  PrioritizedOptions options;
};

struct PrioritizedResult {
  bool success = false;
  std::unordered_map<std::string, TimedPath> solution;
  double sum_of_costs = 0.0;
  int makespan = 0;
  bool verified_conflict_free = false;
  std::string message;
};

// 纯函数：优先级时空多智能体寻路
PrioritizedResult PlanPrioritizedPaths(
    const PrioritizedRequest& request,
    const amr_dispatcher_core::catalog::StationCatalog& catalog);

}  // namespace amr_dispatcher_core::planning
