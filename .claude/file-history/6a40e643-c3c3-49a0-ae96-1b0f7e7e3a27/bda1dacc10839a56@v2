// 高层基于冲突的多智能体寻路搜索器 (High-Level Conflict-Based Search, CBS)。
//
// 对应 Phase 9 (P1)：
// 在多智能体路径规划 (MAPF) 中，通过维护两级搜索架构：
// - 低层 (Low-Level): 为单车在拓扑网络上搜索满足时空约束的最优路径 (PlanSpaceTimeAStar)
// - 高层 (High-Level): 构建约束树 (Constraint Tree, CT)，检测多车两两时空冲突，
//   并通过分支向各智能体注入时空约束，搜索全局总代价 (Sum-of-Costs) 最优的无冲突解。
//
// 冲突类型检测：
// 1. 顶点冲突 (Vertex Conflict): 两智能体在同一时间步出现在同一站点
// 2. 边冲突 (Edge/Swap Conflict): 两智能体在同一时间步在同一无序边上对向穿行或互换位置
// 3. 同向追尾/重叠冲突 (Following/Overlap Conflict): 两智能体在同一时间步同向进入同一物理路段
#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "amr_dispatcher_core/catalog/station_catalog.hpp"
#include "amr_dispatcher_core/planning/space_time_astar.hpp"

namespace amr_dispatcher_core::planning {

enum class ConflictType {
  kVertex,     // 顶点时空重叠
  kEdgeSwap,   // 对向互换穿行
  kEdgeFollow  // 同向追尾重叠
};

// 两智能体之间的时空冲突
struct AgentConflict {
  ConflictType type = ConflictType::kVertex;
  std::string agent_a;
  std::string agent_b;
  std::string station_u;  // 顶点冲突时的站点，或边冲突时的起点
  std::string station_v;  // 边冲突时的终点 (顶点冲突时为空)
  int time_step = 0;       // 冲突发生的时间步 (或边冲突起始时步)
  int time_step_end = 0;   // 边冲突结束时步
};

// 单智能体规划输入规范
struct CBSAgentSpec {
  std::string agent_id;
  std::string start_station;
  std::string goal_station;
  int start_time_step = 0;
  double start_time_sec = 0.0;
};

// CBS 整体请求参数
struct CBSRequest {
  std::vector<CBSAgentSpec> agents;
  double travel_speed_mps = 1.0;
  double time_step_duration_sec = 1.0;

  // 初始外加约束 (例如环境已有在途车辆占用的时空禁区)
  std::vector<VertexConstraint> initial_vertex_constraints;
  std::vector<EdgeConstraint> initial_edge_constraints;

  // 搜索限制
  int max_ct_nodes = 256;         // 最大展开高层约束树节点数
  int max_time_steps = 128;       // 单路径最大时间步
  int low_level_max_nodes = 2048; // 单次低层 A* 最大展开节点数
};

// CBS 协同规划结果
struct CBSResult {
  bool success = false;
  std::unordered_map<std::string, TimedPath> solution; // agent_id -> TimedPath
  double sum_of_costs = 0.0;
  int expanded_ct_nodes = 0;
  std::string message;
};

// 纯函数：多车冲突检测器 (扫描所有两两轨迹，返回发现的首个最早时空冲突)
std::optional<AgentConflict> DetectFirstConflict(
    const std::unordered_map<std::string, TimedPath>& paths);

// 纯函数：高层冲突树搜索 (CBS Solver)
CBSResult PlanConflictBasedSearch(
    const CBSRequest& request,
    const amr_dispatcher_core::catalog::StationCatalog& catalog);

}  // namespace amr_dispatcher_core::planning
