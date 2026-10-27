// 统一多智能体路径规划求解器门面 (Unified MAPF Solver Facade)。
//
// 对应 Phase 10 (P1)：
// 统一封装 CBS（高层约束树搜索）与 Prioritized Planning（优先级规划），实现自适应分级求解：
// 1. 小规模 / 精确场景 (agents <= threshold)：优先运行 CBS 寻找全局最优（Sum-of-Costs 最小）解；
// 2. 超限自适应降级：当 CBS 耗尽高层 CT 节点预算或规划失败时，自动平滑降级为 Prioritized Planning，
//    确保业务系统在极短时间内（毫秒级）产出无碰撞可行解，杜绝任务死锁或超时停滞；
// 3. 大规模并发：当智能体规模较大时，策略性直接采用 Prioritized Planning 极速规划；
// 4. 全局终审校验：所有求解输出均通过 DetectFirstConflict 验证，保证绝对零冲突。
#pragma once

#include <string>
#include <vector>

#include "amr_dispatcher_core/catalog/station_catalog.hpp"
#include "amr_dispatcher_core/planning/conflict_based_search.hpp"
#include "amr_dispatcher_core/planning/prioritized_planning.hpp"

namespace amr_dispatcher_core::planning {

enum class MAPFSolverMode {
  kAuto,            // 智能自适应：车少用 CBS，超限或车多自动降级至 Prioritized
  kCBSOnly,         // 严格仅用 CBS
  kPrioritizedOnly  // 仅用 Prioritized Planning (PBS)
};

struct MAPFPolicy {
  MAPFSolverMode mode = MAPFSolverMode::kAuto;
  int cbs_agent_threshold = 6;            // 当 agent 数量 > 6 时，kAuto 模式直接采用 Prioritized
  bool fallback_to_prioritized = true;    // 当 CBS 搜索超限或失败时，是否允许自动降级到 Prioritized
  int cbs_max_ct_nodes = 128;             // CBS 搜索预算
  PrioritizedOptions prioritized_options; // Prioritized 规划选项
};

struct MAPFUnifiedRequest {
  std::vector<CBSAgentSpec> agents;
  double travel_speed_mps = 1.0;
  double time_step_duration_sec = 1.0;

  std::vector<VertexConstraint> initial_vertex_constraints;
  std::vector<EdgeConstraint> initial_edge_constraints;

  int max_time_steps = 128;
  int low_level_max_nodes = 2048;

  MAPFPolicy policy;
};

struct MAPFUnifiedResult {
  bool success = false;
  std::unordered_map<std::string, TimedPath> solution;
  double sum_of_costs = 0.0;
  int makespan = 0;
  bool verified_conflict_free = false;
  std::string solver_used;     // "cbs" | "prioritized" | "cbs->prioritized"
  std::string fallback_reason; // 若发生降级，记录原因
  std::string message;
};

// 纯函数：统一 MAPF 协同求解器
MAPFUnifiedResult SolveMAPF(
    const MAPFUnifiedRequest& request,
    const amr_dispatcher_core::catalog::StationCatalog& catalog);

}  // namespace amr_dispatcher_core::planning
