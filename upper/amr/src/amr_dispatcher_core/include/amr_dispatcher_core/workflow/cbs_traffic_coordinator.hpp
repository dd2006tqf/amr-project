// CBS 多车协同交通协调器 (CBS Traffic Coordinator)。
//
// 对应 Phase 9 (P2)：
// 连接算法层 (CBS / SpaceTimeAStar) 与调度器业务层 (ResourceTimeWindowSet / TimeSpaceOccupancy)：
// 1. 将待派发的候选任务转化为 CBS 请求；
// 2. 将在途车辆已占用的时间窗（Fixed Traffic）转化为初始 Vertex/Edge 约束；
// 3. 将 CBS 输出的 TimedPath 转换为调度系统标准的 ResourceTimeWindowSet 与节点锁/边锁。
#pragma once

#include <chrono>
#include <string>
#include <unordered_map>
#include <vector>

#include "amr_dispatcher_core/catalog/station_catalog.hpp"
#include "amr_dispatcher_core/planning/conflict_based_search.hpp"
#include "amr_dispatcher_core/planning/mapf_solver.hpp"
#include "amr_dispatcher_core/workflow/time_space_planning.hpp"

namespace amr_dispatcher_core::workflow {

// 单任务 CBS 协调输入
struct CBSMissionRequest {
  std::string mission_id;
  std::string vehicle_id;
  std::string start_station;
  std::string goal_station;
  std::chrono::steady_clock::time_point earliest_start;
};

// CBS 交通协调输出方案
struct CBSMissionPlan {
  std::string mission_id;
  bool success = false;
  std::vector<std::string> path_nodes; // 拓扑站点序列
  ResourceTimeWindowSet time_windows;  // 转换后的标准时空窗口集合
  std::string message;
};

struct CBSCoordinationResult {
  bool all_solved = false;
  std::unordered_map<std::string, CBSMissionPlan> mission_plans;
  std::string message;
};

// 纯函数：将 MAPF 规划结果转为 ResourceTimeWindowSet
//
// `timing` 为空时窗口严格等于离散时步区间（历史行为，供单测精确断言）；
// 传入 `MotionTimingConfig` 时，对每个窗口的 **exit** 做安全膨胀：
//   起步/刹停附加 (accel_overhead_sec) + 尺寸安全余量 (safety_margin_m / 巡航速度)
//   + 最小安全间隔 (min_headway_sec)
// 这样 `traffic_mode: cbs` 提交的窗口与 `time_space` 模式使用同一套时间预算，
// 不会因为只按整数 tick 记时而对后车偏乐观（ResourceTable 只查区间重叠、不查 headway）。
ResourceTimeWindowSet ConvertTimedPathToTimeWindows(
    const amr_dispatcher_core::planning::TimedPath& timed_path,
    std::chrono::steady_clock::time_point base_time,
    double time_step_duration_sec,
    const std::function<std::string(const std::string&, const std::string&)>& edge_id_builder,
    const std::function<std::string(const std::string&)>& node_id_builder,
    const MotionTimingConfig* timing = nullptr);

// 纯函数：执行 CBS 多车协同规划并输出各任务时空窗方案 (默认使用 Auto 自适应策略)
CBSCoordinationResult PlanCBSMultiMissionCoordination(
    const std::vector<CBSMissionRequest>& requests,
    const amr_dispatcher_core::catalog::StationCatalog& catalog,
    const TimeSpaceOccupancy& existing_occupancy,
    const MotionTimingConfig& timing_config,
    std::chrono::steady_clock::time_point now_time,
    const amr_dispatcher_core::planning::MAPFPolicy& policy = amr_dispatcher_core::planning::MAPFPolicy{});

}  // namespace amr_dispatcher_core::workflow
