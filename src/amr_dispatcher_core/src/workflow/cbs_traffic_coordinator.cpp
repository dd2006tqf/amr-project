#include "amr_dispatcher_core/workflow/cbs_traffic_coordinator.hpp"

#include <cmath>

namespace amr_dispatcher_core::workflow {

namespace {

std::string DefaultEdgeIdBuilder(const std::string& u, const std::string& v) {
  if (u < v) {
    return "route_edge:" + u + "__" + v;
  }
  return "route_edge:" + v + "__" + u;
}

std::string DefaultNodeIdBuilder(const std::string& node) {
  return "route_node:" + node;
}

}  // namespace

ResourceTimeWindowSet ConvertTimedPathToTimeWindows(
    const amr_dispatcher_core::planning::TimedPath& timed_path,
    std::chrono::steady_clock::time_point base_time,
    double time_step_duration_sec,
    const std::function<std::string(const std::string&, const std::string&)>& edge_id_builder,
    const std::function<std::string(const std::string&)>& node_id_builder) {
  ResourceTimeWindowSet windows;
  if (timed_path.empty()) return windows;

  const auto edge_builder = edge_id_builder ? edge_id_builder : DefaultEdgeIdBuilder;
  const auto node_builder = node_id_builder ? node_id_builder : DefaultNodeIdBuilder;
  const double dt = (time_step_duration_sec > 0.05) ? time_step_duration_sec : 1.0;

  for (std::size_t i = 0; i < timed_path.size(); ++i) {
    const auto& pt = timed_path[i];
    // 节点窗口
    ResourceTimeWindow nw;
    nw.resource_id = node_builder(pt.station_id);
    nw.is_intersection = true;
    nw.enter = base_time + std::chrono::milliseconds(static_cast<int64_t>(pt.time_step * dt * 1000));
    // 节点至少占用一个时步或至下一时步
    int next_step = pt.time_step + 1;
    if (i + 1 < timed_path.size() && timed_path[i + 1].station_id == pt.station_id) {
      next_step = timed_path[i + 1].time_step;
    }
    nw.exit = base_time + std::chrono::milliseconds(static_cast<int64_t>(next_step * dt * 1000));
    windows.push_back(nw);

    // 边窗口 (若前往下一不同站点)
    if (i + 1 < timed_path.size() && timed_path[i + 1].station_id != pt.station_id) {
      const auto& next_pt = timed_path[i + 1];
      ResourceTimeWindow ew;
      ew.resource_id = edge_builder(pt.station_id, next_pt.station_id);
      ew.is_intersection = false;
      ew.enter = base_time + std::chrono::milliseconds(static_cast<int64_t>(pt.time_step * dt * 1000));
      ew.exit = base_time + std::chrono::milliseconds(static_cast<int64_t>(next_pt.time_step * dt * 1000));
      windows.push_back(ew);
    }
  }

  return windows;
}

CBSCoordinationResult PlanCBSMultiMissionCoordination(
    const std::vector<CBSMissionRequest>& requests,
    const amr_dispatcher_core::catalog::StationCatalog& catalog,
    const TimeSpaceOccupancy& existing_occupancy,
    const MotionTimingConfig& timing_config,
    std::chrono::steady_clock::time_point now_time,
    const amr_dispatcher_core::planning::MAPFPolicy& policy) {
  CBSCoordinationResult result;
  if (requests.empty()) {
    result.all_solved = true;
    result.message = "no mission requests";
    return result;
  }

  amr_dispatcher_core::planning::MAPFUnifiedRequest mapf_req;
  mapf_req.travel_speed_mps = timing_config.cruise_speed_mps;
  mapf_req.time_step_duration_sec = 1.0;
  mapf_req.policy = policy;

  const double dt = mapf_req.time_step_duration_sec;

  // 1. 将现存在途占用转换为初始时空约束 (Fixed Traffic Constraints)
  for (const auto& [res_id, entries] : existing_occupancy.by_resource) {
    for (const auto& entry : entries) {
      if (entry.exit <= now_time) continue; // 已过去的占用忽略

      const auto start_offset_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
          (entry.enter > now_time) ? (entry.enter - now_time) : std::chrono::milliseconds(0)).count();
      const auto end_offset_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
          entry.exit - now_time).count();

      const int t_start = static_cast<int>(start_offset_ms / (dt * 1000));
      const int t_end = static_cast<int>(std::ceil(end_offset_ms / (dt * 1000)));

      if (res_id.rfind("route_node:", 0) == 0) {
        const std::string node = res_id.substr(11);
        for (int t = t_start; t <= t_end; ++t) {
          amr_dispatcher_core::planning::VertexConstraint vc;
          vc.agent_id = ""; // 对所有新规划 agent 生效
          vc.station_id = node;
          vc.time_step = t;
          mapf_req.initial_vertex_constraints.push_back(vc);
        }
      } else if (res_id.rfind("route_edge:", 0) == 0) {
        const std::string edge_nodes = res_id.substr(11);
        auto pos = edge_nodes.find("__");
        if (pos != std::string::npos) {
          amr_dispatcher_core::planning::EdgeConstraint ec;
          ec.agent_id = "";
          ec.from_station = edge_nodes.substr(0, pos);
          ec.to_station = edge_nodes.substr(pos + 2);
          ec.t_start = t_start;
          ec.t_end = t_end;
          mapf_req.initial_edge_constraints.push_back(ec);
        }
      }
    }
  }

  // 2. 组装待规划智能体列表
  for (const auto& req : requests) {
    amr_dispatcher_core::planning::CBSAgentSpec spec;
    spec.agent_id = req.mission_id;
    spec.start_station = req.start_station;
    spec.goal_station = req.goal_station;

    const auto start_delay_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        (req.earliest_start > now_time) ? (req.earliest_start - now_time) : std::chrono::milliseconds(0)).count();
    spec.start_time_step = static_cast<int>(start_delay_ms / (dt * 1000));
    spec.start_time_sec = start_delay_ms / 1000.0;

    mapf_req.agents.push_back(spec);
  }

  // 3. 执行统一多智能体求解 (支持 CBS 及超限自动降级至 Prioritized Planning)
  auto mapf_res = amr_dispatcher_core::planning::SolveMAPF(mapf_req, catalog);
  if (!mapf_res.success) {
    result.all_solved = false;
    result.message = "MAPF failed (" + mapf_res.solver_used + "): " + mapf_res.message;
    return result;
  }

  // 4. 将规划解回填并转换为 ResourceTimeWindowSet
  result.all_solved = true;
  result.message = "coordinated by " + mapf_res.solver_used;

  for (const auto& req : requests) {
    CBSMissionPlan mplan;
    mplan.mission_id = req.mission_id;

    auto it = mapf_res.solution.find(req.mission_id);
    if (it != mapf_res.solution.end() && !it->second.empty()) {
      mplan.success = true;
      // 提取拓扑节点路径
      for (const auto& pt : it->second) {
        if (mplan.path_nodes.empty() || mplan.path_nodes.back() != pt.station_id) {
          mplan.path_nodes.push_back(pt.station_id);
        }
      }
      // 转换时空窗口
      mplan.time_windows = ConvertTimedPathToTimeWindows(
          it->second, now_time, dt, DefaultEdgeIdBuilder, DefaultNodeIdBuilder);
    } else {
      mplan.success = false;
      mplan.message = "no solution in MAPF result";
      result.all_solved = false;
    }

    result.mission_plans[req.mission_id] = std::move(mplan);
  }

  return result;
}

}  // namespace amr_dispatcher_core::workflow
