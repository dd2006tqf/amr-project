#include "amr_dispatcher_core/workflow/time_space_planning.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace amr_dispatcher_core::workflow {

namespace {

using Clock = std::chrono::steady_clock;

double Seconds(const Clock::duration& d) {
  return std::chrono::duration_cast<std::chrono::duration<double>>(d).count();
}

Clock::duration FromSeconds(double s) {
  return std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(s));
}

// 冲突检测：与已有占用比对。返回最早可进入时刻（无冲突时为 enter 本身）。
struct ConflictCheck {
  bool conflict = false;
  std::string resource_id;
  std::string conflicting_mission_id;
  std::string reason;
  double suggested_delay_sec = 0.0;
};

ConflictCheck CheckResource(const std::string& resource_id, const Clock::time_point& enter,
                            const Clock::time_point& exit, const std::string& mission_id,
                            const TimeSpaceOccupancy& occupancy, double min_headway_sec) {
  ConflictCheck result;
  const auto it = occupancy.by_resource.find(resource_id);
  if (it == occupancy.by_resource.end()) {
    return result;
  }
  for (const auto& entry : it->second) {
    if (entry.mission_id == mission_id) {
      continue;  // 自己的旧预约不算冲突
    }
    // 区间重叠
    if (enter < entry.exit && entry.enter < exit) {
      result.conflict = true;
      result.resource_id = resource_id;
      result.conflicting_mission_id = entry.mission_id;
      result.reason = "overlap";
      const double delay = Seconds(entry.exit - enter) + min_headway_sec;
      result.suggested_delay_sec = std::max(result.suggested_delay_sec, delay);
      continue;
    }
    // 安全间隔不足：后车进入时刻早于前车离开 + headway
    if (enter >= entry.exit) {
      const double gap = Seconds(enter - entry.exit);
      if (gap < min_headway_sec) {
        result.conflict = true;
        result.resource_id = resource_id;
        result.conflicting_mission_id = entry.mission_id;
        result.reason = "headway";
        const double delay = min_headway_sec - gap;
        result.suggested_delay_sec = std::max(result.suggested_delay_sec, delay);
      }
    }
  }
  return result;
}

}  // namespace

TimeSpacePlan PlanTimeSpaceReservation(const TimeSpaceCandidate& candidate,
                                       const TimeSpaceOccupancy& occupancy,
                                       const MotionTimingConfig& config) {
  TimeSpacePlan plan;
  if (candidate.path_node_ids.size() < 2) {
    plan.reject_reason = "path needs at least 2 nodes";
    return plan;
  }
  if (!candidate.edge_id_builder || !candidate.node_id_builder) {
    plan.reject_reason = "resource id builders are required";
    return plan;
  }
  const std::size_t segments = candidate.path_node_ids.size() - 1;
  if (!candidate.segment_lengths_m.empty() &&
      candidate.segment_lengths_m.size() != segments) {
    plan.reject_reason = "segment_lengths_m size mismatch with path";
    return plan;
  }
  const double speed = config.cruise_speed_mps > 1e-6 ? config.cruise_speed_mps : 1.0;

  // 1) 逐资源区间（几何 -> 时间）
  ResourceTimeWindowSet base_windows;
  base_windows.reserve(segments * 2 + 1);
  Clock::time_point cursor = candidate.earliest_start;
  for (std::size_t i = 0; i < segments; ++i) {
    const double length_m =
        candidate.segment_lengths_m.empty() ? 0.0 : candidate.segment_lengths_m[i];
    const double travel_sec = length_m / speed;
    // 起步/刹停 + 转弯附加 + 尺寸安全余量换算成时间
    const double overhead_sec = config.accel_overhead_sec + config.turn_overhead_sec +
                                config.safety_margin_m / speed;
    const double occupy_sec = travel_sec + overhead_sec;
    const auto& from = candidate.path_node_ids[i];
    const auto& to = candidate.path_node_ids[i + 1];

    // 边（通道）：进入 -> 驶出
    ResourceTimeWindow edge;
    edge.resource_id = candidate.edge_id_builder(from, to);
    edge.enter = cursor;
    edge.exit = cursor + FromSeconds(occupy_sec);
    edge.is_intersection = false;
    base_windows.push_back(edge);
    cursor = edge.exit;

    // 节点（路口）：车辆在路口内的完整占用区间——与相邻两段重叠，
    // 冲突判定因此覆盖"交叉/对向在路口内部"的物理冲突。
    ResourceTimeWindow node;
    node.resource_id = candidate.node_id_builder(to);
    node.enter = cursor;
    node.exit = cursor + FromSeconds(occupy_sec);
    node.is_intersection = true;
    base_windows.push_back(node);
    cursor = node.exit;
  }
  // 起点节点区间
  {
    ResourceTimeWindow start;
    start.resource_id = candidate.node_id_builder(candidate.path_node_ids.front());
    start.enter = candidate.earliest_start;
    start.exit = candidate.earliest_start + FromSeconds(config.accel_overhead_sec);
    start.is_intersection = true;
    base_windows.push_back(start);
  }

  // 2) 冲突检测 + 整体后移（最多重试若干轮）
  const int max_attempts = 8;
  double total_delay = 0.0;
  ResourceTimeWindowSet windows = base_windows;
  for (int attempt = 0; attempt < max_attempts; ++attempt) {
    bool any_conflict = false;
    ConflictCheck worst;
    std::vector<TimeSpaceConflict> attempt_conflicts;
    for (const auto& window : windows) {
      const auto conflict = CheckResource(window.resource_id, window.enter, window.exit,
                                          candidate.mission_id, occupancy,
                                          config.min_headway_sec);
      if (conflict.conflict) {
        any_conflict = true;
        TimeSpaceConflict record;
        record.has_conflict = true;
        record.resource_id = conflict.resource_id;
        record.conflicting_mission_id = conflict.conflicting_mission_id;
        record.reason = conflict.reason;
        record.suggested_delay_sec = conflict.suggested_delay_sec;
        attempt_conflicts.push_back(record);
        if (conflict.suggested_delay_sec > worst.suggested_delay_sec) {
          worst = conflict;
        }
      }
    }
    if (!any_conflict) {
      plan.feasible = true;
      plan.windows = windows;
      plan.total_delay_sec = total_delay;
      return plan;
    }
    if (worst.suggested_delay_sec <= 0.0) {
      break;
    }
    // 记录本轮全部冲突（诊断用），不只记最严重的一个
    for (const auto& record : attempt_conflicts) {
      plan.conflicts.push_back(record);
    }
    // 整体后移：所有资源区间同步平移
    const auto shift = FromSeconds(worst.suggested_delay_sec);
    for (auto& window : windows) {
      window.enter += shift;
      window.exit += shift;
    }
    total_delay += worst.suggested_delay_sec;
  }

  plan.feasible = false;
  plan.reject_reason = "time-space conflict could not be resolved within " +
                       std::to_string(max_attempts) + " shift attempts";
  plan.total_delay_sec = total_delay;
  return plan;
}

void CommitTimeSpaceOccupancy(TimeSpaceOccupancy* occupancy, const std::string& mission_id,
                               const ResourceTimeWindowSet& windows) {
  if (occupancy == nullptr) {
    return;
  }
  for (const auto& window : windows) {
    TimeSpaceOccupancy::Entry entry;
    entry.mission_id = mission_id;
    entry.enter = window.enter;
    entry.exit = window.exit;
    entry.is_intersection = window.is_intersection;
    occupancy->by_resource[window.resource_id].push_back(entry);
  }
}

bool ReleaseTimeSpaceResources(TimeSpaceOccupancy* occupancy, const std::string& mission_id,
                               const std::vector<std::string>& resource_ids,
                               std::vector<std::string>* released_resource_ids) {
  if (occupancy == nullptr) {
    return false;
  }
  bool any = false;
  for (const auto& resource_id : resource_ids) {
    const auto it = occupancy->by_resource.find(resource_id);
    if (it == occupancy->by_resource.end()) {
      continue;
    }
    auto& entries = it->second;
    const std::size_t before = entries.size();
    entries.erase(std::remove_if(entries.begin(), entries.end(),
                                  [&mission_id](const TimeSpaceOccupancy::Entry& e) {
                                    return e.mission_id == mission_id;
                                  }),
                  entries.end());
    if (entries.size() != before) {
      any = true;
      if (released_resource_ids != nullptr) {
        released_resource_ids->push_back(resource_id);
      }
    }
    if (entries.empty()) {
      occupancy->by_resource.erase(it);
    }
  }
  return any;
}

bool HoldsTimeSpaceResource(const TimeSpaceOccupancy& occupancy, const std::string& mission_id,
                             const std::string& resource_id) {
  const auto it = occupancy.by_resource.find(resource_id);
  if (it == occupancy.by_resource.end()) {
    return false;
  }
  for (const auto& entry : it->second) {
    if (entry.mission_id == mission_id) {
      return true;
    }
  }
  return false;
}

bool ExtendTimeSpaceOccupancy(TimeSpaceOccupancy* occupancy, const std::string& mission_id,
                               const Clock::time_point& new_exit, std::string* message) {
  if (occupancy == nullptr) {
    if (message != nullptr) *message = "occupancy is null";
    return false;
  }
  // 先验证延长后是否与他车窗口冲突（不满足则不改动占用表）。
  for (const auto& [resource_id, entries] : occupancy->by_resource) {
    for (const auto& entry : entries) {
      if (entry.mission_id != mission_id) {
        continue;
      }
      if (new_exit <= entry.exit) {
        continue;  // 未延长
      }
      // 与该资源上其他车辆的窗口比对
      for (const auto& other : entries) {
        if (other.mission_id == mission_id) continue;
        if (other.enter < new_exit && entry.enter < other.exit) {
          if (message != nullptr) {
            *message = "extension conflicts with mission " + other.mission_id + " on " +
                       resource_id;
          }
          return false;
        }
      }
    }
  }
  // 无冲突：执行延长
  for (auto& [resource_id, entries] : occupancy->by_resource) {
    for (auto& entry : entries) {
      if (entry.mission_id == mission_id && new_exit > entry.exit) {
        entry.exit = new_exit;
      }
    }
  }
  if (message != nullptr) *message = "time window extended";
  return true;
}

}  // namespace amr_dispatcher_core::workflow
