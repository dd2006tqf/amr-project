#include "amr_dispatcher_core/workflow/traffic_planning.hpp"

#include <algorithm>
#include <cstddef>
#include <sstream>
#include <unordered_map>
#include <unordered_set>

#include "amr_dispatcher_core/dispatcher/traffic_reservation.hpp"

namespace amr_dispatcher_core::workflow {

namespace dispatcher = amr_dispatcher_core::dispatcher;

// A10：路由锁 id 单一实现收敛于 dispatcher::ResourceTable —— workflow 侧改为薄封装，
// 删除逐行重复体，保证生成/解析两侧永远同源（跨命名空间 round-trip 测试锁定）。

std::string RouteLockId(const std::string& from, const std::string& to) {
  return dispatcher::ResourceTable::RouteEdgeId(from, to);
}

std::string RouteNodeLockId(const std::string& station_id) {
  return dispatcher::ResourceTable::RouteNodeId(station_id);
}

std::optional<std::pair<std::string, std::string>> ParseRouteLock(
    const std::string& resource_id) {
  return dispatcher::ResourceTable::ParseRouteEdge(resource_id);
}

bool IsTrafficBlockOwner(const std::string& owner_id) {
  return dispatcher::ResourceTable::IsTrafficBlockOwner(owner_id);
}

std::string TrafficBlockOwnerId(const std::string& reason) {
  return "traffic_block:" + reason;
}

TrafficReservationDecision PlanTrafficBlock(
    const TrafficBlockRequest& request, const RouteLockMap& route_locks) {
  TrafficReservationDecision decision;
  decision.resource_id = RouteLockId(request.from_station, request.to_station);
  if (!request.stations_known) {
    decision.message = "unknown station in route block request: " + request.from_station +
                       " -> " + request.to_station;
    return decision;
  }
  if (!request.route_exists) {
    decision.message =
        "no enabled station route to block: " + request.from_station + " -> " + request.to_station;
    return decision;
  }
  const auto existing = route_locks.find(decision.resource_id);
  if (existing != route_locks.end()) {
    decision.message =
        "route resource already reserved: " + decision.resource_id + " by " + existing->second;
    return decision;
  }
  decision.success = true;
  decision.mutate_state = true;
  decision.owner_id = TrafficBlockOwnerId(request.reason);
  decision.message = "blocked route " + request.from_station + " -> " + request.to_station;
  return decision;
}

TrafficReservationDecision PlanTrafficUnblock(
    const std::string& from_station, const std::string& to_station,
    const RouteLockMap& route_locks) {
  TrafficReservationDecision decision;
  decision.resource_id = RouteLockId(from_station, to_station);
  const auto lock = route_locks.find(decision.resource_id);
  if (lock == route_locks.end()) {
    decision.message = "route resource not found: " + decision.resource_id;
    return decision;
  }
  decision.owner_id = lock->second;
  if (!IsTrafficBlockOwner(lock->second)) {
    decision.message = "route resource is reserved by mission: " + lock->second;
    return decision;
  }
  decision.success = true;
  decision.mutate_state = true;
  decision.message = "unblocked route " + from_station + " -> " + to_station;
  return decision;
}

TrafficClearDecision PlanTrafficReservationClear(
    const std::string& resource_id, const RouteLockMap& route_locks) {
  TrafficClearDecision decision;
  decision.resource_id = resource_id;
  const auto lock = route_locks.find(resource_id);
  if (lock == route_locks.end()) {
    decision.message = "traffic reservation not found: " + resource_id;
    return decision;
  }
  decision.success = true;
  decision.owner_id = lock->second;
  if (IsTrafficBlockOwner(lock->second)) {
    decision.erase_resource = true;
    decision.message = "cleared blocked route " + resource_id;
    return decision;
  }
  decision.release_mission_id = lock->second;
  decision.message = "cleared mission route reservation " + resource_id;
  return decision;
}

TrafficDeadlockDecision PlanTrafficDeadlockDetection(const RouteLockMap& route_locks) {
  TrafficDeadlockDecision decision;
  std::unordered_map<std::string, std::vector<std::string>> station_to_missions;
  std::unordered_map<std::string, std::vector<std::string>> station_to_resources;
  for (const auto& [resource_id, owner_id] : route_locks) {
    if (IsTrafficBlockOwner(owner_id)) {
      continue;
    }
    const auto lock = ParseRouteLock(resource_id);
    if (!lock.has_value()) {
      continue;
    }
    station_to_missions[lock->first].push_back(owner_id);
    station_to_missions[lock->second].push_back(owner_id);
    station_to_resources[lock->first].push_back(resource_id);
    station_to_resources[lock->second].push_back(resource_id);
  }

  std::unordered_set<std::string> conflict_missions;
  std::unordered_set<std::string> conflict_resources;
  std::unordered_set<std::string> conflict_stations;
  for (const auto& [station_id, mission_ids] : station_to_missions) {
    std::unordered_set<std::string> unique_missions(mission_ids.begin(), mission_ids.end());
    if (unique_missions.size() < 2U) {
      continue;
    }
    conflict_stations.insert(station_id);
    std::vector<std::string> ordered_missions(unique_missions.begin(), unique_missions.end());
    std::sort(ordered_missions.begin(), ordered_missions.end());
    for (const auto& mission_id : ordered_missions) {
      conflict_missions.insert(mission_id);
    }
    const auto resource_it = station_to_resources.find(station_id);
    if (resource_it != station_to_resources.end()) {
      for (const auto& resource_id : resource_it->second) {
        conflict_resources.insert(resource_id);
      }
    }
    std::ostringstream desc;
    desc << "station " << station_id << " shared by missions ";
    for (std::size_t index = 0; index < ordered_missions.size(); ++index) {
      if (index > 0) {
        desc << ", ";
      }
      desc << ordered_missions[index];
    }
    decision.descriptions.push_back(desc.str());
  }

  decision.deadlocked = !conflict_stations.empty();
  decision.mission_ids.assign(conflict_missions.begin(), conflict_missions.end());
  std::sort(decision.mission_ids.begin(), decision.mission_ids.end());
  decision.conflict_resource_ids.assign(conflict_resources.begin(), conflict_resources.end());
  std::sort(decision.conflict_resource_ids.begin(), decision.conflict_resource_ids.end());
  decision.conflict_station_ids.assign(conflict_stations.begin(), conflict_stations.end());
  std::sort(decision.conflict_station_ids.begin(), decision.conflict_station_ids.end());
  if (decision.deadlocked) {
    decision.message = "traffic deadlock candidates detected";
  }
  return decision;
}

std::vector<std::string> BuildRouteLockIds(
    const std::vector<std::string>& station_path, const bool include_intersection_locks) {
  // A10：薄封装 —— 与 dispatcher::ResourceTable::BuildLockIds 同一实现
  return dispatcher::ResourceTable::BuildLockIds(station_path, include_intersection_locks);
}

std::vector<std::string> PlanMissionRouteLocks(
    const amr_dispatcher_core::catalog::StationCatalog& catalog,
    const std::string& from_station, const std::string& to_station) {
  if (to_station.empty()) {
    // 目的站缺失：不预留任何锁，派发链随后按未知站点判负（保持现网行为）
    return {};
  }
  if (from_station.empty()) {
    // 无取货站（车辆当前位姿作起点）：只锁目的站节点，无法预知途经路段
    return {RouteNodeLockId(to_station)};
  }
  const auto path = amr_dispatcher_core::catalog::FindRoutePath(catalog, from_station, to_station);
  if (!path.has_value()) {
    // 目录断开/站点未知 -> 空锁集，照旧无锁派发
    return {};
  }
  return BuildRouteLockIds(*path, /*include_intersection_locks=*/true);
}

bool ReserveRouteLocksForMission(
    RouteLockMap& route_locks, RouteLockReservationMap& route_locks_by_mission,
    const std::string& mission_id, const std::vector<std::string>& route_lock_ids,
    std::string* message) {
  for (const auto& lock_id : route_lock_ids) {
    const auto existing = route_locks.find(lock_id);
    if (existing != route_locks.end() && existing->second != mission_id) {
      if (message != nullptr) {
        *message = "route resource locked: " + lock_id + " by " + existing->second;
      }
      return false;
    }
  }
  for (const auto& lock_id : route_lock_ids) {
    route_locks[lock_id] = mission_id;
  }
  route_locks_by_mission[mission_id] = RouteLockReservation{mission_id, route_lock_ids};
  if (message != nullptr) {
    *message = "reserved " + std::to_string(route_lock_ids.size()) + " route lock(s)";
  }
  return true;
}

bool ReleaseRouteLocksForMission(
    RouteLockMap& route_locks, RouteLockReservationMap& route_locks_by_mission,
    const std::string& mission_id) {
  bool released = false;
  const auto reservation = route_locks_by_mission.find(mission_id);
  if (reservation != route_locks_by_mission.end()) {
    for (const auto& lock_id : reservation->second.route_lock_ids) {
      const auto lock = route_locks.find(lock_id);
      if (lock != route_locks.end() && lock->second == mission_id) {
        route_locks.erase(lock);
        released = true;
      }
    }
    route_locks_by_mission.erase(reservation);
  }
  for (auto lock = route_locks.begin(); lock != route_locks.end();) {
    if (lock->second == mission_id) {
      lock = route_locks.erase(lock);
      released = true;
    } else {
      ++lock;
    }
  }
  return released;
}

}  // namespace amr_dispatcher_core::workflow
