#include "amr_dispatcher_core/dispatcher/traffic_reservation.hpp"

#include <algorithm>
#include <sstream>
#include <unordered_set>

namespace amr_dispatcher_core::dispatcher {
namespace {

bool StartsWith(const std::string& value, const std::string& prefix) {
  return value.rfind(prefix, 0) == 0;
}

constexpr const char* kBlockPrefix = "traffic_block:";

// A7：剪除已过期的时空预约（end < now），防止历史条目在表中无限累积
void PruneExpiredTimeSpace(
    std::unordered_map<std::string, std::vector<TimeSpaceReservation>>& reservations,
    const std::chrono::steady_clock::time_point now) {
  for (auto it = reservations.begin(); it != reservations.end();) {
    auto& list = it->second;
    list.erase(std::remove_if(list.begin(), list.end(),
                              [now](const TimeSpaceReservation& reservation) {
                                return reservation.interval.end < now;
                              }),
               list.end());
    if (list.empty()) {
      it = reservations.erase(it);
    } else {
      ++it;
    }
  }
}

}  // namespace

std::string ResourceTable::RouteEdgeId(const std::string& from, const std::string& to) {
  return from <= to ? "route_edge:" + from + "__" + to : "route_edge:" + to + "__" + from;
}

std::string ResourceTable::RouteNodeId(const std::string& station_id) {
  return "route_node:" + station_id;
}

std::optional<std::pair<std::string, std::string>> ResourceTable::ParseRouteEdge(
    const std::string& resource_id) {
  const std::string prefix = "route_edge:";
  if (!StartsWith(resource_id, prefix)) {
    return std::nullopt;
  }
  const auto separator = resource_id.find("__", prefix.size());
  if (separator == std::string::npos) {
    return std::nullopt;
  }
  const auto first = resource_id.substr(prefix.size(), separator - prefix.size());
  const auto second = resource_id.substr(separator + 2);
  if (first.empty() || second.empty()) {
    return std::nullopt;
  }
  return std::make_pair(first, second);
}

bool ResourceTable::IsTrafficBlockOwner(const std::string& owner_id) {
  return StartsWith(owner_id, kBlockPrefix);
}

std::vector<std::string> ResourceTable::BuildLockIds(
    const std::vector<std::string>& station_path, const bool include_intersection_locks) {
  std::vector<std::string> lock_ids;
  std::unordered_set<std::string> seen;  // 保序去重：重复节点/边只保留首次出现
  const auto append_once = [&lock_ids, &seen](const std::string& lock_id) {
    if (seen.insert(lock_id).second) {
      lock_ids.push_back(lock_id);
    }
  };
  if (include_intersection_locks) {
    for (const auto& station_id : station_path) {
      append_once(RouteNodeId(station_id));
    }
  }
  for (std::size_t index = 1; index < station_path.size(); ++index) {
    append_once(RouteEdgeId(station_path[index - 1], station_path[index]));
  }
  return lock_ids;
}

ResourceTable::ReserveResult ResourceTable::Reserve(
    const std::string& mission_id, const std::vector<std::string>& resource_ids) {
  ReserveResult result;
  if (mission_id.empty()) {
    result.message = "mission id must not be empty";
    return result;
  }
  // A7：入口先剪除过期时空预约，表大小不随历史条目累积
  PruneExpiredTimeSpace(time_space_reservations_, std::chrono::steady_clock::now());
  for (const auto& resource_id : resource_ids) {
    const auto existing = locks_.find(resource_id);
    if (existing != locks_.end() && existing->second != mission_id) {
      std::ostringstream oss;
      oss << "resource locked: " << resource_id << " by " << existing->second;
      result.message = oss.str();
      return result;  // 未落任何锁，原子性成立
    }
  }
  // A7：写入前过滤同任务已持有锁与列表内重复，newly_reserved 只记真正新增
  std::unordered_set<std::string> counted;
  for (const auto& resource_id : resource_ids) {
    if (!counted.insert(resource_id).second) {
      continue;  // 列表内重复不重复计入
    }
    const auto existing = locks_.find(resource_id);
    if (existing != locks_.end() && existing->second == mission_id) {
      continue;  // 同任务已持有，不算新增
    }
    locks_[resource_id] = mission_id;
    result.newly_reserved.push_back(resource_id);
  }
  std::ostringstream oss;
  oss << "reserved " << result.newly_reserved.size() << " resource(s)";
  result.success = true;
  result.message = oss.str();
  return result;
}

bool ResourceTable::ReserveTimeSpace(
    const std::string& mission_id,
    const std::string& resource_id,
    std::chrono::steady_clock::time_point start,
    std::chrono::steady_clock::time_point end,
    std::string* message) {
  if (start >= end) {
    if (message) *message = "invalid time interval: start >= end";
    return false;
  }

  // M3：入口先剪除过期时空预约，与 Reserve/Release 保持一致——
  // 否则已结束的时段仍会参与冲突判断，误拒时间重叠的回溯候选。
  PruneExpiredTimeSpace(time_space_reservations_, std::chrono::steady_clock::now());

  // 1. 检查是否存在永久排他锁（人工封路等）
  if (Locked(resource_id) && Owner(resource_id) != mission_id) {
    if (message) *message = "resource statically locked by: " + *Owner(resource_id);
    return false;
  }

  // 2. 检查该资源在目标时间区间内是否与其他任务的预约时段重叠
  TimeInterval candidate_interval{start, end};
  auto& reservations = time_space_reservations_[resource_id];
  for (const auto& res : reservations) {
    if (res.mission_id != mission_id && res.interval.OverlapsWith(candidate_interval)) {
      if (message) {
        *message = "time-space contention on " + resource_id + " with " + res.mission_id;
      }
      return false;
    }
  }

  // 3. 无时空冲突，落入预约表
  reservations.push_back(TimeSpaceReservation{mission_id, resource_id, candidate_interval});
  if (message) *message = "time-space reservation confirmed";
  return true;
}

bool ResourceTable::ReserveTimeSpaceBatch(
    const std::string& mission_id,
    const std::vector<std::pair<std::string, TimeInterval>>& resource_intervals,
    std::string* message) {
  // 全有或全无：先整体校验（区间合法 + 无静态排他锁 + 无时空冲突），
  // 全部通过后再一次性落表——中途失败不留半套预约。
  std::vector<std::pair<std::string, TimeInterval>> pending;
  pending.reserve(resource_intervals.size());
  for (const auto& [resource_id, interval] : resource_intervals) {
    if (interval.start >= interval.end) {
      if (message) *message = "invalid time interval on " + resource_id + ": start >= end";
      return false;
    }
    // 静态排他锁（人工封路 / 其它任务的静态锁）不可被时空预约覆盖
    const auto static_owner = Owner(resource_id);
    if (static_owner.has_value() && *static_owner != mission_id) {
      if (message) {
        *message = "resource statically locked by: " + *static_owner + " (" + resource_id + ")";
      }
      return false;
    }
    pending.emplace_back(resource_id, interval);
  }
  // 时空冲突校验（对**同一批次内**的重复资源也要查：先查已落表的，
  // 再把本批次内部的重叠也一并检查）
  for (std::size_t i = 0; i < pending.size(); ++i) {
    for (std::size_t j = i + 1; j < pending.size(); ++j) {
      if (pending[i].first == pending[j].first &&
          pending[i].second.OverlapsWith(pending[j].second)) {
        if (message) {
          *message = "intra-batch time-space overlap on " + pending[i].first;
        }
        return false;
      }
    }
  }
  PruneExpiredTimeSpace(time_space_reservations_, std::chrono::steady_clock::now());
  for (const auto& [resource_id, interval] : pending) {
    const auto it = time_space_reservations_.find(resource_id);
    if (it == time_space_reservations_.end()) {
      continue;
    }
    for (const auto& res : it->second) {
      if (res.mission_id != mission_id && res.interval.OverlapsWith(interval)) {
        if (message) {
          *message = "time-space contention on " + resource_id + " with " + res.mission_id;
        }
        return false;
      }
    }
  }
  // 全部通过 -> 一次性落表
  for (const auto& [resource_id, interval] : pending) {
    time_space_reservations_[resource_id].push_back(
        TimeSpaceReservation{mission_id, resource_id, interval});
  }
  if (message) {
    *message = "time-space reservation confirmed for " + std::to_string(pending.size()) +
               " resource(s)";
  }
  return true;
}

std::vector<std::string> ResourceTable::ReleaseTimeSpaceResources(
    const std::string& mission_id, const std::vector<std::string>& resource_ids) {
  std::vector<std::string> released;
  for (const auto& resource_id : resource_ids) {
    const auto it = time_space_reservations_.find(resource_id);
    if (it == time_space_reservations_.end()) {
      continue;
    }
    auto& list = it->second;
    const std::size_t before = list.size();
    list.erase(std::remove_if(list.begin(), list.end(),
                               [&mission_id](const TimeSpaceReservation& r) {
                                 return r.mission_id == mission_id;
                               }),
               list.end());
    if (list.size() != before) {
      released.push_back(resource_id);
    }
    if (list.empty()) {
      time_space_reservations_.erase(it);
    }
  }
  return released;
}

bool ResourceTable::ExtendTimeSpaceReservation(
    const std::string& mission_id, const std::string& resource_id,
    const std::chrono::steady_clock::time_point& new_end, std::string* message) {
  PruneExpiredTimeSpace(time_space_reservations_, std::chrono::steady_clock::now());
  auto it = time_space_reservations_.find(resource_id);
  if (it == time_space_reservations_.end()) {
    if (message) *message = "no time-space reservation for " + resource_id;
    return false;
  }
  // 找到本任务在该资源上的预约并确认延长不与他人冲突
  const auto mine = std::find_if(
      it->second.begin(), it->second.end(),
      [&mission_id](const TimeSpaceReservation& r) { return r.mission_id == mission_id; });
  if (mine == it->second.end()) {
    if (message) *message = "no time-space reservation for mission " + mission_id + " on " + resource_id;
    return false;
  }
  if (new_end <= mine->interval.end) {
    if (message) *message = "no extension needed";
    return true;  // 幂等：未延长
  }
  const TimeInterval extended{mine->interval.start, new_end};
  for (const auto& res : it->second) {
    if (res.mission_id == mission_id) {
      continue;
    }
    if (res.interval.OverlapsWith(extended)) {
      if (message) {
        *message = "extension conflicts with mission " + res.mission_id + " on " + resource_id;
      }
      return false;
    }
  }
  mine->interval.end = new_end;
  if (message) *message = "time-space reservation extended";
  return true;
}

ResourceTable::ReleaseResult ResourceTable::Release(const std::string& mission_id) {
  ReleaseResult result;
  // A7：入口先剪除过期时空预约
  PruneExpiredTimeSpace(time_space_reservations_, std::chrono::steady_clock::now());
  for (auto lock = locks_.begin(); lock != locks_.end();) {
    if (lock->second == mission_id) {
      lock = locks_.erase(lock);
      result.released = true;
    } else {
      ++lock;
    }
  }

  // 清空该任务的所有时空预约
  for (auto& [res_id, list] : time_space_reservations_) {
    auto before_size = list.size();
    list.erase(std::remove_if(list.begin(), list.end(),
                              [&mission_id](const TimeSpaceReservation& r) {
                                return r.mission_id == mission_id;
                              }),
               list.end());
    if (list.size() < before_size) {
      result.released = true;
    }
  }

  result.message =
      result.released ? "released resources of " + mission_id : "no resources held by " + mission_id;
  return result;
}

std::vector<std::string> ResourceTable::ReleaseResources(
    const std::string& mission_id, const std::vector<std::string>& resource_ids) {
  std::vector<std::string> released;
  if (mission_id.empty()) {
    return released;  // 空 owner 不允许释放任何东西
  }
  for (const auto& resource_id : resource_ids) {
    const auto existing = locks_.find(resource_id);
    // 非本任务持有的 id（他人锁/traffic_block/未持有）一律跳过：所有权安全 + 幂等
    if (existing == locks_.end() || existing->second != mission_id) {
      continue;
    }
    locks_.erase(existing);
    released.push_back(resource_id);
  }
  return released;
}

bool ResourceTable::Block(const std::string& resource_id, const std::string& reason,
                          std::string* message) {
  const std::string owner = std::string(kBlockPrefix) + reason;
  const auto existing = locks_.find(resource_id);
  if (existing != locks_.end()) {
    if (message != nullptr) {
      *message = "resource already held: " + resource_id + " by " + existing->second;
    }
    return false;
  }
  locks_[resource_id] = owner;
  if (message != nullptr) {
    *message = "blocked " + resource_id;
  }
  return true;
}

bool ResourceTable::Unblock(const std::string& resource_id, std::string* message) {
  const auto existing = locks_.find(resource_id);
  if (existing == locks_.end()) {
    if (message != nullptr) {
      *message = "resource not locked: " + resource_id;
    }
    return false;
  }
  if (!IsTrafficBlockOwner(existing->second)) {
    if (message != nullptr) {
      *message = "cannot unblock resource held by mission: " + existing->second;
    }
    return false;
  }
  locks_.erase(existing);
  if (message != nullptr) {
    *message = "unblocked " + resource_id;
  }
  return true;
}

bool ResourceTable::Locked(const std::string& resource_id) const {
  return locks_.find(resource_id) != locks_.end();
}

std::optional<std::string> ResourceTable::Owner(const std::string& resource_id) const {
  const auto it = locks_.find(resource_id);
  if (it == locks_.end()) {
    return std::nullopt;
  }
  return it->second;
}

std::size_t ResourceTable::TimeSpaceReservationCount() const {
  std::size_t total = 0;
  for (const auto& entry : time_space_reservations_) {
    total += entry.second.size();
  }
  return total;
}

std::vector<std::pair<std::string, std::string>> ResourceTable::Snapshot() const {
  std::vector<std::pair<std::string, std::string>> out(locks_.begin(), locks_.end());
  std::sort(out.begin(), out.end(), [](const auto& lhs, const auto& rhs) {
    return lhs.first < rhs.first;
  });
  return out;
}

}  // namespace amr_dispatcher_core::dispatcher
