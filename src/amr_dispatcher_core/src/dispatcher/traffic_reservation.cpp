#include "amr_dispatcher_core/dispatcher/traffic_reservation.hpp"

#include <algorithm>
#include <sstream>

namespace amr_dispatcher_core::dispatcher {
namespace {

bool StartsWith(const std::string& value, const std::string& prefix) {
  return value.rfind(prefix, 0) == 0;
}

constexpr const char* kBlockPrefix = "traffic_block:";

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
  if (include_intersection_locks) {
    for (const auto& station_id : station_path) {
      lock_ids.push_back(RouteNodeId(station_id));
    }
  }
  for (std::size_t index = 1; index < station_path.size(); ++index) {
    lock_ids.push_back(RouteEdgeId(station_path[index - 1], station_path[index]));
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
  for (const auto& resource_id : resource_ids) {
    const auto existing = locks_.find(resource_id);
    if (existing != locks_.end() && existing->second != mission_id) {
      std::ostringstream oss;
      oss << "resource locked: " << resource_id << " by " << existing->second;
      result.message = oss.str();
      return result;  // 未落任何锁，原子性成立
    }
  }
  for (const auto& resource_id : resource_ids) {
    locks_[resource_id] = mission_id;
    result.newly_reserved.push_back(resource_id);
  }
  std::ostringstream oss;
  oss << "reserved " << resource_ids.size() << " resource(s)";
  result.success = true;
  result.message = oss.str();
  return result;
}

ResourceTable::ReleaseResult ResourceTable::Release(const std::string& mission_id) {
  ReleaseResult result;
  for (auto lock = locks_.begin(); lock != locks_.end();) {
    if (lock->second == mission_id) {
      lock = locks_.erase(lock);
      result.released = true;
    } else {
      ++lock;
    }
  }
  result.message =
      result.released ? "released resources of " + mission_id : "no resources held by " + mission_id;
  return result;
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
  const auto lock = locks_.find(resource_id);
  if (lock == locks_.end()) {
    if (message != nullptr) {
      *message = "resource not found: " + resource_id;
    }
    return false;
  }
  if (!IsTrafficBlockOwner(lock->second)) {
    if (message != nullptr) {
      *message = "resource is reserved by mission: " + lock->second;
    }
    return false;
  }
  locks_.erase(lock);
  if (message != nullptr) {
    *message = "unblocked " + resource_id;
  }
  return true;
}

bool ResourceTable::Locked(const std::string& resource_id) const {
  return locks_.count(resource_id) > 0;
}

std::optional<std::string> ResourceTable::Owner(const std::string& resource_id) const {
  const auto lock = locks_.find(resource_id);
  if (lock == locks_.end()) {
    return std::nullopt;
  }
  return lock->second;
}

std::vector<std::pair<std::string, std::string>> ResourceTable::Snapshot() const {
  std::vector<std::pair<std::string, std::string>> snapshot(locks_.begin(), locks_.end());
  std::sort(snapshot.begin(), snapshot.end());
  return snapshot;
}

}  // namespace amr_dispatcher_core::dispatcher
