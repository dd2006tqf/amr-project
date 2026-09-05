#pragma once

#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace amr_dispatcher_core::dispatcher {

// 资源预留表：resource_id → mission_id（traffic_block:* 前缀表示人工封路）。
// 资源分两类：route_edge:A__B（路段）与 route_node:station（节点/交汇点）。
class ResourceTable {
 public:
  struct ReserveResult {
    bool success = false;
    std::string message;
    std::vector<std::string> newly_reserved;  // 失败时已全部回滚，保持原子性
  };

  struct ReleaseResult {
    bool released = false;
    std::string message;
  };

  // 原子预留：任一资源被他人持有则整体失败且不落锁。
  ReserveResult Reserve(const std::string& mission_id,
                        const std::vector<std::string>& resource_ids);

  // 释放某任务持有的全部资源（含遗漏登记），返回是否实际删除。
  ReleaseResult Release(const std::string& mission_id);

  // 人工封路：owner = "traffic_block:<reason>"，对任务预留不可见覆盖。
  bool Block(const std::string& resource_id, const std::string& reason, std::string* message);
  bool Unblock(const std::string& resource_id, std::string* message);

  bool Locked(const std::string& resource_id) const;
  std::optional<std::string> Owner(const std::string& resource_id) const;
  std::size_t size() const { return locks_.size(); }

  std::vector<std::pair<std::string, std::string>> Snapshot() const;  // 已排序

  // 资源命名规则。
  static std::string RouteEdgeId(const std::string& from, const std::string& to);
  static std::string RouteNodeId(const std::string& station_id);
  static std::vector<std::string> BuildLockIds(
      const std::vector<std::string>& station_path, bool include_intersection_locks);
  static std::optional<std::pair<std::string, std::string>> ParseRouteEdge(
      const std::string& resource_id);
  static bool IsTrafficBlockOwner(const std::string& owner_id);

 private:
  std::unordered_map<std::string, std::string> locks_;  // resource_id → owner
};

using TrafficReservationTable = ResourceTable;

}  // namespace amr_dispatcher_core::dispatcher
