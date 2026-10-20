#pragma once

#include <chrono>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace amr_dispatcher_core::dispatcher {

// 时空预约区间：[start_time, end_time]
struct TimeInterval {
  std::chrono::steady_clock::time_point start;
  std::chrono::steady_clock::time_point end;

  bool OverlapsWith(const TimeInterval& other) const {
    return start < other.end && other.start < end;
  }
};

struct TimeSpaceReservation {
  std::string mission_id;
  std::string resource_id;
  TimeInterval interval;
};

// 资源预留表：兼具静态原子锁与时空预约区间锁（Time-Space Reservation）
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

  // 原子静态排他预留：任一资源被他人持有则整体失败且不落锁。
  ReserveResult Reserve(const std::string& mission_id,
                        const std::vector<std::string>& resource_ids);

  // 时空窗口预约：允许多车错峰占用同一路段，只要时间窗口不重叠即可并行预约！
  bool ReserveTimeSpace(const std::string& mission_id,
                        const std::string& resource_id,
                        std::chrono::steady_clock::time_point start,
                        std::chrono::steady_clock::time_point end,
                        std::string* message);

  // 释放某任务持有的全部资源（含时空预约），返回是否实际删除。
  ReleaseResult Release(const std::string& mission_id);

  // 逐段动态解锁：仅释放 owner == mission_id 且列于 resource_ids 的锁。
  // 他人锁（含 traffic_block）、未持有/已释放的 id 一律跳过（对未持有 id 是安全 no-op）。
  // 返回实际删除的 resource_id 列表（保序、与入参一致），供幂等重算与日志取证。
  // 注意：只处理静态锁；时空预约不在逐段释放语义内（Release(mission) 才全清）。
  std::vector<std::string> ReleaseResources(
      const std::string& mission_id, const std::vector<std::string>& resource_ids);

  // 人工封路：owner = "traffic_block:<reason>"，对任务预留不可见覆盖。
  bool Block(const std::string& resource_id, const std::string& reason, std::string* message);
  bool Unblock(const std::string& resource_id, std::string* message);

  bool Locked(const std::string& resource_id) const;
  std::optional<std::string> Owner(const std::string& resource_id) const;
  std::size_t size() const { return locks_.size(); }

  // 时空预约表当前有效条目总数（过期回收的可观测指标）
  std::size_t TimeSpaceReservationCount() const;

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
  // resource_id -> 所有的时空预约区间
  std::unordered_map<std::string, std::vector<TimeSpaceReservation>> time_space_reservations_;
};

using TrafficReservationTable = ResourceTable;

}  // namespace amr_dispatcher_core::dispatcher
