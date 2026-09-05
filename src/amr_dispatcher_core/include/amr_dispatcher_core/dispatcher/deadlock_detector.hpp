#pragma once

#include <chrono>
#include <cstddef>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "amr_dispatcher_core/dispatcher/mission_state.hpp"

namespace amr_dispatcher_core::dispatcher {

// 死锁检测配置
struct DeadlockDetectorConfig {
  std::size_t max_active_missions = 16;        // 同时活跃 mission 上限
  std::chrono::milliseconds active_idle_threshold{std::chrono::seconds(10)};
  std::chrono::milliseconds reservation_hold_threshold{std::chrono::seconds(30)};
};

// 死锁检测输入快照：调用方从外部聚合 mission 状态 + traffic_reservation 状态。
struct DeadlockSnapshot {
  std::vector<Mission> active_missions;          // 当前 ACTIVE 的任务
  std::unordered_map<std::string, std::chrono::steady_clock::time_point> last_progress;
  std::unordered_map<std::string, std::chrono::steady_clock::time_point> reservation_acquired_at;
  std::unordered_set<std::string> reserved_resource_ids;
};

// 检测结论：单次 Detect 返回的所有嫌疑。
enum class DeadlockKind {
  kIdleActive,            // ACTIVE 任务在 idle_threshold 内无 progress
  kLongReservationHold,   // 持有资源超过阈值（典型循环等待前置资源）
  kMutualBlock,           // 两个以上任务相互等待对方的资源（强死锁）
};

struct DeadlockFinding {
  std::vector<std::string> mission_ids;  // 涉及的 mission_id
  DeadlockKind kind = DeadlockKind::kIdleActive;
  std::string description;
};

// 死锁检测器：纯函数式，调用方每 N 毫秒喂入快照、返回 findings。
// 不持有 mission 数据本身，便于测试。
class DeadlockDetector {
 public:
  explicit DeadlockDetector(DeadlockDetectorConfig config = {});

  std::vector<DeadlockFinding> Detect(const DeadlockSnapshot& snapshot) const;

 private:
  DeadlockDetectorConfig config_;
};

}  // namespace amr_dispatcher_core::dispatcher
