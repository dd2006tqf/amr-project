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

// 资源申请与等待状态模型 (Wait-For Graph 模型)
struct ResourceAllocationModel {
  // mission_id -> 任务当前正在等待申请的资源 ID 列表 (例如等待 route_edge:B__C)
  std::unordered_map<std::string, std::vector<std::string>> waiting_for;
  // resource_id -> 当前持有该资源的 mission_id
  std::unordered_map<std::string, std::string> held_by;
};

// 死锁检测输入快照：调用方从外部聚合 mission 状态 + traffic_reservation 状态。
struct DeadlockSnapshot {
  std::vector<Mission> active_missions;          // 当前 ACTIVE 的任务
  std::unordered_map<std::string, std::chrono::steady_clock::time_point> last_progress;
  std::unordered_map<std::string, std::chrono::steady_clock::time_point> reservation_acquired_at;
  std::unordered_set<std::string> reserved_resource_ids;

  // 严密的资源依赖分配关系（支持 0 毫秒图论环检测）
  ResourceAllocationModel allocation_graph;
};

// 检测结论：单次 Detect 返回的所有嫌疑。
enum class DeadlockKind {
  kIdleActive,            // ACTIVE 任务在 idle_threshold 内无 progress
  kLongReservationHold,   // 持有资源超过阈值
  kMutualBlock,           // 依赖环死锁 (Wait-For Graph 存在环)
};

struct DeadlockFinding {
  std::vector<std::string> mission_ids;  // 涉及的 mission_id
  DeadlockKind kind = DeadlockKind::kIdleActive;
  std::string description;
  std::vector<std::string> cycle_path;   // 具体的成环路径，例如: [m1 -> r2 -> m2 -> r1 -> m1]
};

/**
 * @brief 工业级死锁检测器
 * 同时具备：
 * 1. 纯图论的 Wait-For-Graph (WFG) DFS 有向图环检测（0 毫秒确定性识别循环互斥争抢）；
 * 2. 启发式超时与长时间持锁兜底监控。
 */
class DeadlockDetector {
 public:
  explicit DeadlockDetector(DeadlockDetectorConfig config = {});

  std::vector<DeadlockFinding> Detect(const DeadlockSnapshot& snapshot) const;

  /**
   * @brief 纯图论有向图环检测算法 (DFS 实现)
   * @param wfg_model 资源持有与等待模型
   * @return std::vector<std::vector<std::string>> 检测到的所有死锁环 (mission 节点序列)
   */
  std::vector<std::vector<std::string>> DetectCycles(
      const ResourceAllocationModel& wfg_model) const;

 private:
  DeadlockDetectorConfig config_;
};

}  // namespace amr_dispatcher_core::dispatcher
