#pragma once

#include <cstddef>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

#include "amr_dispatcher_core/dispatcher/mission_comparator.hpp"
#include "amr_dispatcher_core/dispatcher/mission_state.hpp"

namespace amr_dispatcher_core::dispatcher {

// 优先级任务队列。
// - 排序委托可插拔 MissionComparator（默认 priority FIFO）；
// - 入队拒绝重复 id 与超容量；
// - 支持按 order_id 整单暂停/恢复（暂停任务出队时跳过而非删除）；
// - 非线程安全，由上层（节点层）保证单线程访问或加锁。
class MissionQueue {
 public:
  struct Config {
    std::size_t max_size = 128;
    std::string comparator = "priority_fifo";
  };

  struct PushResult {
    bool accepted = false;
    std::string message;
  };

  struct PopResult {
    bool success = false;
    bool queue_empty = false;
    bool all_paused = false;   // 队列非空但全部处于暂停订单
    std::string message;
    std::optional<Mission> mission;
  };

  explicit MissionQueue(Config config);
  MissionQueue() : MissionQueue(Config{}) {}

  // 替换排序策略（转移所有权）。替换后立即重排。
  void SetComparator(std::unique_ptr<MissionComparator> comparator);
  std::string comparator_name() const { return comparator_->name(); }

  PushResult Push(Mission mission);
  PopResult PopNext();
  const Mission* Peek() const;

  // 返回 true 表示命中并删除；同时清理该订单的暂停标记。
  bool Cancel(const std::string& mission_id);

  // 调整排队中任务的优先级并恢复堆序；不存在返回 false。
  bool Reprioritize(const std::string& mission_id, int priority);

  // 整单暂停/恢复（作用于 order_id，不影响任务状态字段本身）。
  void PauseOrder(const std::string& order_id);
  void ResumeOrder(const std::string& order_id);
  bool IsOrderPaused(const std::string& order_id) const;
  std::size_t CancelOrder(const std::string& order_id);  // 整单取消，返回删除数

  std::size_t size() const { return items_.size(); }
  bool empty() const { return items_.empty(); }
  std::size_t capacity() const { return config_.max_size; }
  bool Contains(const std::string& mission_id) const;

  // 按出队顺序导出快照（调度状态话题/CLI 列表用）。
  std::vector<Mission> Snapshot() const;

 private:
  void Sort() const;

  Config config_;
  std::unique_ptr<MissionComparator> comparator_;
  std::vector<Mission> items_;
  std::unordered_set<std::string> paused_order_ids_;
  mutable bool dirty_ = false;  // 惰性排序：入队/改优先级只置脏，出队前重排
};

}  // namespace amr_dispatcher_core::dispatcher
