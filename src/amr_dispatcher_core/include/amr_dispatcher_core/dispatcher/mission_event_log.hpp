#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace amr_dispatcher_core::dispatcher {

// 调度事件流水条目。stamp 为 ISO-8601 文本（由上层注入，core 不碰时钟格式）。
struct MissionEvent {
  std::string stamp;
  std::string mission_id;
  std::string event;          // 例如 "state:ACTIVE"、"reserved"、"failed"
  std::string previous_event;
  std::string message;
  bool recoverable = true;
};

// 固定容量环形事件缓冲：满后覆盖最旧条目，O(1) 追加，无动态分配抖动。
// 用于故障回溯与 CLI `history` 命令。
class MissionEventLog {
 public:
  static constexpr std::size_t kDefaultCapacity = 1024;
  static constexpr std::size_t kMaxCapacity = 65536;

  explicit MissionEventLog(std::size_t capacity = kDefaultCapacity);

  void Append(MissionEvent event);

  // 最近 limit 条（新→旧），可选按事件名 / mission_id 过滤。
  std::vector<MissionEvent> Recent(
      std::size_t limit, const std::string& event_filter = "",
      const std::string& mission_id_filter = "") const;

  // 全量导出（旧→新），快照/落盘用。
  std::vector<MissionEvent> Snapshot() const;

  std::size_t size() const { return size_; }
  std::size_t capacity() const { return capacity_; }
  std::uint64_t total_appended() const { return total_appended_; }
  void Clear();

  // 参数夹取：请求值 ≤0 用 default_limit，再夹到 max_limit。
  static std::size_t NormalizeLimit(
      int requested_limit, std::size_t default_limit = kDefaultCapacity,
      std::size_t max_limit = kMaxCapacity);

 private:
  std::size_t IndexFromOldest(std::size_t offset) const;

  std::size_t capacity_;
  std::vector<MissionEvent> slots_;
  std::size_t head_ = 0;      // 下一次写入位置
  std::size_t size_ = 0;
  std::uint64_t total_appended_ = 0;
};

}  // namespace amr_dispatcher_core::dispatcher
