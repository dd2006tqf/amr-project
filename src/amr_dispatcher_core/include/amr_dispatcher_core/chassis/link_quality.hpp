#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

namespace amr_dispatcher_core::chassis {

// 链路质量统计：滑动窗口累计丢包率、CRC 错误率、平均时延、最大时延。
// 线程安全（内部 mutex 保护），单 chassis_backend 复用一份。
struct LinkQualityConfig {
  std::size_t window_size = 64;           // 滑动窗口样本数
  std::size_t success_threshold = 50;     // 连续成功帧数到该值视为恢复良好
  double healthy_loss_rate = 0.02;        // 健康上限（窗口内丢包率 <= 该值）
  double degraded_loss_rate = 0.15;       // 降级阈值（超过进入 degraded）
};

enum class LinkHealthTier {
  kUnknown,
  kHealthy,
  kDegraded,
  kCritical,
};

struct LinkQualitySnapshot {
  std::size_t frames_total = 0;
  std::size_t frames_success = 0;
  std::size_t frames_lost = 0;
  std::size_t frames_crc_error = 0;
  double loss_rate = 0.0;                 // 丢包率（含 CRC 错误）
  double average_latency_ms = 0.0;
  double max_latency_ms = 0.0;
  std::chrono::steady_clock::time_point last_seen{};
  LinkHealthTier tier = LinkHealthTier::kUnknown;
};

class LinkQualityMonitor {
 public:
  explicit LinkQualityMonitor(LinkQualityConfig config = {});

  // 每次从后端收到一帧时调用：success 表示解码成功；latency 为从写入命令到收到对应反馈的往返时延（若无填 0）。
  void RecordSuccess(double latency_ms);
  void RecordLoss();
  void RecordCrcError();

  LinkQualitySnapshot Snapshot() const;
  LinkHealthTier CurrentTier() const;

  // 健康判定
  bool IsHealthy() const;
  bool IsCritical() const;
  // 自上次进入非 Healthy 后是否累计到 success_threshold 帧成功（用于触发恢复）
  bool ConsumeRecoverySignal();

  void Reset();

 private:
  struct Sample {
    bool success = false;
    double latency_ms = 0.0;
  };

  void RecomputeLocked();

  LinkQualityConfig config_;
  mutable std::mutex mutex_;
  std::vector<Sample> window_;
  std::size_t head_ = 0;
  std::size_t window_filled_ = 0;
  std::size_t total_success_ = 0;
  std::size_t total_lost_ = 0;
  std::size_t total_crc_error_ = 0;
  std::size_t consecutive_success_ = 0;
  std::size_t pending_recovery_signal_ = 0;
  double latency_sum_ms_ = 0.0;
  double latency_max_ms_ = 0.0;
  std::chrono::steady_clock::time_point last_seen_{};
  LinkHealthTier tier_ = LinkHealthTier::kUnknown;
  LinkQualitySnapshot cached_snapshot_{};
};

}  // namespace amr_dispatcher_core::chassis
