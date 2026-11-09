#include "amr_dispatcher_core/chassis/link_quality.hpp"

#include <algorithm>
#include <utility>

namespace amr_dispatcher_core::chassis {

LinkQualityMonitor::LinkQualityMonitor(LinkQualityConfig config)
    : config_(std::move(config)) {
  // window_size=0 属于非法配置：钳到 1，避免环形缓冲取模除零
  if (config_.window_size == 0) {
    config_.window_size = 1;
  }
  window_.assign(config_.window_size, Sample{});
}

void LinkQualityMonitor::RecordSuccess(const double latency_ms) {
  std::lock_guard<std::mutex> lock(mutex_);
  window_[head_] = Sample{true, latency_ms};
  head_ = (head_ + 1) % config_.window_size;
  if (window_filled_ < config_.window_size) {
    ++window_filled_;
  }
  ++total_success_;
  ++consecutive_success_;
  last_seen_ = std::chrono::steady_clock::now();

  // 恢复信号（边沿）：条件"连续成功达标且（按重算前的旧 tier）尚未恢复健康"
  // 从不满足变为满足的沿只置一次；latch 跟随条件电平，避免消费后立刻重发。
  const bool condition = consecutive_success_ >= config_.success_threshold &&
                         tier_ != LinkHealthTier::kHealthy;
  if (condition && !recovery_condition_latched_) {
    pending_recovery_signal_ = true;
  }
  recovery_condition_latched_ = condition;

  RecomputeLocked();
}

void LinkQualityMonitor::RecordLoss() {
  std::lock_guard<std::mutex> lock(mutex_);
  window_[head_] = Sample{false, 0.0};
  head_ = (head_ + 1) % config_.window_size;
  if (window_filled_ < config_.window_size) {
    ++window_filled_;
  }
  ++total_lost_;
  consecutive_success_ = 0;
  last_seen_ = std::chrono::steady_clock::now();
  // 新丢包降级：清除未消费的恢复信号与条件电平（下一轮恢复重新取沿）
  pending_recovery_signal_ = false;
  recovery_condition_latched_ = false;
  RecomputeLocked();
}

void LinkQualityMonitor::RecordCrcError() {
  std::lock_guard<std::mutex> lock(mutex_);
  window_[head_] = Sample{false, 0.0};
  head_ = (head_ + 1) % config_.window_size;
  if (window_filled_ < config_.window_size) {
    ++window_filled_;
  }
  // M2：收到 CRC 错帧本身就是"链路有活动"的证据，同样刷新 last_seen，
  // 否则纯 CRC 错误流会被误判为链路静默（stale）。
  last_seen_ = std::chrono::steady_clock::now();
  ++total_crc_error_;
  ++total_lost_;
  consecutive_success_ = 0;
  pending_recovery_signal_ = false;
  recovery_condition_latched_ = false;
  RecomputeLocked();
}

void LinkQualityMonitor::RecomputeLocked() {
  cached_snapshot_.frames_total = total_success_ + total_lost_;
  cached_snapshot_.frames_success = total_success_;
  cached_snapshot_.frames_lost = total_lost_;
  cached_snapshot_.frames_crc_error = total_crc_error_;
  cached_snapshot_.last_seen = last_seen_;

  // 真滑动窗口：只统计 ring 中最近 window_filled_ 个样本的丢包与时延。
  // 累计计数（frames_*）只保留历史总量，不参与 loss_rate / tier 判定，
  // 否则长期健康的累计基数会把突发丢包稀释得测不出来。
  std::size_t window_losses = 0;
  std::size_t window_success = 0;
  double window_latency_sum_ms = 0.0;
  double window_latency_max_ms = 0.0;
  for (std::size_t i = 0; i < window_filled_; ++i) {
    const auto& sample = window_[(head_ + window_.size() - window_filled_ + i) % window_.size()];
    if (sample.success) {
      ++window_success;
      window_latency_sum_ms += sample.latency_ms;
      window_latency_max_ms = std::max(window_latency_max_ms, sample.latency_ms);
    } else {
      ++window_losses;
    }
  }

  if (window_filled_ == 0) {
    cached_snapshot_.loss_rate = 0.0;
    cached_snapshot_.average_latency_ms = 0.0;
    cached_snapshot_.max_latency_ms = 0.0;
    tier_ = LinkHealthTier::kUnknown;
    cached_snapshot_.tier = tier_;
    return;
  }
  cached_snapshot_.loss_rate =
      static_cast<double>(window_losses) / static_cast<double>(window_filled_);
  cached_snapshot_.average_latency_ms =
      window_success > 0 ? window_latency_sum_ms / static_cast<double>(window_success) : 0.0;
  cached_snapshot_.max_latency_ms = window_latency_max_ms;

  // 优先级：critical > degraded > healthy > unknown
  if (cached_snapshot_.loss_rate >= config_.degraded_loss_rate * 2.0) {
    tier_ = LinkHealthTier::kCritical;
  } else if (cached_snapshot_.loss_rate >= config_.degraded_loss_rate) {
    tier_ = LinkHealthTier::kDegraded;
  } else if (cached_snapshot_.loss_rate <= config_.healthy_loss_rate &&
             consecutive_success_ >= config_.success_threshold) {
    tier_ = LinkHealthTier::kHealthy;
  } else if (tier_ == LinkHealthTier::kUnknown) {
    tier_ = LinkHealthTier::kDegraded;
  }
  cached_snapshot_.tier = tier_;
}

LinkQualitySnapshot LinkQualityMonitor::Snapshot() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return cached_snapshot_;
}

LinkHealthTier LinkQualityMonitor::CurrentTier() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return tier_;
}

bool LinkQualityMonitor::IsHealthy() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return tier_ == LinkHealthTier::kHealthy;
}

bool LinkQualityMonitor::IsCritical() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return tier_ == LinkHealthTier::kCritical;
}

bool LinkQualityMonitor::ConsumeRecoverySignal() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!pending_recovery_signal_) {
    return false;
  }
  pending_recovery_signal_ = false;
  return true;
}

void LinkQualityMonitor::Reset() {
  std::lock_guard<std::mutex> lock(mutex_);
  std::fill(window_.begin(), window_.end(), Sample{});
  head_ = 0;
  window_filled_ = 0;
  total_success_ = 0;
  total_lost_ = 0;
  total_crc_error_ = 0;
  consecutive_success_ = 0;
  pending_recovery_signal_ = false;
  recovery_condition_latched_ = false;
  cached_snapshot_ = LinkQualitySnapshot{};
  tier_ = LinkHealthTier::kUnknown;
}

}  // namespace amr_dispatcher_core::chassis
