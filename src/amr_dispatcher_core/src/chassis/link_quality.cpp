#include "amr_dispatcher_core/chassis/link_quality.hpp"

#include <algorithm>

namespace amr_dispatcher_core::chassis {

LinkQualityMonitor::LinkQualityMonitor(LinkQualityConfig config)
    : config_(config), window_(config.window_size, Sample{}) {}

void LinkQualityMonitor::RecordSuccess(const double latency_ms) {
  std::lock_guard<std::mutex> lock(mutex_);
  window_[head_] = Sample{true, latency_ms};
  head_ = (head_ + 1) % config_.window_size;
  if (window_filled_ < config_.window_size) {
    ++window_filled_;
  }
  ++total_success_;
  ++consecutive_success_;
  latency_sum_ms_ += latency_ms;
  latency_max_ms_ = std::max(latency_max_ms_, latency_ms);
  last_seen_ = std::chrono::steady_clock::now();
  if (consecutive_success_ >= config_.success_threshold &&
      tier_ != LinkHealthTier::kHealthy) {
    ++pending_recovery_signal_;
  }
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
  RecomputeLocked();
}

void LinkQualityMonitor::RecordCrcError() {
  std::lock_guard<std::mutex> lock(mutex_);
  window_[head_] = Sample{false, 0.0};
  head_ = (head_ + 1) % config_.window_size;
  if (window_filled_ < config_.window_size) {
    ++window_filled_;
  }
  ++total_crc_error_;
  ++total_lost_;
  consecutive_success_ = 0;
  RecomputeLocked();
}

void LinkQualityMonitor::RecomputeLocked() {
  cached_snapshot_.frames_total = total_success_ + total_lost_;
  cached_snapshot_.frames_success = total_success_;
  cached_snapshot_.frames_lost = total_lost_;
  cached_snapshot_.frames_crc_error = total_crc_error_;
  if (cached_snapshot_.frames_total == 0) {
    cached_snapshot_.loss_rate = 0.0;
    cached_snapshot_.average_latency_ms = 0.0;
    cached_snapshot_.max_latency_ms = 0.0;
    tier_ = LinkHealthTier::kUnknown;
    cached_snapshot_.tier = tier_;
    return;
  }
  cached_snapshot_.loss_rate =
      static_cast<double>(cached_snapshot_.frames_lost) / static_cast<double>(cached_snapshot_.frames_total);
  cached_snapshot_.average_latency_ms =
      total_success_ > 0 ? latency_sum_ms_ / static_cast<double>(total_success_) : 0.0;
  cached_snapshot_.max_latency_ms = latency_max_ms_;
  cached_snapshot_.last_seen = last_seen_;

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
  if (pending_recovery_signal_ == 0) {
    return false;
  }
  --pending_recovery_signal_;
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
  pending_recovery_signal_ = 0;
  latency_sum_ms_ = 0.0;
  latency_max_ms_ = 0.0;
  cached_snapshot_ = LinkQualitySnapshot{};
  tier_ = LinkHealthTier::kUnknown;
}

}  // namespace amr_dispatcher_core::chassis
