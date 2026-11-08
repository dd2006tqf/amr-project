#include "amr_dispatcher_core/chassis/link_recovery.hpp"

#include <algorithm>
#include <cmath>
#include <random>

namespace amr_dispatcher_core::chassis {

LinkRecoveryPolicy::LinkRecoveryPolicy(LinkRecoveryConfig config)
    : config_(std::move(config)),
      current_backoff_(config_.initial_backoff) {}

std::chrono::milliseconds LinkRecoveryPolicy::Tick() {
  switch (state_) {
    case State::kIdle: {
      attempts_ = 0;
      current_backoff_ = config_.initial_backoff;
      state_ = State::kWaiting;
      return std::chrono::milliseconds{0};
    }
    case State::kWaiting: {
      const auto wait = current_backoff_;
      state_ = State::kAttempting;
      return wait;
    }
    case State::kAttempting: {
      ++attempts_;
      if (config_.open_attempt && config_.open_attempt()) {
        state_ = State::kConnected;
        return std::chrono::milliseconds{0};
      }
      if (config_.max_attempts > 0 && attempts_ >= config_.max_attempts) {
        state_ = State::kExhausted;
        return std::chrono::milliseconds{0};
      }
      current_backoff_ = DecideBackoff(config_, attempts_);
      state_ = State::kWaiting;
      return std::chrono::milliseconds{0};
    }
    case State::kConnected:
      return std::chrono::milliseconds{0};
    case State::kExhausted:
      return std::chrono::milliseconds{0};
  }
  return std::chrono::milliseconds{0};
}

void LinkRecoveryPolicy::NotifyConnected() {
  state_ = State::kConnected;
  attempts_ = 0;
  current_backoff_ = config_.initial_backoff;
}

void LinkRecoveryPolicy::NotifyFailed() {
  if (state_ == State::kConnected) {
    state_ = State::kWaiting;
  }
}

void LinkRecoveryPolicy::ForceReset() {
  state_ = State::kIdle;
  attempts_ = 0;
  current_backoff_ = config_.initial_backoff;
}

std::chrono::milliseconds LinkRecoveryPolicy::DecideBackoff(
    const LinkRecoveryConfig& config, std::size_t attempt_index,
    std::function<double()> jitter_source) {
  // attempt_index 从 1 开始，第一次失败后用 initial_backoff。
  const double exp = std::pow(config.backoff_multiplier, static_cast<double>(attempt_index - 1));
  const double base_ms = static_cast<double>(config.initial_backoff.count()) * exp;
  const double clamped = std::min(base_ms, static_cast<double>(config.max_backoff.count()));
  const double jitter = (jitter_source ? jitter_source() : 0.0) * config.jitter_ratio;
  const double with_jitter = clamped * (1.0 + jitter);
  return std::chrono::milliseconds{static_cast<long>(std::max(0.0, with_jitter))};
}

}  // namespace amr_dispatcher_core::chassis
