#pragma once

#include <chrono>
#include <cstddef>
#include <functional>
#include <string>
#include <vector>

#include "amr_dispatcher_core/dispatcher/deadlock_detector.hpp"

namespace amr_dispatcher_core::dispatcher {

// 恢复策略：根据 deadlock findings 给出每个 mission 的处置（pause / cancel / retry）。
struct RecoveryAction {
  enum class Kind { kNone, kPause, kCancel, kRetry };
  std::string mission_id;
  Kind kind = Kind::kNone;
  std::string reason;
};

// 策略配置
struct RecoveryPolicyConfig {
  std::size_t max_retries = 3;                       // 单任务最大重试次数
  std::chrono::milliseconds retry_backoff{std::chrono::seconds(5)};
  bool auto_cancel_on_exhausted = true;              // 重试耗尽后自动 cancel
};

class RecoveryPolicy {
 public:
  explicit RecoveryPolicy(RecoveryPolicyConfig config = {});

  // 给一组 findings 决定 actions；同时需要调用方告知每个 mission 的 attempt 次数，
  // 以判定是否已超过 max_retries。
  std::vector<RecoveryAction> Decide(
      const std::vector<DeadlockFinding>& findings,
      const std::function<std::size_t(const std::string&)>& attempt_of) const;

  const RecoveryPolicyConfig& config() const { return config_; }

 private:
  RecoveryPolicyConfig config_;
};

}  // namespace amr_dispatcher_core::dispatcher
