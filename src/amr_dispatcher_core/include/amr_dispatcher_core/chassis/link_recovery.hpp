#pragma once

#include <chrono>
#include <cstddef>
#include <functional>
#include <string>

#include "amr_dispatcher_core/chassis/link_quality.hpp"

namespace amr_dispatcher_core::chassis {

// 重连策略配置：指数退避（带抖动）。
struct LinkRecoveryConfig {
  std::chrono::milliseconds initial_backoff{200};
  std::chrono::milliseconds max_backoff{std::chrono::seconds(10)};
  double backoff_multiplier = 2.0;
  double jitter_ratio = 0.2;                 // [0, 1)：随机抖动比例
  std::size_t max_attempts = 0;              // 0 表示不限次数
  std::chrono::milliseconds open_timeout{std::chrono::seconds(2)};
  // 触发重连的回调：失败时调用 DecideBackoff(n_attempts) 决定下一步。
  std::function<bool()> open_attempt;        // 返回 true 表示 Open 成功
};

// 重连状态机：每次 Tick() 推进一次，调用方驱动（通常是底盘 read 循环）。
class LinkRecoveryPolicy {
 public:
  enum class State {
    kIdle,        // 未发起过重连
    kWaiting,     // 等待下一次 backoff
    kAttempting,  // 正在调用 open_attempt
    kConnected,   // 上一次重连成功；调用方应反馈成功/失败以驱动状态
    kExhausted,   // 达到 max_attempts 上限且仍未成功
  };

  explicit LinkRecoveryPolicy(LinkRecoveryConfig config);

  // 由调用方驱动一次循环。返回建议 sleep 时长（= 下一次 backoff）；
  // 返回 0ms 表示本轮立即需要再次尝试。
  std::chrono::milliseconds Tick();

  void NotifyConnected();    // 调用方确认链路恢复
  void NotifyFailed();       // 调用方确认链路仍失败（成功 Tick 后又断开）
  void ForceReset();

  State state() const { return state_; }
  std::size_t attempts() const { return attempts_; }
  std::chrono::milliseconds current_backoff() const { return current_backoff_; }

  // 工具：纯函数，暴露给测试和上层调度
  static std::chrono::milliseconds DecideBackoff(
      const LinkRecoveryConfig& config, std::size_t attempt_index,
      std::function<double()> jitter_source = {});

 private:
  LinkRecoveryConfig config_;
  State state_ = State::kIdle;
  std::size_t attempts_ = 0;
  std::chrono::milliseconds current_backoff_;
};

}  // namespace amr_dispatcher_core::chassis
