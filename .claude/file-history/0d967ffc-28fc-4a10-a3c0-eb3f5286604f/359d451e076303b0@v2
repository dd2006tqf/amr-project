#include "amr_dispatcher_core/chassis/backend_degradation.hpp"

#include <utility>

namespace amr_dispatcher_core::chassis {

BackendDegradationController::BackendDegradationController(
    std::vector<BackendDescriptor> backends,
    std::vector<ChassisBackendConfig> configs,
    BackendDegradationConfig config)
    : backends_(std::move(backends)),
      configs_(std::move(configs)),
      config_(config) {
  if (backends_.empty()) {
    backends_ = DefaultBackendDescriptors();
    if (configs_.empty()) {
      configs_.emplace_back();
    }
  }
  if (configs_.size() < backends_.size()) {
    configs_.resize(backends_.size());
  }
  // 默认选第一个后端
  SwitchTo(0, nullptr);

  // 连接丢失后的重连策略：open_attempt 按 preferred 顺序扫描可打开的后端。
  LinkRecoveryConfig recovery_cfg;
  recovery_cfg.open_attempt = [this] { return TryOpenAnyBackend(nullptr); };
  recovery_ = std::make_unique<LinkRecoveryPolicy>(std::move(recovery_cfg));
}

void BackendDegradationController::RecordSuccess(const double latency_ms) {
  quality_.RecordSuccess(latency_ms);
}

void BackendDegradationController::RecordLoss() { quality_.RecordLoss(); }

void BackendDegradationController::RecordCrcError() { quality_.RecordCrcError(); }

bool BackendDegradationController::EvaluateAndMaybeSwitch(std::string* reason) {
  // 连接已断（构造或切档时 Open 失败导致 current_ 为空）：
  // 按 LinkRecoveryPolicy 的指数退避节奏扫描重连，未到退避窗口直接返回。
  if (!current_) {
    if (recovery_->state() == LinkRecoveryPolicy::State::kConnected) {
      recovery_->NotifyFailed();  // 上次连上又断了，退回等待态重新计时
    }
    const auto now = std::chrono::steady_clock::now();
    if (reconnect_not_before_.time_since_epoch().count() != 0 &&
        now < reconnect_not_before_) {
      return false;
    }
    const auto wait = recovery_->Tick();  // 可能在 kAttempting 态触发 open_attempt
    if (current_) {
      reconnect_not_before_ = {};
      last_switch_at_ = now;
      if (reason) {
        *reason = "reconnected to " + current_->Name();
      }
      return true;
    }
    reconnect_not_before_ = now + wait;
    return false;
  }

  const auto snap = quality_.Snapshot();
  if (snap.frames_total < config_.warmup_samples) {
    return false;
  }
  const auto now = std::chrono::steady_clock::now();
  // 切换冷却窗：降档与回切双向共用。先判窗口再消费恢复信号，
  // 避免冷却期内把恢复信号白白吃掉导致窗口结束后错失回切。
  const bool cooling = last_switch_at_.time_since_epoch().count() != 0 &&
                       now - last_switch_at_ < config_.cooldown;
  // 恢复回切：切离首选后链路重新达标（连续成功达到 success_threshold 触发恢复信号），
  // 优先于降级判定执行，回切到首选后端。
  if (current_index_ > 0 && !cooling && quality_.ConsumeRecoverySignal()) {
    const std::size_t prev_index = current_index_;
    if (SwitchTo(0, reason)) {
      last_switch_at_ = now;
      return true;
    }
    // 首选后端打不开（设备故障）：留在原后端，进入冷却等下一轮恢复窗口再试。
    // 不做这一步恢复，SwitchTo 失败会把 current_ 置空，整条链路直接断档。
    SwitchTo(prev_index, nullptr);
    last_switch_at_ = now;
    if (reason) {
      *reason = "failback open failed, kept backend index " + std::to_string(prev_index);
    }
    return false;
  }
  if (snap.loss_rate < config_.demote_loss_rate) {
    return false;
  }
  // 触发降级：在 cooldown 内只触发一次
  if (cooling) {
    return false;
  }
  if (current_index_ + 1 >= backends_.size()) {
    return false;  // 已经在最低优先级
  }
  std::string r = "loss_rate=" + std::to_string(snap.loss_rate) +
                  " exceeds demote_loss_rate=" +
                  std::to_string(config_.demote_loss_rate);
  if (SwitchTo(current_index_ + 1, nullptr)) {
    last_switch_at_ = now;
    if (reason) {
      *reason = std::move(r);
    }
    return true;
  }
  return false;
}

bool BackendDegradationController::TryOpenAnyBackend(std::string* reason) {
  for (std::size_t i = 0; i < backends_.size(); ++i) {
    if (SwitchTo(i, reason)) {
      return true;
    }
  }
  return false;
}

bool BackendDegradationController::SwitchTo(std::size_t index, std::string* reason) {
  if (index >= backends_.size()) {
    return false;
  }
  if (current_) {
    current_->Close();
  }
  current_ = backends_[index].factory(configs_[index]);
  std::string err;
  if (!current_->Open(&err)) {
    if (reason) {
      *reason = "open failed: " + err;
    }
    current_.reset();
    return false;
  }
  if (reason) {
    *reason = "switched to " + backends_[index].name;
  }
  current_index_ = index;
  quality_.Reset();
  return true;
}

}  // namespace amr_dispatcher_core::chassis
