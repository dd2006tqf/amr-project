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
}

void BackendDegradationController::RecordSuccess(const double latency_ms) {
  quality_.RecordSuccess(latency_ms);
}

void BackendDegradationController::RecordLoss() { quality_.RecordLoss(); }

void BackendDegradationController::RecordCrcError() { quality_.RecordCrcError(); }

bool BackendDegradationController::EvaluateAndMaybeSwitch(std::string* reason) {
  const auto snap = quality_.Snapshot();
  if (snap.frames_total < config_.warmup_samples) {
    return false;
  }
  if (snap.loss_rate < config_.demote_loss_rate) {
    return false;
  }
  // 触发降级：在 cooldown 内只触发一次
  const auto now = std::chrono::steady_clock::now();
  if (last_demote_at_.time_since_epoch().count() != 0 &&
      now - last_demote_at_ < config_.cooldown) {
    return false;
  }
  if (current_index_ + 1 >= backends_.size()) {
    return false;  // 已经在最低优先级
  }
  std::string r = "loss_rate=" + std::to_string(snap.loss_rate) +
                  " exceeds demote_loss_rate=" +
                  std::to_string(config_.demote_loss_rate);
  if (SwitchTo(current_index_ + 1, nullptr)) {
    last_demote_at_ = now;
    if (reason) {
      *reason = std::move(r);
    }
    return true;
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
