#include "amr_dispatcher_core/safety/fault_supervisor.hpp"

namespace amr_dispatcher_core::safety {

FaultSupervisor::FaultSupervisor(SupervisorConfig config)
    : config_(config), start_time_(std::chrono::steady_clock::now()) {}

void FaultSupervisor::UpdateHealth(bool system_healthy, const std::string& error_message) {
  health_seen_ = true;
  last_system_healthy_ = system_healthy;
  last_error_message_ = error_message;
}

void FaultSupervisor::Tick() {
  auto now = std::chrono::steady_clock::now();

  if (!health_seen_) {
    state_ = SupervisorState::kWaiting;
    message_ = "waiting for health signal";
    return;
  }

  bool in_grace = (now - start_time_) < config_.startup_grace;

  if (!last_system_healthy_) {
    if (in_grace) {
      state_ = SupervisorState::kArming;
      message_ = "startup grace active: " + last_error_message_;
      return;
    }

    state_ = SupervisorState::kFault;
    message_ = "emergency stop requested: " + last_error_message_;

    bool cooldown_ok = (last_command_time_.time_since_epoch().count() == 0) ||
                       (now - last_command_time_) >= config_.command_cooldown;
    if (!estop_requested_ && cooldown_ok) {
      estop_requested_ = true;
      last_command_time_ = now;
    }
    return;
  }

  // 健康正常
  if (state_ == SupervisorState::kFault) {
    if (config_.auto_clear) {
      state_ = SupervisorState::kRecovery;
      message_ = "system recovered, auto-clearing estop";
      estop_requested_ = false;
      last_command_time_ = now;
    }
  } else if (state_ == SupervisorState::kRecovery || state_ == SupervisorState::kArming || state_ == SupervisorState::kWaiting) {
    state_ = SupervisorState::kNormal;
    message_ = "nominal operation";
  }
}

std::string FaultSupervisor::state_string() const {
  switch (state_) {
    case SupervisorState::kWaiting: return "WAITING";
    case SupervisorState::kArming: return "ARMING";
    case SupervisorState::kNormal: return "NORMAL";
    case SupervisorState::kFault: return "FAULT";
    case SupervisorState::kRecovery: return "RECOVERY";
  }
  return "UNKNOWN";
}

void FaultSupervisor::Reset() {
  state_ = SupervisorState::kWaiting;
  message_ = "reset";
  health_seen_ = false;
  estop_requested_ = false;
  start_time_ = std::chrono::steady_clock::now();
}

}  // namespace amr_dispatcher_core::safety
