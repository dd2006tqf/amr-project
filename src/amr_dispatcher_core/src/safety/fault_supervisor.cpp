#include "amr_dispatcher_core/safety/fault_supervisor.hpp"

namespace amr_dispatcher_core::safety {

FaultSupervisor::FaultSupervisor(SupervisorConfig config)
    : config_(config), start_time_(std::chrono::steady_clock::now()) {}

bool FaultSupervisor::CooldownReady() const {
  if (last_command_time_.time_since_epoch().count() == 0) {
    return true;
  }
  const auto now = std::chrono::steady_clock::now();
  return (now - last_command_time_) >= config_.command_cooldown;
}

void FaultSupervisor::UpdateHealth(bool system_healthy, const std::string& error_message) {
  MultiSourceHealthReport report;
  report.chassis_healthy = system_healthy;
  report.watchdog_healthy = system_healthy;
  report.detail_message = error_message;
  UpdateMultiSourceHealth(report);
}

void FaultSupervisor::UpdateMultiSourceHealth(const MultiSourceHealthReport& report) {
  health_seen_ = true;
  latest_report_ = report;
}

void FaultSupervisor::Tick() {
  const auto now = std::chrono::steady_clock::now();

  if (!health_seen_) {
    state_ = SupervisorState::kWaiting;
    recommendation_ = SystemActionRecommendation::kNone;
    message_ = "waiting for health signal";
    return;
  }

  const bool in_grace = (now - start_time_) < config_.startup_grace;

  // 1. 判断是否属于严重硬件/人身故障 (E-Stop, Bumper, Watchdog 丢失, 高危丢包率 > 35%)
  bool critical_fault = latest_report_.manual_estop ||
                        !latest_report_.watchdog_healthy ||
                        (latest_report_.chassis_loss_rate >= config_.critical_loss_rate_threshold);

  if (critical_fault) {
    if (in_grace && !latest_report_.manual_estop) {
      state_ = SupervisorState::kArming;
      recommendation_ = SystemActionRecommendation::kNone;
      message_ = "startup grace active: " + latest_report_.detail_message;
      return;
    }

    state_ = SupervisorState::kFault;
    recommendation_ = SystemActionRecommendation::kEmergencyStop;
    message_ = "critical safety fault: " + latest_report_.detail_message;

    if (!estop_requested_ && CooldownReady()) {
      estop_requested_ = true;
      last_command_time_ = now;
    }
    return;
  }

  // 2. 判断是否属于死锁阻塞 (死锁发生时建议暂缓派单)
  if (latest_report_.deadlock_detected) {
    state_ = SupervisorState::kDegraded;
    recommendation_ = SystemActionRecommendation::kPauseDispatch;
    message_ = "topology deadlock active, pausing new dispatches";
    return;
  }

  // 3. 判断是否属于通信降级 (中度丢包 15%~35%)
  if (!latest_report_.chassis_healthy ||
      (latest_report_.chassis_loss_rate >= config_.degraded_loss_rate_threshold)) {
    state_ = SupervisorState::kDegraded;
    recommendation_ = SystemActionRecommendation::kClampSpeed;
    message_ = "communication degraded, speed clamped";
    return;
  }

  // 4. 健康指标完全恢复正常
  if (state_ == SupervisorState::kFault) {
    if (config_.auto_clear) {
      state_ = SupervisorState::kRecovery;
      recommendation_ = SystemActionRecommendation::kClampSpeed; // 恢复观察期维持降级限速
      message_ = "system recovered, observing stabilization";
      estop_requested_ = false;
      last_command_time_ = now;
    }
  } else if (state_ == SupervisorState::kRecovery || state_ == SupervisorState::kArming ||
             state_ == SupervisorState::kWaiting || state_ == SupervisorState::kDegraded) {
    state_ = SupervisorState::kNormal;
    recommendation_ = SystemActionRecommendation::kNone;
    message_ = "nominal operation";
    estop_requested_ = false;
  }
}

std::string FaultSupervisor::state_string() const {
  switch (state_) {
    case SupervisorState::kWaiting: return "WAITING";
    case SupervisorState::kArming: return "ARMING";
    case SupervisorState::kNormal: return "NORMAL";
    case SupervisorState::kDegraded: return "DEGRADED";
    case SupervisorState::kFault: return "FAULT";
    case SupervisorState::kRecovery: return "RECOVERY";
  }
  return "UNKNOWN";
}

void FaultSupervisor::Reset() {
  state_ = SupervisorState::kWaiting;
  recommendation_ = SystemActionRecommendation::kNone;
  message_ = "reset";
  health_seen_ = false;
  estop_requested_ = false;
  start_time_ = std::chrono::steady_clock::now();
}

}  // namespace amr_dispatcher_core::safety
