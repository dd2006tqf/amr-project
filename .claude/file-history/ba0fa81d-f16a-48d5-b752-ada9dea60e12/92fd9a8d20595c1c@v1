#include "amr_dispatcher_core/safety/fault_supervisor.hpp"

#include <vector>

namespace amr_dispatcher_core::safety {

namespace {

// 四个防御数据源的固定键（与 last_report_time_ 对应）
constexpr const char* kSourceChassis = "chassis";
constexpr const char* kSourceWatchdog = "watchdog";
constexpr const char* kSourceDeadlock = "deadlock";
constexpr const char* kSourceEstop = "estop";

}  // namespace

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
  // 基础接口只承载 chassis/watchdog 两个源，不代打 deadlock/estop 时戳，
  // 使从未上报的源不参与时效判定（避免被误判为过期急停/死锁）。
  health_seen_ = true;
  latest_report_ = MultiSourceHealthReport{};
  latest_report_.chassis_healthy = system_healthy;
  latest_report_.watchdog_healthy = system_healthy;
  latest_report_.detail_message = error_message;
  const auto now = std::chrono::steady_clock::now();
  last_report_time_[kSourceChassis] = now;
  last_report_time_[kSourceWatchdog] = now;
}

void FaultSupervisor::UpdateMultiSourceHealth(const MultiSourceHealthReport& report) {
  health_seen_ = true;
  latest_report_ = report;
  // 聚合上报视为四个源同时刷新
  const auto now = std::chrono::steady_clock::now();
  last_report_time_[kSourceChassis] = now;
  last_report_time_[kSourceWatchdog] = now;
  last_report_time_[kSourceDeadlock] = now;
  last_report_time_[kSourceEstop] = now;
}

void FaultSupervisor::Tick() { Tick(std::chrono::steady_clock::now()); }

void FaultSupervisor::Tick(std::chrono::steady_clock::time_point now) {
  if (!health_seen_) {
    state_ = SupervisorState::kWaiting;
    recommendation_ = SystemActionRecommendation::kNone;
    message_ = "waiting for health signal";
    return;
  }

  const bool in_grace = (now - start_time_) < config_.startup_grace;

  // 0. 上报时效：缺失记录 = 从未上报，不参与判定；超时 = 该源数据不可信。
  //    （不可信不等于故障：过期源不能触发急停，但其"健康"结论也不再采信）
  const auto stale = [&](const char* source) {
    const auto it = last_report_time_.find(source);
    if (it == last_report_time_.end()) {
      return false;  // 从未上报的源不参与时效判定
    }
    return (now - it->second) > config_.staleness_timeout;
  };
  const bool chassis_stale = stale(kSourceChassis);
  const bool watchdog_stale = stale(kSourceWatchdog);
  const bool deadlock_stale = stale(kSourceDeadlock);
  const bool estop_stale = stale(kSourceEstop);
  const bool any_stale = chassis_stale || watchdog_stale || deadlock_stale || estop_stale;

  // 1. 判断是否属于严重硬件/人身故障 (E-Stop, Bumper, Watchdog 丢失, 高危丢包率 > 35%)
  //    人工急停/看门狗按最后已知值锁存（安全信号不因报告过期而清除）；
  //    高危丢包率只在该源数据仍新鲜时采信，过期数据不升级为急停。
  bool critical_fault = latest_report_.manual_estop ||
                        !latest_report_.watchdog_healthy ||
                        (!chassis_stale &&
                         latest_report_.chassis_loss_rate >= config_.critical_loss_rate_threshold);

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

  // 1.5 上报时效仲裁：任一源过期 -> 降级限速（不采信过期源的健康结论，
  //     也不将过期数据升级为急停/死锁，避免"停止上报"反而触发硬故障）。
  if (any_stale) {
    std::vector<std::string> stale_sources;
    if (chassis_stale) stale_sources.emplace_back(kSourceChassis);
    if (watchdog_stale) stale_sources.emplace_back(kSourceWatchdog);
    if (deadlock_stale) stale_sources.emplace_back(kSourceDeadlock);
    if (estop_stale) stale_sources.emplace_back(kSourceEstop);
    std::string joined;
    for (std::size_t i = 0; i < stale_sources.size(); ++i) {
      if (i > 0) joined += ",";
      joined += stale_sources[i];
    }
    state_ = SupervisorState::kDegraded;
    recommendation_ = SystemActionRecommendation::kClampSpeed;
    message_ = "health report stale: " + joined;
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
  last_report_time_.clear();  // 各源时效时戳一并清空，重新等待上报
  start_time_ = std::chrono::steady_clock::now();
}

}  // namespace amr_dispatcher_core::safety
