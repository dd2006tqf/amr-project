#include "amr_dispatcher_core/safety/cmd_vel_gate.hpp"

#include <algorithm>
#include <utility>

namespace amr_dispatcher_core::safety {

CmdVelGate::CmdVelGate(CmdVelGateConfig config) : config_(std::move(config)) {}

void CmdVelGate::PushEvent(SafetyEvent event) {
  recent_events_.push_back(std::move(event));
  if (recent_events_.size() > config_.event_history_capacity) {
    recent_events_.erase(recent_events_.begin());
  }
}

void CmdVelGate::ClearEvents() { recent_events_.clear(); }

namespace {

double Scale(double value, double max_abs) {
  if (std::abs(value) <= max_abs) return value;
  return (value > 0 ? max_abs : -max_abs);
}

}  // namespace

CmdVelGateDecision CmdVelGate::Evaluate(const CmdVelCommand& command) const {
  CmdVelGateDecision decision;
  decision.linear_x_mps = command.linear_x_mps;
  decision.linear_y_mps = command.linear_y_mps;
  decision.angular_z_radps = command.angular_z_radps;
  decision.allowed = true;

  // 1) 上限缩放
  decision.linear_x_mps = Scale(decision.linear_x_mps, config_.max_linear_x_mps);
  decision.linear_y_mps = Scale(decision.linear_y_mps, config_.max_linear_y_mps);
  decision.angular_z_radps = Scale(decision.angular_z_radps, config_.max_angular_z_radps);

  // 2) 多源阻止：critical 事件 → 全停；warn → 缩放至 0.3 倍
  bool critical = false;
  bool warned = false;
  std::string critical_reason;
  for (const auto& e : recent_events_) {
    if (e.severity == SafetySeverity::kCritical) {
      if (config_.source_blocked && config_.source_blocked(e.source)) {
        critical = true;
        critical_reason = e.source + ": " + e.message;
      }
    } else if (e.severity == SafetySeverity::kWarn) {
      warned = true;
    }
  }

  if (critical) {
    decision.allowed = false;
    decision.linear_x_mps = 0.0;
    decision.linear_y_mps = 0.0;
    decision.angular_z_radps = 0.0;
    decision.reason = critical_reason;
    decision.severity = SafetySeverity::kCritical;
    return decision;
  }
  if (warned) {
    decision.linear_x_mps *= 0.3;
    decision.linear_y_mps *= 0.3;
    decision.angular_z_radps *= 0.3;
    decision.severity = SafetySeverity::kWarn;
    decision.reason = "warn active: reducing to 30%";
  }
  return decision;
}

}  // namespace amr_dispatcher_core::safety
