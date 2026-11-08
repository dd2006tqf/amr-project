#include "amr_dispatcher_core/safety/cmd_vel_gate.hpp"

#include <algorithm>
#include <cmath>
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
  // 非有限值兜底：NaN 属于无效指令直接归零；±Inf 视为超限走下方钳位
  if (std::isnan(value)) return 0.0;
  if (std::abs(value) <= max_abs) return value;
  return (value > 0 ? max_abs : -max_abs);
}

}  // namespace

CmdVelGateDecision CmdVelGate::Evaluate(const CmdVelCommand& command,
                                        std::chrono::steady_clock::time_point now) const {
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
  // 事件带 TTL：超过 stamp + event_ttl 的历史事件视为已解除，不再影响当前决策
  bool critical = false;
  bool warned = false;
  std::string critical_reason;
  for (const auto& e : recent_events_) {
    if (now - e.stamp > config_.event_ttl) {
      continue;  // 事件已过期
    }
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
