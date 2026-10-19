#pragma once

#include <chrono>
#include <cstddef>
#include <functional>
#include <string>
#include <vector>

namespace amr_dispatcher_core::safety {

enum class SafetySeverity { kInfo, kWarn, kCritical };

struct SafetyEvent {
  std::string source;
  SafetySeverity severity = SafetySeverity::kInfo;
  std::string message;
  std::chrono::steady_clock::time_point stamp = std::chrono::steady_clock::now();
};

struct CmdVelCommand {
  double linear_x_mps = 0.0;
  double linear_y_mps = 0.0;
  double angular_z_radps = 0.0;
  std::chrono::steady_clock::time_point stamp = std::chrono::steady_clock::now();
};

struct CmdVelGateDecision {
  double linear_x_mps = 0.0;
  double linear_y_mps = 0.0;
  double angular_z_radps = 0.0;
  bool allowed = true;
  std::string reason;
  SafetySeverity severity = SafetySeverity::kInfo;
};

// cmd_vel_gate 配置：上游源（bumper / estop / ...）通过 source_blocked 回调告知是否激活阻止
struct CmdVelGateConfig {
  double max_linear_x_mps = 1.5;
  double max_linear_y_mps = 1.0;
  double max_angular_z_radps = 2.0;
  std::function<bool(const std::string&)> source_blocked;
  std::size_t event_history_capacity = 256;
  // 事件 TTL：Evaluate 时刻超过 stamp + event_ttl 的事件视为已失效，不再触发阻止/降速
  std::chrono::milliseconds event_ttl{std::chrono::milliseconds(2000)};
};

class CmdVelGate {
 public:
  explicit CmdVelGate(CmdVelGateConfig config = {});

  void PushEvent(SafetyEvent event);
  void ClearEvents();

  // now 用于事件 TTL 判定；默认取当前时刻（与旧行为兼容）
  CmdVelGateDecision Evaluate(
      const CmdVelCommand& command,
      std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now()) const;

  const std::vector<SafetyEvent>& recent_events() const { return recent_events_; }

 private:
  CmdVelGateConfig config_;
  std::vector<SafetyEvent> recent_events_;
};

}  // namespace amr_dispatcher_core::safety
