#pragma once

#include <chrono>
#include <string>

namespace amr_dispatcher_core::safety {

enum class SupervisorState {
  kWaiting,   // 启动等待健康数据
  kArming,    // 启动宽限期（不误报故障）
  kNormal,    // 正常巡航
  kFault,     // 故障激活（已触发急停）
  kRecovery   // 故障已清除，自愈恢复等待稳定
};

struct SupervisorConfig {
  std::chrono::milliseconds startup_grace{std::chrono::seconds(2)};
  std::chrono::milliseconds command_cooldown{std::chrono::milliseconds(500)};
  bool auto_clear = true;
};

class FaultSupervisor {
 public:
  explicit FaultSupervisor(SupervisorConfig config = {});

  // 接收系统健康心跳与状态输入
  void UpdateHealth(bool system_healthy, const std::string& error_message);

  // 核心周期巡检 Tick
  void Tick();

  SupervisorState state() const { return state_; }
  std::string state_string() const;
  std::string message() const { return message_; }
  bool emergency_stop_requested() const { return estop_requested_; }

  void Reset();

 private:
  SupervisorConfig config_;
  SupervisorState state_ = SupervisorState::kWaiting;
  std::string message_ = "waiting for health signal";

  bool health_seen_ = false;
  bool last_system_healthy_ = false;
  std::string last_error_message_;

  std::chrono::steady_clock::time_point start_time_;
  std::chrono::steady_clock::time_point last_command_time_;
  bool estop_requested_ = false;
};

}  // namespace amr_dispatcher_core::safety
