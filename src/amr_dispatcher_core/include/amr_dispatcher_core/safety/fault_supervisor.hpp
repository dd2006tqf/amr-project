#pragma once

#include <chrono>
#include <string>
#include <vector>

namespace amr_dispatcher_core::safety {

enum class SupervisorState {
  kWaiting,   // 启动等待健康数据
  kArming,    // 启动宽限期（不误报故障）
  kNormal,    // 正常巡航 (全部指标正常)
  kDegraded,  // 降级巡航 (如轻微丢包/传感器延迟，建议平滑限速)
  kFault,     // 严重故障 (触发 E-Stop 硬切断驱动)
  kRecovery   // 故障已清除，自愈恢复观察期
};

enum class SystemActionRecommendation {
  kNone,          // 正常放行
  kClampSpeed,    // 降级限速 (如线速度限制到 0.2 m/s)
  kPauseDispatch, // 调度器暂缓接单 / 挂起高风险任务
  kEmergencyStop  // 安全门硬切断速度输出 (Zero Output)
};

struct MultiSourceHealthReport {
  bool chassis_healthy = true;
  double chassis_loss_rate = 0.0;
  bool watchdog_healthy = true;
  std::string missing_heartbeat_node;
  bool deadlock_detected = false;
  bool manual_estop = false;
  std::string detail_message;
};

struct SupervisorConfig {
  std::chrono::milliseconds startup_grace{std::chrono::seconds(2)};
  std::chrono::milliseconds command_cooldown{std::chrono::milliseconds(500)};
  double degraded_loss_rate_threshold = 0.15; // 丢包率 > 15% 触发 kDegraded 限速
  double critical_loss_rate_threshold = 0.35; // 丢包率 > 35% 触发 kFault 急停
  bool auto_clear = true;
};

/**
 * @brief 工业级全链路系统故障监督中心 (Fault Supervisor Engine)
 * 聚合底层通信丢包、看门狗心跳、死锁报警、人工急停四大防御数据源，
 * 输出统一的系统健康评级与多级联动动作建议 (Action Recommendation)。
 */
class FaultSupervisor {
 public:
  explicit FaultSupervisor(SupervisorConfig config = {});

  // 1. 基础健康上报接口 (向后兼容)
  void UpdateHealth(bool system_healthy, const std::string& error_message);

  // 2. 多数据源聚合上报接口
  void UpdateMultiSourceHealth(const MultiSourceHealthReport& report);

  // 核心周期巡检 Tick (建议 10Hz~50Hz 调用)
  void Tick();

  SupervisorState state() const { return state_; }
  std::string state_string() const;
  std::string message() const { return message_; }
  bool emergency_stop_requested() const { return estop_requested_; }

  SystemActionRecommendation recommendation() const { return recommendation_; }

  void Reset();

 private:
  bool CooldownReady() const;

  SupervisorConfig config_;
  SupervisorState state_ = SupervisorState::kWaiting;
  SystemActionRecommendation recommendation_ = SystemActionRecommendation::kNone;
  std::string message_ = "waiting for health signal";

  bool health_seen_ = false;
  MultiSourceHealthReport latest_report_;

  std::chrono::steady_clock::time_point start_time_;
  std::chrono::steady_clock::time_point last_command_time_;
  bool estop_requested_ = false;
};

}  // namespace amr_dispatcher_core::safety
