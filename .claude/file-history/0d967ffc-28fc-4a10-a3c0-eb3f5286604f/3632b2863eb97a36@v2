#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "amr_dispatcher_core/chassis/chassis_backend.hpp"
#include "amr_dispatcher_core/chassis/chassis_backend_factory.hpp"
#include "amr_dispatcher_core/chassis/link_quality.hpp"
#include "amr_dispatcher_core/chassis/link_recovery.hpp"

namespace amr_dispatcher_core::chassis {

// 后端降级配置
struct BackendDegradationConfig {
  std::size_t warmup_samples = 8;
  double demote_loss_rate = 0.30;
  std::chrono::milliseconds cooldown{std::chrono::seconds(30)};
};

// 控制器：维护 preferred list，按 LinkQualityMonitor 状态在串行后端间自动降级，
// 链路恢复信号触发回切首选；连接丢失时经 LinkRecoveryPolicy 按指数退避扫描重连。
// 调用方驱动 Record* 与 EvaluateAndMaybeSwitch；切换时关闭旧 backend 并 Open 新 backend。
class BackendDegradationController {
 public:
  BackendDegradationController(
      std::vector<BackendDescriptor> backends,
      std::vector<ChassisBackendConfig> configs,
      BackendDegradationConfig config = {});

  void RecordSuccess(double latency_ms);
  void RecordLoss();
  void RecordCrcError();

  // 评估链路质量，必要时切换后端。reason 输出降级/回切/重连原因（可空）。
  // 返回 true 表示发生了切换。
  bool EvaluateAndMaybeSwitch(std::string* reason);

  std::unique_ptr<ChassisBackend>& current() { return current_; }
  const std::unique_ptr<ChassisBackend>& current() const { return current_; }
  std::size_t current_index() const { return current_index_; }

  const LinkQualityMonitor& quality() const { return quality_; }

  // 强行切换到 index（用于测试或运维指令）
  bool SwitchTo(std::size_t index, std::string* reason);

 private:
  // 重连 open_attempt：按 preferred 顺序扫描，首个可打开的后端即为重连成功。
  bool TryOpenAnyBackend(std::string* reason);

  std::vector<BackendDescriptor> backends_;
  std::vector<ChassisBackendConfig> configs_;
  BackendDegradationConfig config_;
  LinkQualityMonitor quality_;
  std::unique_ptr<ChassisBackend> current_;
  std::size_t current_index_ = 0;
  std::chrono::steady_clock::time_point last_switch_at_{};  // 最近一次切换（降档或回切），冷却窗基准
  std::unique_ptr<LinkRecoveryPolicy> recovery_;            // 连接丢失后的指数退避重连策略
  std::chrono::steady_clock::time_point reconnect_not_before_{};  // 下一次重连尝试的最早时刻
};

}  // namespace amr_dispatcher_core::chassis
