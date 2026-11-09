#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>

namespace amr_dispatcher_core::chassis {

// T1：底盘电压 -> 选车/派单可用的"电量百分比"状态机（纯逻辑，可脱离 ROS 单测）。
//
// 为什么需要它
// ------------
// `/battery_state` 此前**没有任何订阅者**：chassis_driver 发得很勤，
// 但调度侧的 `vehicle_states_[].battery_percent` 对 local_sim 车是写死 100，
// 只有 VDA 5050 车才从 state 报文更新。后果是台架上
//   * min_battery_percent 永远放行；
//   * auto_charging_* 永远不触发。
// 数字看起来"正常"，功能其实整条是断的——比报错更难发现。
//
// 语义
// ----
// * 只接受**新鲜**的电压样本：底盘掉线后不能拿最后一帧电压当现状；
// * 电压换算成百分比是纯线性折算（见 battery_voltage.hpp），不含放电曲线，
//   台架演示足够，真机应由 BMS 直接给 SOC；
// * `has_measurement` 与 `percent` 分离：前者表示"有没有可信数据"，
//   后者是数值。从未收到过电压时 percent 保持 0 而不是 100 ——
//   未知电量绝不能被当成满电（那会让低电量保护静默失效）。
class BatteryStateTracker {
 public:
  struct Config {
    double full_voltage = 12.6;    // 满电端点（12V 台架；24V 整车 25.2）
    double empty_voltage = 11.0;   // 空电端点（24V 整车 21.0）
    // 样本保鲜期：超过该时长的电压不再代表现状。
    // 必须小于 vehicle_state_max_age_ms（默认 5s），否则调度会把陈旧电量当新鲜的。
    std::uint64_t max_age_ms = 3000;
  };

  BatteryStateTracker() = default;
  explicit BatteryStateTracker(Config config) : config_(config) {}

  // 收到一帧电压（单位 V）。`now_ms` 用单调时钟，由调用方提供。
  void OnVoltage(double voltage_v, std::uint64_t now_ms);

  // 该电量现在是否可用于选车/充电决策。
  bool IsFresh(std::uint64_t now_ms) const;

  bool has_measurement() const { return has_measurement_; }
  double LastVoltage() const { return last_voltage_v_; }
  // 未收到过样本（或配置非法）时返回 0。
  double Percent() const { return percent_; }
  std::uint64_t LastSampleMs() const { return last_sample_ms_; }

  void Reset();

 private:
  double ComputePercent(double voltage_v) const;

  Config config_;
  bool has_measurement_ = false;
  double last_voltage_v_ = 0.0;
  double percent_ = 0.0;
  std::uint64_t last_sample_ms_ = 0;
};

}  // namespace amr_dispatcher_core::chassis
