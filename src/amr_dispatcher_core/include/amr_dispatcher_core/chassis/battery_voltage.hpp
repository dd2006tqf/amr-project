#pragma once

#include <algorithm>

namespace amr_dispatcher_core::chassis {

// 把底盘上报的母线电压折算成 0-100 的电量百分比。
//
// 为什么需要这层：项目的下游全部按 percent 判定（dispatcher 的
// min_battery_percent、auto_charging_*_threshold_percent），而底盘只给电压。
// 直接用固定电压阈值会让 12V 台架与 24V 整车的配置无法共存，
// 所以端点做成参数（battery_full_voltage / battery_empty_voltage）。
//
// 注意这是线性折算，不含锂电池放电曲线。台架演示足够，
// 真机若要精确 SOC 应由 BMS 直接上报百分比。
inline double BatteryVoltageToPercent(double voltage, double full_voltage, double empty_voltage) {
  if (voltage <= 0.0) {
    return 0.0;
  }
  // 端点配反属于配置错误：退化为"只做 0/100 钳位"，不返回 NaN 或负值。
  // 阈值取两端较大者，语义是"电压够高就算满"，避免把 empty 值当成满电门槛。
  if (full_voltage <= empty_voltage) {
    return voltage >= std::max(full_voltage, empty_voltage) ? 100.0 : 0.0;
  }
  const double ratio = (voltage - empty_voltage) / (full_voltage - empty_voltage);
  return std::clamp(ratio * 100.0, 0.0, 100.0);
}

}  // namespace amr_dispatcher_core::chassis
