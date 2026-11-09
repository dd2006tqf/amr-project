#include "amr_dispatcher_core/chassis/battery_state_tracker.hpp"

#include "amr_dispatcher_core/chassis/battery_voltage.hpp"

namespace amr_dispatcher_core::chassis {

void BatteryStateTracker::OnVoltage(const double voltage_v, const std::uint64_t now_ms) {
  // 非正电压属于"没有数据"而不是"0V"：底盘未接电池时 ADC 读到 0，
  // 若当成 0% 会让 auto_charging 立刻派充电单，属于误动作。
  if (!(voltage_v > 0.0)) {
    return;
  }
  has_measurement_ = true;
  last_voltage_v_ = voltage_v;
  percent_ = ComputePercent(voltage_v);
  last_sample_ms_ = now_ms;
}

bool BatteryStateTracker::IsFresh(const std::uint64_t now_ms) const {
  if (!has_measurement_) {
    return false;
  }
  // 时钟回退（now < last）视为新鲜：宁可短暂沿用旧值，也不因一次异常时间戳
  // 把整车的电量判成未知而导致派单抖动。
  if (now_ms < last_sample_ms_) {
    return true;
  }
  return (now_ms - last_sample_ms_) <= config_.max_age_ms;
}

double BatteryStateTracker::ComputePercent(const double voltage_v) const {
  // 复用既有的线性折算（含端点写反的退化处理），保证与 /battery_state 的
  // percentage 字段逐位一致——同一个电压在两条路径上不能算出两个数。
  return BatteryVoltageToPercent(voltage_v, config_.full_voltage, config_.empty_voltage);
}

void BatteryStateTracker::Reset() {
  has_measurement_ = false;
  last_voltage_v_ = 0.0;
  percent_ = 0.0;
  last_sample_ms_ = 0;
}

}  // namespace amr_dispatcher_core::chassis
