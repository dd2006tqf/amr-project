#pragma once

#include <cstdint>

#include "amr_dispatcher_core/chassis/chassis_packet.hpp"

namespace amr_dispatcher_ros {

// T2：底盘上报的"不可继续运动"判据（纯函数，可脱离 rclcpp 单测）。
//
// 背景
// ----
// 0xA1 的 flags 与 0xA2 的 fault 位此前只被映射成一行日志：
//   * HandleState 比对字符串 "fault" 打印 ERROR；
//   * HandleOdometry 完全不看 flags。
// 结果是 MCU 报过流/UART 超时/急停输入时，车照常被派单、照常跑。
//
// 设计取舍
// --------
// 这里**不新增停机通路**，只回答"这一帧是否应当把链路判为不可信"。
// 一旦为 true，chassis_driver 把 link_alive_ 置 false，心跳随之下沉为
// false，由既有的 safety_gate 500ms 看门狗接管零速——复用已验证的链路。
//
// 为什么 estop_input 也算故障：部分底盘会把物理急停的状态回传，而急停
// 是按下的常态而非瞬态。若只在"输入有效期间"降级，旋出急停后需要重新
// 建立链路，这符合安全语义（人按了急停，就不该自动恢复行驶）。
//
// 为什么 over_current 算：TB6612 触发热保护时 PWM 已被硬件切断，此时
// ROS 侧还以为车在按指令走，是最危险的组合。
inline bool IsChassisFault(const uint8_t flags) {
  using namespace amr_dispatcher_core::chassis;
  return HasMickFlag(flags, kFault) || HasMickFlag(flags, kUartTimeout) ||
         HasMickFlag(flags, kEstopInput) || HasMickFlag(flags, kOverCurrent);
}

// 0xA2 的 fault 位：与 flags 同义，合并成一个判据。
inline bool IsChassisFault(const amr_dispatcher_core::chassis::ChassisStatePacket& state) {
  return state.fault;
}

}  // namespace amr_dispatcher_ros
