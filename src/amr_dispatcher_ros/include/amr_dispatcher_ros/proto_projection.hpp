// rosidl 消息 -> protobuf 消息的集中转换层。
//
// ============================================================================
// 为什么要有这一层，而不是在每个节点里就地写转换
// ============================================================================
// 阶段 2 的验收标准是「proto 与 rosidl 两条链路逐字段一致」。若每个发布点
// 各自写一遍 msg.x -> proto.set_x(...)，两份代码必然缓慢漂移：某个字段后来
// 在 rosidl 侧加了，proto 侧忘记加，编译器不会报错（proto3 的 setter 存在
// 即可），运行时也不报错，只有对比工具或下游消费者才会发现。
//
// 这里的做法是把转换收敛成**纯函数**，并满足三个性质：
//
//   1. 纯函数：不读时钟、不读成员变量、不抛异常（除 protobuf 自身分配失败）。
//      因此单测可以直接喂构造好的输入，不依赖 ROS 运行时。
//   2. 单一来源：发布点只调用一次，用它填 proto 消息。rosidl 消息与 proto
//      消息都从**同一个上游结构**派生，不存在「两次独立构造」。
//   3. 可穷举校验：每个 message 的每个字段都必须出现在 To*() 里，否则
//      test_proto_projection.cpp 的字段清单断言会失败。
//
// ============================================================================
// 为什么要处理「-1 哨兵」和「字符串枚举」
// ============================================================================
// 两处 rosidl -> proto 不是逐字复制，必须显式翻译，否则会静默出错：
//
//   ChassisLinkHealth.command_delivery_rate
//     .msg 里 -1 表示「不适用」（文本协议 / MCU 未回显）。proto3 无符号类型
//     不能表达负数，proto 侧拆成 (valid, rate) 两个字段。若不翻译，-1 会被
//     写成 proto 的 -1.0（double 能存），但 valid 位永远 true，消费者会把
//     「不适用」误读成「命令全丢」。
//
//   ChassisLinkHealth.tier / SafetyState.severity
//     .msg 里是自由字符串。proto 侧是 enum + *_text 兜底字段。翻译规则：
//     已知值填 enum，未知值枚举留 UNKNOWN 且把原文写进 *_text，保证信息不丢。
//     这条规则写在 ToTierEnum / ToSeverityEnum 里，两侧共用同一实现。
#ifndef AMR_DISPATCHER_ROS__PROTO_PROJECTION_HPP_
#define AMR_DISPATCHER_ROS__PROTO_PROJECTION_HPP_

#include <cstdint>
#include <string>

#include "amr_dispatcher_interfaces/msg/chassis_link_health.hpp"
#include "amr_dispatcher_interfaces/msg/dispatcher_state.hpp"
#include "amr_dispatcher_interfaces/msg/mission_event.hpp"
#include "amr_dispatcher_interfaces/msg/safety_state.hpp"
#include "amr_dispatcher_interfaces/msg/topology_state.hpp"
#include "amr_dispatcher_interfaces/msg/tracking_error.hpp"
#include "amr_dispatcher_interfaces/msg/vehicle_link_state.hpp"

#include "amr_dispatcher_proto/generated/chassis.pb.h"
#include "amr_dispatcher_proto/generated/dispatcher.pb.h"
#include "amr_dispatcher_proto/generated/safety.pb.h"

namespace amr_dispatcher_ros
{
namespace proto_projection
{

// ---------------------------------------------------------------------------
// 枚举翻译
// ---------------------------------------------------------------------------
// 这两组函数是 rosidl 字符串 <-> proto enum 的唯一翻译点。发布端与检查端
// 都用它，因此「未知值写进 *_text」的规则只有一处实现。
//
// 不做「大小写不敏感」之类的宽容匹配：.msg 的生产者只有本项目两个节点，
// 取值由代码写死。宽容匹配反而会掩盖拼写 bug。

/// "HEALTHY" -> TIER_HEALTHY；未知值 -> TIER_UNKNOWN（调用方负责填 tier_text）。
amr_dispatcher::proto::ChassisLinkHealth::Tier ToTierEnum(const std::string& tier);

/// proto enum -> 规范字符串（写回 rosidl 时用；未知 enum 值返回 "UNKNOWN"）。
std::string ToTierString(amr_dispatcher::proto::ChassisLinkHealth::Tier tier);

/// "INFO" / "WARN" / "CRITICAL" -> SEVERITY_*；未知值 -> SEVERITY_UNKNOWN。
amr_dispatcher::proto::SafetyState::Severity ToSeverityEnum(const std::string& severity);

/// proto enum -> 规范字符串。
std::string ToSeverityString(amr_dispatcher::proto::SafetyState::Severity severity);

// ---------------------------------------------------------------------------
// Header / Time
// ---------------------------------------------------------------------------
// builtin_interfaces/Time 与 std_msgs/Header 是 ROS 生态的标准结构，
// proto 侧定义了结构完全一致的 Time / Header，桥接时只搬数字，不做时区/单位换算。

void ProjectStamp(const builtin_interfaces::msg::Time& in, amr_dispatcher::proto::Time* out);

void ProjectHeader(const std_msgs::msg::Header& in, amr_dispatcher::proto::Header* out);

// ---------------------------------------------------------------------------
// 7 个消息的投影
// ---------------------------------------------------------------------------
// 约定：out 必须是**新建或已 Clear()** 的消息。函数内部不做 Clear，
// 因为发布点的常见写法是栈上构造一个新的（默认即空），Clear 是多余开销；
// 但复用同一对象时必须由调用方先 Clear，否则 repeated 字段会累积。

void ProjectChassisLinkHealth(
  const amr_dispatcher_interfaces::msg::ChassisLinkHealth& in,
  amr_dispatcher::proto::ChassisLinkHealth* out);

void ProjectTrackingError(
  const amr_dispatcher_interfaces::msg::TrackingError& in,
  amr_dispatcher::proto::TrackingError* out);

void ProjectVehicleLinkState(
  const amr_dispatcher_interfaces::msg::VehicleLinkState& in,
  amr_dispatcher::proto::VehicleLinkState* out);

void ProjectDispatcherState(
  const amr_dispatcher_interfaces::msg::DispatcherState& in,
  amr_dispatcher::proto::DispatcherState* out);

void ProjectSafetyState(
  const amr_dispatcher_interfaces::msg::SafetyState& in,
  amr_dispatcher::proto::SafetyState* out);

void ProjectTopologyState(
  const amr_dispatcher_interfaces::msg::TopologyState& in,
  amr_dispatcher::proto::TopologyState* out);

void ProjectMissionEvent(
  const amr_dispatcher_interfaces::msg::MissionEvent& in,
  amr_dispatcher::proto::MissionEvent* out);

}  // namespace proto_projection
}  // namespace amr_dispatcher_ros

#endif  // AMR_DISPATCHER_ROS__PROTO_PROJECTION_HPP_
