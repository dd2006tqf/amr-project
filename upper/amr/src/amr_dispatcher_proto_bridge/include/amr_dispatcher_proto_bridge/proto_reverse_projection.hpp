// protobuf 消息 -> rosidl 消息的反向投影（阶段 3：消费者迁移）。
//
// ============================================================================
// 为什么需要反向投影，而不是让消费者直接读 protobuf
// ============================================================================
// 阶段 3 的验收标准是「消费者改读 proto 话题，且业务行为逐字不变」。若让
// 消费者直接吃 protobuf，`rest_gateway`、`safety_gate`、`dispatcher_node` 里
// 合计 60+ 处字段访问要全部改写，且每一处都可能引入语义漂移（例如
// `severity == "CRITICAL"` 要改成 `severity() == SEVERITY_CRITICAL`，
// 而 `severity.empty()` 这种「还没收到消息」的判据在 proto 侧根本没有对应
// 表达——enum 默认值是 SEVERITY_UNKNOWN）。
//
// 反向投影把 proto 消息**还原成 rosidl 消息**，再交给既有的 rosidl 回调。
// 于是：
//   - 业务逻辑一行不改，迁移风险降到最低；
//   - 消费者可以先切一条话题、观察一个版本周期，随时切回；
//   - 阶段 4 若真要删 rosidl 侧，删除的是「投影层 + 话题」，不是散落的业务代码。
//
// ============================================================================
// 严格互逆：三处有损点必须逐条还原
// ============================================================================
// 正向投影（amr_dispatcher_ros/proto_projection.cpp）不是逐字复制，反向必须
// 把每一条翻译规则反过来，否则往返测试会失败：
//
//   1. ChassisLinkHealth.command_delivery_rate
//      正向：in < 0.0  ->  valid=false, rate=0.0
//            in >= 0.0 ->  valid=true,  rate=in
//      反向：valid=false ->  -1.0（.msg 里 -1 才是「不适用」的哨兵）
//            valid=true  ->  rate
//      **不能**把 valid=false 还原成 0.0：那表示「命令送达率 0%」，是
//      完全相反的运维结论（全丢 vs 不适用）。
//
//   2. ChassisLinkHealth.tier
//      正向：已知值 -> enum 且 tier_text 留空
//            未知值 -> TIER_UNKNOWN 且 tier_text = 原文
//      反向：已知 enum -> 规范字符串
//            TIER_UNKNOWN -> tier_text 非空则用 tier_text，否则 "UNKNOWN"
//      **不能**在 TIER_UNKNOWN 时直接返回 "UNKNOWN"：那会丢失上游的原始
//      自定义值（例如未来新增的 "SATURATED" 档）。
//
//   3. SafetyState.severity
//      与 tier 完全同构（severity_text 兜底）。
//
// 另有一个**不可逆**的边界，写在这里以免误用：
//   rosidl 侧「默认构造、从未收到消息」时 severity 是空字符串，而 proto 侧
//   enum 默认值 SEVERITY_UNKNOWN 反投影出来是 "UNKNOWN"，两者不等。因此
//   消费者里任何依赖 `severity.empty()` 表达「没收到过」的判断，必须在
//   切换时改看自己的 *_seen_ 布尔量（各消费者本来就有），**不能**指望
//   反向投影区分二者。这条已在实际消费者（rest_gateway）核实过。
#ifndef AMR_DISPATCHER_PROTO_BRIDGE__PROTO_REVERSE_PROJECTION_HPP_
#define AMR_DISPATCHER_PROTO_BRIDGE__PROTO_REVERSE_PROJECTION_HPP_

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

namespace amr_dispatcher_proto_bridge
{
namespace reverse_projection
{

// ---------------------------------------------------------------------------
// 枚举翻译（与 amr_dispatcher_ros/proto_projection.cpp 的 To*Enum 互逆）
// ---------------------------------------------------------------------------
// 这两组函数单独暴露，是为了让单测能直接验证「未知值 -> *_text」的回退
// 规则，而不必构造整个消息。

/// proto enum -> rosidl 字符串。
///
/// \param tier      proto 侧枚举值
/// \param tier_text proto 侧兜底原文。仅当 tier 为 TIER_UNKNOWN 且非空时使用。
///                  已知枚举值一律返回规范字符串（忽略 tier_text，
///                  与正向投影「已知值不写 tier_text」对称）。
std::string ToTierString(
  amr_dispatcher::proto::ChassisLinkHealth::Tier tier,
  const std::string & tier_text);

/// proto enum -> rosidl 字符串，规则同 ToTierString。
std::string ToSeverityString(
  amr_dispatcher::proto::SafetyState::Severity severity,
  const std::string & severity_text);

// ---------------------------------------------------------------------------
// Header / Time
// ---------------------------------------------------------------------------
// 只搬数字，不做单位换算：两侧的结构定义逐字相同（见 common.proto）。

void RestoreStamp(const amr_dispatcher::proto::Time & in, builtin_interfaces::msg::Time * out);

void RestoreHeader(const amr_dispatcher::proto::Header & in, std_msgs::msg::Header * out);

// ---------------------------------------------------------------------------
// 7 个消息的反向投影
// ---------------------------------------------------------------------------
// 约定：out 必须是**新建或已清空**的消息。函数内部对 repeated 字段用
// clear_xxx() 再逐条 add，因此复用同一对象也是安全的（这一点比正向投影
// 宽松：正向投影要求调用方自己保证不为脏，因为它只是 add）。
//
// 安全的原因：反向投影的调用点只有双订阅器的回调，每次都是局部新建。

void RestoreChassisLinkHealth(
  const amr_dispatcher::proto::ChassisLinkHealth & in,
  amr_dispatcher_interfaces::msg::ChassisLinkHealth * out);

void RestoreTrackingError(
  const amr_dispatcher::proto::TrackingError & in,
  amr_dispatcher_interfaces::msg::TrackingError * out);

void RestoreVehicleLinkState(
  const amr_dispatcher::proto::VehicleLinkState & in,
  amr_dispatcher_interfaces::msg::VehicleLinkState * out);

void RestoreDispatcherState(
  const amr_dispatcher::proto::DispatcherState & in,
  amr_dispatcher_interfaces::msg::DispatcherState * out);

void RestoreSafetyState(
  const amr_dispatcher::proto::SafetyState & in,
  amr_dispatcher_interfaces::msg::SafetyState * out);

void RestoreTopologyState(
  const amr_dispatcher::proto::TopologyState & in,
  amr_dispatcher_interfaces::msg::TopologyState * out);

void RestoreMissionEvent(
  const amr_dispatcher::proto::MissionEvent & in,
  amr_dispatcher_interfaces::msg::MissionEvent * out);

}  // namespace reverse_projection
}  // namespace amr_dispatcher_proto_bridge

#endif  // AMR_DISPATCHER_PROTO_BRIDGE__PROTO_REVERSE_PROJECTION_HPP_
