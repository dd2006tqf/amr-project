#include "amr_dispatcher_ros/proto_projection.hpp"

#include <utility>

namespace amr_dispatcher_ros
{
namespace proto_projection
{

amr_dispatcher::proto::ChassisLinkHealth::Tier ToTierEnum(const std::string & tier)
{
  using Tier = amr_dispatcher::proto::ChassisLinkHealth::Tier;
  if (tier == "HEALTHY") {
    return amr_dispatcher::proto::ChassisLinkHealth::TIER_HEALTHY;
  }
  if (tier == "DEGRADED") {
    return amr_dispatcher::proto::ChassisLinkHealth::TIER_DEGRADED;
  }
  if (tier == "CRITICAL") {
    return amr_dispatcher::proto::ChassisLinkHealth::TIER_CRITICAL;
  }
  // 包含 "UNKNOWN" 与任何拼写错误 —— 都归到 TIER_UNKNOWN，
  // 原文由调用方写进 tier_text 保留证据。
  (void)sizeof(Tier);
  return amr_dispatcher::proto::ChassisLinkHealth::TIER_UNKNOWN;
}

std::string ToTierString(amr_dispatcher::proto::ChassisLinkHealth::Tier tier)
{
  switch (tier) {
    case amr_dispatcher::proto::ChassisLinkHealth::TIER_HEALTHY:
      return "HEALTHY";
    case amr_dispatcher::proto::ChassisLinkHealth::TIER_DEGRADED:
      return "DEGRADED";
    case amr_dispatcher::proto::ChassisLinkHealth::TIER_CRITICAL:
      return "CRITICAL";
    case amr_dispatcher::proto::ChassisLinkHealth::TIER_UNKNOWN:
    default:
      return "UNKNOWN";
  }
}

amr_dispatcher::proto::SafetyState::Severity ToSeverityEnum(const std::string & severity)
{
  if (severity == "INFO") {
    return amr_dispatcher::proto::SafetyState::SEVERITY_INFO;
  }
  if (severity == "WARN") {
    return amr_dispatcher::proto::SafetyState::SEVERITY_WARN;
  }
  if (severity == "CRITICAL") {
    return amr_dispatcher::proto::SafetyState::SEVERITY_CRITICAL;
  }
  return amr_dispatcher::proto::SafetyState::SEVERITY_UNKNOWN;
}

std::string ToSeverityString(amr_dispatcher::proto::SafetyState::Severity severity)
{
  switch (severity) {
    case amr_dispatcher::proto::SafetyState::SEVERITY_INFO:
      return "INFO";
    case amr_dispatcher::proto::SafetyState::SEVERITY_WARN:
      return "WARN";
    case amr_dispatcher::proto::SafetyState::SEVERITY_CRITICAL:
      return "CRITICAL";
    case amr_dispatcher::proto::SafetyState::SEVERITY_UNKNOWN:
    default:
      return "UNKNOWN";
  }
}

void ProjectStamp(const builtin_interfaces::msg::Time & in, amr_dispatcher::proto::Time * out)
{
  out->set_sec(in.sec);
  out->set_nanosec(in.nanosec);
}

void ProjectHeader(const std_msgs::msg::Header & in, amr_dispatcher::proto::Header * out)
{
  ProjectStamp(in.stamp, out->mutable_stamp());
  out->set_frame_id(in.frame_id);
}

void ProjectChassisLinkHealth(
  const amr_dispatcher_interfaces::msg::ChassisLinkHealth & in,
  amr_dispatcher::proto::ChassisLinkHealth * out)
{
  ProjectHeader(in.header, out->mutable_header());

  out->set_backend_name(in.backend_name);

  const auto tier = ToTierEnum(in.tier);
  out->set_tier(tier);
  // 只有无法映射到枚举时才有必要保留原文：已知值写回字符串是纯冗余，
  // 且会让每帧多占 8-12 字节（proto3 非空 string 都要占 tag + len + 内容）。
  // 这直接影响本方案「计数器消息更省字节」的论断，所以这里刻意留空。
  if (tier == amr_dispatcher::proto::ChassisLinkHealth::TIER_UNKNOWN && in.tier != "UNKNOWN") {
    out->set_tier_text(in.tier);
  }

  out->set_frames_total(in.frames_total);
  out->set_frames_success(in.frames_success);
  out->set_frames_lost(in.frames_lost);
  out->set_frames_crc_error(in.frames_crc_error);
  out->set_loss_rate(in.loss_rate);
  out->set_avg_latency_ms(in.avg_latency_ms);
  out->set_max_latency_ms(in.max_latency_ms);
  out->set_is_healthy(in.is_healthy);

  // -1 哨兵 -> valid=false。见头文件注释：这是必须翻译、不能逐字复制的字段。
  //
  // 判定用 `< 0.0` 而不是 `== -1.0`：哨兵的生产者是 chassis_driver_node 与
  // link_health_publisher，都把「不适用」写成 -1.0；但任何负值在语义上都只能
  // 是「无意义」，用 < 0 能顺带接住上游将来把哨兵改成 -1.0f 之外的负数。
  if (in.command_delivery_rate < 0.0) {
    out->set_command_delivery_rate_valid(false);
    out->set_command_delivery_rate(0.0);
  } else {
    out->set_command_delivery_rate_valid(true);
    out->set_command_delivery_rate(in.command_delivery_rate);
  }
  out->set_commands_sent(in.commands_sent);
  out->set_commands_delivered(in.commands_delivered);
}

void ProjectTrackingError(
  const amr_dispatcher_interfaces::msg::TrackingError & in,
  amr_dispatcher::proto::TrackingError * out)
{
  ProjectHeader(in.header, out->mutable_header());

  out->set_controller_name(in.controller_name);
  out->set_lateral_error_m(in.lateral_error_m);
  out->set_heading_error_rad(in.heading_error_rad);
  out->set_lookahead_distance_m(in.lookahead_distance_m);
  out->set_target_linear_x_mps(in.target_linear_x_mps);
  out->set_target_angular_z_radps(in.target_angular_z_radps);
  out->set_actual_linear_x_mps(in.actual_linear_x_mps);
  out->set_actual_angular_z_radps(in.actual_angular_z_radps);
  out->set_nearest_waypoint_index(in.nearest_waypoint_index);
  out->set_arrived(in.arrived);
}

void ProjectVehicleLinkState(
  const amr_dispatcher_interfaces::msg::VehicleLinkState & in,
  amr_dispatcher::proto::VehicleLinkState * out)
{
  out->set_vehicle_id(in.vehicle_id);
  out->set_backend(in.backend);
  out->set_online(in.online);
  out->set_busy(in.busy);
  out->set_quarantined(in.quarantined);
  out->set_position_valid(in.position_valid);
  out->set_current_station_id(in.current_station_id);
  out->set_battery_percent(in.battery_percent);
  out->set_state_age_ms(in.state_age_ms);
  out->set_last_error(in.last_error);
}

void ProjectDispatcherState(
  const amr_dispatcher_interfaces::msg::DispatcherState & in,
  amr_dispatcher::proto::DispatcherState * out)
{
  ProjectHeader(in.header, out->mutable_header());

  out->set_current_comparator(in.current_comparator);
  out->set_queue_size(in.queue_size);
  out->set_queue_capacity(in.queue_capacity);
  out->set_active_missions_count(in.active_missions_count);
  out->set_paused_missions_count(in.paused_missions_count);
  out->set_deadlock_detected(in.deadlock_detected);
  out->set_active_mission_id(in.active_mission_id);

  // repeated 字段：out 必须为空（见头文件约定），用 Add 而非赋值以避免重复。
  for (const auto & id : in.active_mission_ids) {
    out->add_active_mission_ids(id);
  }

  // 嵌套消息数组：这是本方案相对 rosidl 的主要结构优势（原生 repeated message，
  // 消费者可增量解析，不必先知道条数）。逐条投影，不重排、不填充默认值——
  // 条数与顺序必须与上游严格一致，否则一致性检查会失败（这是正确行为）。
  for (const auto & link : in.vehicle_links) {
    ProjectVehicleLinkState(link, out->add_vehicle_links());
  }

  out->set_dispatch_total(in.dispatch_total);
  out->set_dispatch_rejected_total(in.dispatch_rejected_total);
  out->set_dispatch_deferred_total(in.dispatch_deferred_total);
  out->set_lock_contention_total(in.lock_contention_total);
  out->set_event_total(in.event_total);

  out->set_overlay_active(in.overlay_active);
  out->set_overlay_revision(in.overlay_revision);
  out->set_blocked_edges(in.blocked_edges);
  out->set_one_way_edges(in.one_way_edges);
}

void ProjectSafetyState(
  const amr_dispatcher_interfaces::msg::SafetyState & in,
  amr_dispatcher::proto::SafetyState * out)
{
  ProjectHeader(in.header, out->mutable_header());

  const auto severity = ToSeverityEnum(in.severity);
  out->set_severity(severity);
  // 与 tier 同理：只在无法映射时保留原文。
  // 安全消息是最高频的（20Hz），冗余字符串的代价在这里最明显。
  if (severity == amr_dispatcher::proto::SafetyState::SEVERITY_UNKNOWN &&
    in.severity != "UNKNOWN")
  {
    out->set_severity_text(in.severity);
  }

  out->set_motion_allowed(in.motion_allowed);
  out->set_speed_scale(in.speed_scale);
  out->set_active_stop_source(in.active_stop_source);
  out->set_reason(in.reason);

  for (const auto & source : in.active_sources) {
    out->add_active_sources(source);
  }
}

void ProjectTopologyState(
  const amr_dispatcher_interfaces::msg::TopologyState & in,
  amr_dispatcher::proto::TopologyState * out)
{
  ProjectStamp(in.stamp, out->mutable_stamp());

  for (const auto & edge : in.active_edges) {
    out->add_active_edges(edge);
  }
  for (const auto & edge : in.locked_edges) {
    out->add_locked_edges(edge);
  }
  for (const auto & owner : in.edge_owners) {
    out->add_edge_owners(owner);
  }
  for (const auto & node : in.deadlocked_nodes) {
    out->add_deadlocked_nodes(node);
  }

  out->set_deadlock_detected(in.deadlock_detected);
}

void ProjectMissionEvent(
  const amr_dispatcher_interfaces::msg::MissionEvent & in,
  amr_dispatcher::proto::MissionEvent * out)
{
  ProjectStamp(in.stamp, out->mutable_stamp());

  out->set_mission_id(in.mission_id);
  out->set_order_id(in.order_id);
  out->set_event(in.event);
  out->set_from_state(in.from_state);
  out->set_to_state(in.to_state);
  out->set_reason(in.reason);
}

}  // namespace proto_projection
}  // namespace amr_dispatcher_ros
