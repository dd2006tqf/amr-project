#include "amr_dispatcher_proto_bridge/proto_reverse_projection.hpp"

namespace amr_dispatcher_proto_bridge
{
namespace reverse_projection
{

std::string ToTierString(
  amr_dispatcher::proto::ChassisLinkHealth::Tier tier, const std::string & tier_text)
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
      // 未知枚举值才有必要回退到原文：方向与正向投影的
      // 「TIER_UNKNOWN && in.tier != "UNKNOWN" 时写 tier_text」严格对称。
      //
      // proto3 会把**未识别的 enum 数值**原样保留在字段里（而不是归零），
      // 于是未来上游新增一档而本端 proto 定义未更新时，这里拿到的是那个
      // 未知数值 -> 落进 default -> 仍能通过 tier_text 还原原文。
      return tier_text.empty() ? "UNKNOWN" : tier_text;
  }
}

std::string ToSeverityString(
  amr_dispatcher::proto::SafetyState::Severity severity, const std::string & severity_text)
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
      return severity_text.empty() ? "UNKNOWN" : severity_text;
  }
}

void RestoreStamp(const amr_dispatcher::proto::Time & in, builtin_interfaces::msg::Time * out)
{
  out->sec = in.sec();
  out->nanosec = in.nanosec();
}

void RestoreHeader(const amr_dispatcher::proto::Header & in, std_msgs::msg::Header * out)
{
  RestoreStamp(in.stamp(), &out->stamp);
  out->frame_id = in.frame_id();
}

void RestoreChassisLinkHealth(
  const amr_dispatcher::proto::ChassisLinkHealth & in,
  amr_dispatcher_interfaces::msg::ChassisLinkHealth * out)
{
  RestoreHeader(in.header(), &out->header);

  out->backend_name = in.backend_name();
  out->tier = ToTierString(in.tier(), in.tier_text());

  out->frames_total = in.frames_total();
  out->frames_success = in.frames_success();
  out->frames_lost = in.frames_lost();
  out->frames_crc_error = in.frames_crc_error();
  out->loss_rate = in.loss_rate();
  out->avg_latency_ms = in.avg_latency_ms();
  out->max_latency_ms = in.max_latency_ms();
  out->is_healthy = in.is_healthy();

  // 有损点 1：valid=false 还原成 -1 哨兵，不能还原成 0.0。
  // 0.0 的语义是「命令送达率 0%」，与「不适用」是相反的运维结论。
  out->command_delivery_rate =
    in.command_delivery_rate_valid() ? in.command_delivery_rate() : -1.0;
  out->commands_sent = in.commands_sent();
  out->commands_delivered = in.commands_delivered();
}

void RestoreTrackingError(
  const amr_dispatcher::proto::TrackingError & in,
  amr_dispatcher_interfaces::msg::TrackingError * out)
{
  RestoreHeader(in.header(), &out->header);

  out->controller_name = in.controller_name();
  out->lateral_error_m = in.lateral_error_m();
  out->heading_error_rad = in.heading_error_rad();
  out->lookahead_distance_m = in.lookahead_distance_m();
  out->target_linear_x_mps = in.target_linear_x_mps();
  out->target_angular_z_radps = in.target_angular_z_radps();
  out->actual_linear_x_mps = in.actual_linear_x_mps();
  out->actual_angular_z_radps = in.actual_angular_z_radps();
  out->nearest_waypoint_index = in.nearest_waypoint_index();
  out->arrived = in.arrived();
}

void RestoreVehicleLinkState(
  const amr_dispatcher::proto::VehicleLinkState & in,
  amr_dispatcher_interfaces::msg::VehicleLinkState * out)
{
  out->vehicle_id = in.vehicle_id();
  out->backend = in.backend();
  out->online = in.online();
  out->busy = in.busy();
  out->quarantined = in.quarantined();
  out->position_valid = in.position_valid();
  out->current_station_id = in.current_station_id();
  out->battery_percent = in.battery_percent();
  out->state_age_ms = in.state_age_ms();
  out->last_error = in.last_error();
}

void RestoreDispatcherState(
  const amr_dispatcher::proto::DispatcherState & in,
  amr_dispatcher_interfaces::msg::DispatcherState * out)
{
  RestoreHeader(in.header(), &out->header);

  out->current_comparator = in.current_comparator();
  out->queue_size = in.queue_size();
  out->queue_capacity = in.queue_capacity();
  out->active_missions_count = in.active_missions_count();
  out->paused_missions_count = in.paused_missions_count();
  out->deadlock_detected = in.deadlock_detected();
  out->active_mission_id = in.active_mission_id();

  // repeated 字段：先清空再逐条追加。复用 out 时不清会累积，
  // 而本函数承诺「复用安全」（见头文件约定）。
  out->active_mission_ids.clear();
  out->active_mission_ids.reserve(static_cast<std::size_t>(in.active_mission_ids_size()));
  for (const auto & id : in.active_mission_ids()) {
    out->active_mission_ids.push_back(id);
  }

  // 嵌套消息数组：条数与顺序必须与上游严格一致（正向投影也是逐条 add，
  // 不做重排/填充），否则一致性检查会失败——这是正确行为。
  out->vehicle_links.clear();
  out->vehicle_links.reserve(static_cast<std::size_t>(in.vehicle_links_size()));
  for (const auto & link : in.vehicle_links()) {
    amr_dispatcher_interfaces::msg::VehicleLinkState restored;
    RestoreVehicleLinkState(link, &restored);
    out->vehicle_links.push_back(std::move(restored));
  }

  out->dispatch_total = in.dispatch_total();
  out->dispatch_rejected_total = in.dispatch_rejected_total();
  out->dispatch_deferred_total = in.dispatch_deferred_total();
  out->lock_contention_total = in.lock_contention_total();
  out->event_total = in.event_total();

  out->overlay_active = in.overlay_active();
  out->overlay_revision = in.overlay_revision();
  out->blocked_edges = in.blocked_edges();
  out->one_way_edges = in.one_way_edges();
}

void RestoreSafetyState(
  const amr_dispatcher::proto::SafetyState & in,
  amr_dispatcher_interfaces::msg::SafetyState * out)
{
  RestoreHeader(in.header(), &out->header);

  out->severity = ToSeverityString(in.severity(), in.severity_text());
  out->motion_allowed = in.motion_allowed();
  out->speed_scale = in.speed_scale();
  out->active_stop_source = in.active_stop_source();
  out->reason = in.reason();

  out->active_sources.clear();
  out->active_sources.reserve(static_cast<std::size_t>(in.active_sources_size()));
  for (const auto & source : in.active_sources()) {
    out->active_sources.push_back(source);
  }
}

void RestoreTopologyState(
  const amr_dispatcher::proto::TopologyState & in,
  amr_dispatcher_interfaces::msg::TopologyState * out)
{
  RestoreStamp(in.stamp(), &out->stamp);

  out->active_edges.clear();
  out->active_edges.reserve(static_cast<std::size_t>(in.active_edges_size()));
  for (const auto & edge : in.active_edges()) {
    out->active_edges.push_back(edge);
  }

  out->locked_edges.clear();
  out->locked_edges.reserve(static_cast<std::size_t>(in.locked_edges_size()));
  for (const auto & edge : in.locked_edges()) {
    out->locked_edges.push_back(edge);
  }

  out->edge_owners.clear();
  out->edge_owners.reserve(static_cast<std::size_t>(in.edge_owners_size()));
  for (const auto & owner : in.edge_owners()) {
    out->edge_owners.push_back(owner);
  }

  out->deadlocked_nodes.clear();
  out->deadlocked_nodes.reserve(static_cast<std::size_t>(in.deadlocked_nodes_size()));
  for (const auto & node : in.deadlocked_nodes()) {
    out->deadlocked_nodes.push_back(node);
  }

  out->deadlock_detected = in.deadlock_detected();
}

void RestoreMissionEvent(
  const amr_dispatcher::proto::MissionEvent & in,
  amr_dispatcher_interfaces::msg::MissionEvent * out)
{
  RestoreStamp(in.stamp(), &out->stamp);

  out->mission_id = in.mission_id();
  out->order_id = in.order_id();
  out->event = in.event();
  out->from_state = in.from_state();
  out->to_state = in.to_state();
  out->reason = in.reason();
}

}  // namespace reverse_projection
}  // namespace amr_dispatcher_proto_bridge
