// 往返恒等测试：rosidl -> proto -> rosidl 必须逐字段还原。
//
// ============================================================================
// 为什么这个测试必须放在 amr_dispatcher_ros 而不是 proto_bridge
// ============================================================================
// 正向投影（Project*）在 amr_dispatcher_ros，反向投影（Restore*）在
// amr_dispatcher_proto_bridge。往返测试要同时链接两者。若放进 bridge，
// bridge 就必须依赖 ros 包——而 bridge 存在的全部意义正是「tools 能订阅
// proto 而不拖进 ros 的 hardware_interface / pluginlib / BT 依赖」。
// 因此方向相反：本文件在 ros 包里，额外链接 bridge。
//
// ============================================================================
// 这个测试真正要防的是什么
// ============================================================================
// 两个投影层各自都有逐字段单测（test_proto_projection.cpp /
// test_proto_reverse_projection.cpp），但它们**各自独立**通过并不能保证
// 互逆：正向把 -1 翻译成 valid=false，反向也测了 valid=false -> -1，
// 单看两边都对；但若正向实际写的是 valid=false 而反向读的是别的位置，
// 两个单测依旧会绿。
//
// 只有把两个方向首尾相接跑一遍，才能证明「阶段 3 切到 proto 后消费者看到
// 的消息与阶段 2 之前完全相同」——这正是阶段 3 的验收标准。
//
// 填值原则与正向单测一致：全部避开 0 / false / "" / 空数组，
// 否则 proto3 的默认值语义会让「没投影」与「投影了默认值」无法区分。
#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "amr_dispatcher_interfaces/msg/chassis_link_health.hpp"
#include "amr_dispatcher_interfaces/msg/dispatcher_state.hpp"
#include "amr_dispatcher_interfaces/msg/mission_event.hpp"
#include "amr_dispatcher_interfaces/msg/safety_state.hpp"
#include "amr_dispatcher_interfaces/msg/topology_state.hpp"
#include "amr_dispatcher_interfaces/msg/tracking_error.hpp"
#include "amr_dispatcher_interfaces/msg/vehicle_link_state.hpp"

#include "amr_dispatcher_proto_bridge/proto_dual_subscriber.hpp"
#include "amr_dispatcher_proto_bridge/proto_reverse_projection.hpp"
// ProtoTopicName 在 proto_dual_publisher.hpp（与投影函数分开），
// 话题名 parity 用例要用到它。
#include "amr_dispatcher_ros/proto_dual_publisher.hpp"
#include "amr_dispatcher_ros/proto_projection.hpp"

namespace
{

using amr_dispatcher_interfaces::msg::ChassisLinkHealth;
using amr_dispatcher_interfaces::msg::DispatcherState;
using amr_dispatcher_interfaces::msg::MissionEvent;
using amr_dispatcher_interfaces::msg::SafetyState;
using amr_dispatcher_interfaces::msg::TopologyState;
using amr_dispatcher_interfaces::msg::TrackingError;
using amr_dispatcher_interfaces::msg::VehicleLinkState;

builtin_interfaces::msg::Time MakeStamp()
{
  builtin_interfaces::msg::Time t;
  t.sec = 1730000000;
  t.nanosec = 987654321u;
  return t;
}

std_msgs::msg::Header MakeHeader()
{
  std_msgs::msg::Header h;
  h.stamp = MakeStamp();
  h.frame_id = "map";
  return h;
}

void ExpectStampEq(
  const builtin_interfaces::msg::Time & a, const builtin_interfaces::msg::Time & b)
{
  EXPECT_EQ(a.sec, b.sec);
  EXPECT_EQ(a.nanosec, b.nanosec);
}

void ExpectHeaderEq(const std_msgs::msg::Header & a, const std_msgs::msg::Header & b)
{
  ExpectStampEq(a.stamp, b.stamp);
  EXPECT_EQ(a.frame_id, b.frame_id);
}

// 完整的单条车辆链路（供 DispatcherState.vehicle_links 使用）。
VehicleLinkState MakeVehicleLink(const std::string & id)
{
  VehicleLinkState v;
  v.vehicle_id = id;
  v.backend = "vda5050";
  v.online = true;
  v.busy = true;
  v.quarantined = true;
  v.position_valid = true;
  v.current_station_id = "ST_7";
  v.battery_percent = 42.5;
  v.state_age_ms = 1234;
  v.last_error = "none";
  return v;
}

// ---------------------------------------------------------------------------
// 逐消息往返
// ---------------------------------------------------------------------------

TEST(RoundTrip, ChassisLinkHealth) {
  ChassisLinkHealth in;
  in.header = MakeHeader();
  in.backend_name = "serial";
  in.tier = "DEGRADED";
  in.frames_total = 1000u;
  in.frames_success = 900u;
  in.frames_lost = 100u;
  in.frames_crc_error = 5u;
  in.loss_rate = 0.1;
  in.avg_latency_ms = 12.5;
  in.max_latency_ms = 80.0;
  in.is_healthy = false;
  // 刻意用 -1 哨兵：这是三处有损点里最容易在往返中被抹平的一个。
  in.command_delivery_rate = -1.0;
  in.commands_sent = 500u;
  in.commands_delivered = 485u;

  amr_dispatcher::proto::ChassisLinkHealth p;
  amr_dispatcher_ros::proto_projection::ProjectChassisLinkHealth(in, &p);

  ChassisLinkHealth out;
  amr_dispatcher_proto_bridge::reverse_projection::RestoreChassisLinkHealth(p, &out);

  ExpectHeaderEq(in.header, out.header);
  EXPECT_EQ(in.backend_name, out.backend_name);
  EXPECT_EQ(in.tier, out.tier);
  EXPECT_EQ(in.frames_total, out.frames_total);
  EXPECT_EQ(in.frames_success, out.frames_success);
  EXPECT_EQ(in.frames_lost, out.frames_lost);
  EXPECT_EQ(in.frames_crc_error, out.frames_crc_error);
  EXPECT_DOUBLE_EQ(in.loss_rate, out.loss_rate);
  EXPECT_DOUBLE_EQ(in.avg_latency_ms, out.avg_latency_ms);
  EXPECT_DOUBLE_EQ(in.max_latency_ms, out.max_latency_ms);
  EXPECT_EQ(in.is_healthy, out.is_healthy);
  // -1 -> valid=false -> -1：往返必须回到 -1，不能是 0.0。
  EXPECT_DOUBLE_EQ(out.command_delivery_rate, -1.0);
  EXPECT_EQ(in.commands_sent, out.commands_sent);
  EXPECT_EQ(in.commands_delivered, out.commands_delivered);
}

TEST(RoundTrip, ChassisLinkHealthDeliveryRateApplicable) {
  // 适用值同样要原样回来（valid=true 分支）。
  ChassisLinkHealth in;
  in.backend_name = "can";
  in.tier = "HEALTHY";
  in.command_delivery_rate = 0.97;

  amr_dispatcher::proto::ChassisLinkHealth p;
  amr_dispatcher_ros::proto_projection::ProjectChassisLinkHealth(in, &p);
  ChassisLinkHealth out;
  amr_dispatcher_proto_bridge::reverse_projection::RestoreChassisLinkHealth(p, &out);

  EXPECT_DOUBLE_EQ(out.command_delivery_rate, 0.97);
  EXPECT_EQ(out.tier, "HEALTHY");
}

TEST(RoundTrip, ChassisLinkHealthUnknownTierKeepsText) {
  // 枚举外值走 *_text 兜底。这条同时验证正向「已知值不写 text」与反向
  // 「UNKNOWN 才读 text」两条规则是对称的。
  ChassisLinkHealth in;
  in.backend_name = "serial";
  in.tier = "SATURATED";

  amr_dispatcher::proto::ChassisLinkHealth p;
  amr_dispatcher_ros::proto_projection::ProjectChassisLinkHealth(in, &p);
  ASSERT_EQ(p.tier(), amr_dispatcher::proto::ChassisLinkHealth::TIER_UNKNOWN);
  ASSERT_EQ(p.tier_text(), "SATURATED");

  ChassisLinkHealth out;
  amr_dispatcher_proto_bridge::reverse_projection::RestoreChassisLinkHealth(p, &out);
  EXPECT_EQ(out.tier, "SATURATED");
}

TEST(RoundTrip, TrackingError) {
  TrackingError in;
  in.header = MakeHeader();
  in.controller_name = "stanley";
  in.lateral_error_m = 0.03;
  in.heading_error_rad = -0.02;
  in.lookahead_distance_m = 0.6;
  in.target_linear_x_mps = 0.5;
  in.target_angular_z_radps = 0.1;
  in.actual_linear_x_mps = 0.49;
  in.actual_angular_z_radps = 0.09;
  in.nearest_waypoint_index = 17;
  in.arrived = true;

  amr_dispatcher::proto::TrackingError p;
  amr_dispatcher_ros::proto_projection::ProjectTrackingError(in, &p);
  TrackingError out;
  amr_dispatcher_proto_bridge::reverse_projection::RestoreTrackingError(p, &out);

  ExpectHeaderEq(in.header, out.header);
  EXPECT_EQ(in.controller_name, out.controller_name);
  EXPECT_DOUBLE_EQ(in.lateral_error_m, out.lateral_error_m);
  EXPECT_DOUBLE_EQ(in.heading_error_rad, out.heading_error_rad);
  EXPECT_DOUBLE_EQ(in.lookahead_distance_m, out.lookahead_distance_m);
  EXPECT_DOUBLE_EQ(in.target_linear_x_mps, out.target_linear_x_mps);
  EXPECT_DOUBLE_EQ(in.target_angular_z_radps, out.target_angular_z_radps);
  EXPECT_DOUBLE_EQ(in.actual_linear_x_mps, out.actual_linear_x_mps);
  EXPECT_DOUBLE_EQ(in.actual_angular_z_radps, out.actual_angular_z_radps);
  EXPECT_EQ(in.nearest_waypoint_index, out.nearest_waypoint_index);
  EXPECT_EQ(in.arrived, out.arrived);
}

TEST(RoundTrip, VehicleLinkState) {
  const VehicleLinkState in = MakeVehicleLink("amr_01");

  amr_dispatcher::proto::VehicleLinkState p;
  amr_dispatcher_ros::proto_projection::ProjectVehicleLinkState(in, &p);
  VehicleLinkState out;
  amr_dispatcher_proto_bridge::reverse_projection::RestoreVehicleLinkState(p, &out);

  EXPECT_EQ(in.vehicle_id, out.vehicle_id);
  EXPECT_EQ(in.backend, out.backend);
  EXPECT_EQ(in.online, out.online);
  EXPECT_EQ(in.busy, out.busy);
  EXPECT_EQ(in.quarantined, out.quarantined);
  EXPECT_EQ(in.position_valid, out.position_valid);
  EXPECT_EQ(in.current_station_id, out.current_station_id);
  EXPECT_DOUBLE_EQ(in.battery_percent, out.battery_percent);
  EXPECT_EQ(in.state_age_ms, out.state_age_ms);
  EXPECT_EQ(in.last_error, out.last_error);
}

TEST(RoundTrip, DispatcherState) {
  DispatcherState in;
  in.header = MakeHeader();
  in.current_comparator = "nearest";
  in.queue_size = 3u;
  in.queue_capacity = 10u;
  in.active_missions_count = 2u;
  in.paused_missions_count = 1u;
  in.deadlock_detected = true;
  in.active_mission_id = "m-42";
  in.active_mission_ids = {"m-42", "m-43", "m-44"};
  in.vehicle_links = {MakeVehicleLink("amr_01"), MakeVehicleLink("amr_02")};
  in.dispatch_total = 100u;
  in.dispatch_rejected_total = 5u;
  in.dispatch_deferred_total = 8u;
  in.lock_contention_total = 11u;
  in.event_total = 250u;
  in.overlay_active = true;
  in.overlay_revision = 9u;
  in.blocked_edges = 2u;
  in.one_way_edges = 1u;

  amr_dispatcher::proto::DispatcherState p;
  amr_dispatcher_ros::proto_projection::ProjectDispatcherState(in, &p);
  DispatcherState out;
  amr_dispatcher_proto_bridge::reverse_projection::RestoreDispatcherState(p, &out);

  ExpectHeaderEq(in.header, out.header);
  EXPECT_EQ(in.current_comparator, out.current_comparator);
  EXPECT_EQ(in.queue_size, out.queue_size);
  EXPECT_EQ(in.queue_capacity, out.queue_capacity);
  EXPECT_EQ(in.active_missions_count, out.active_missions_count);
  EXPECT_EQ(in.paused_missions_count, out.paused_missions_count);
  EXPECT_EQ(in.deadlock_detected, out.deadlock_detected);
  EXPECT_EQ(in.active_mission_id, out.active_mission_id);
  ASSERT_EQ(out.active_mission_ids.size(), in.active_mission_ids.size());
  for (std::size_t i = 0; i < in.active_mission_ids.size(); ++i) {
    EXPECT_EQ(in.active_mission_ids[i], out.active_mission_ids[i]);
  }
  // 嵌套数组的条数与顺序必须严格一致：rest_gateway 直接把它序列化成 JSON 数组。
  ASSERT_EQ(out.vehicle_links.size(), in.vehicle_links.size());
  for (std::size_t i = 0; i < in.vehicle_links.size(); ++i) {
    EXPECT_EQ(in.vehicle_links[i].vehicle_id, out.vehicle_links[i].vehicle_id);
    EXPECT_EQ(in.vehicle_links[i].online, out.vehicle_links[i].online);
    EXPECT_EQ(in.vehicle_links[i].busy, out.vehicle_links[i].busy);
    EXPECT_EQ(in.vehicle_links[i].quarantined, out.vehicle_links[i].quarantined);
    EXPECT_EQ(in.vehicle_links[i].backend, out.vehicle_links[i].backend);
  }
  EXPECT_EQ(in.dispatch_total, out.dispatch_total);
  EXPECT_EQ(in.dispatch_rejected_total, out.dispatch_rejected_total);
  EXPECT_EQ(in.dispatch_deferred_total, out.dispatch_deferred_total);
  EXPECT_EQ(in.lock_contention_total, out.lock_contention_total);
  EXPECT_EQ(in.event_total, out.event_total);
  EXPECT_EQ(in.overlay_active, out.overlay_active);
  EXPECT_EQ(in.overlay_revision, out.overlay_revision);
  EXPECT_EQ(in.blocked_edges, out.blocked_edges);
  EXPECT_EQ(in.one_way_edges, out.one_way_edges);
}

TEST(RoundTrip, SafetyState) {
  SafetyState in;
  in.header = MakeHeader();
  in.severity = "CRITICAL";
  in.motion_allowed = false;
  in.speed_scale = 0.3;
  in.active_stop_source = "bumper";
  in.reason = "obstacle";
  in.active_sources = {"bumper", "estop"};

  amr_dispatcher::proto::SafetyState p;
  amr_dispatcher_ros::proto_projection::ProjectSafetyState(in, &p);
  SafetyState out;
  amr_dispatcher_proto_bridge::reverse_projection::RestoreSafetyState(p, &out);

  ExpectHeaderEq(in.header, out.header);
  // safety_gate 与 rest_gateway 都用 severity == "CRITICAL" 做判定，
  // 字面量必须逐字相同。
  EXPECT_EQ(out.severity, "CRITICAL");
  EXPECT_EQ(in.motion_allowed, out.motion_allowed);
  EXPECT_DOUBLE_EQ(in.speed_scale, out.speed_scale);
  EXPECT_EQ(in.active_stop_source, out.active_stop_source);
  EXPECT_EQ(in.reason, out.reason);
  ASSERT_EQ(out.active_sources.size(), in.active_sources.size());
  for (std::size_t i = 0; i < in.active_sources.size(); ++i) {
    EXPECT_EQ(in.active_sources[i], out.active_sources[i]);
  }
}

TEST(RoundTrip, SafetyStateUnknownSeverityKeepsText) {
  SafetyState in;
  in.severity = "PANIC";
  in.motion_allowed = false;

  amr_dispatcher::proto::SafetyState p;
  amr_dispatcher_ros::proto_projection::ProjectSafetyState(in, &p);
  ASSERT_EQ(p.severity(), amr_dispatcher::proto::SafetyState::SEVERITY_UNKNOWN);
  ASSERT_EQ(p.severity_text(), "PANIC");

  SafetyState out;
  amr_dispatcher_proto_bridge::reverse_projection::RestoreSafetyState(p, &out);
  EXPECT_EQ(out.severity, "PANIC");
}

TEST(RoundTrip, TopologyState) {
  TopologyState in;
  in.stamp = MakeStamp();
  in.active_edges = {"A-B", "B-C"};
  in.locked_edges = {"C-D"};
  in.edge_owners = {"amr_01"};
  in.deadlocked_nodes = {"B"};
  in.deadlock_detected = true;

  amr_dispatcher::proto::TopologyState p;
  amr_dispatcher_ros::proto_projection::ProjectTopologyState(in, &p);
  TopologyState out;
  amr_dispatcher_proto_bridge::reverse_projection::RestoreTopologyState(p, &out);

  ExpectStampEq(in.stamp, out.stamp);
  ASSERT_EQ(out.active_edges.size(), in.active_edges.size());
  for (std::size_t i = 0; i < in.active_edges.size(); ++i) {
    EXPECT_EQ(in.active_edges[i], out.active_edges[i]);
  }
  ASSERT_EQ(out.locked_edges.size(), in.locked_edges.size());
  for (std::size_t i = 0; i < in.locked_edges.size(); ++i) {
    EXPECT_EQ(in.locked_edges[i], out.locked_edges[i]);
  }
  ASSERT_EQ(out.edge_owners.size(), in.edge_owners.size());
  for (std::size_t i = 0; i < in.edge_owners.size(); ++i) {
    EXPECT_EQ(in.edge_owners[i], out.edge_owners[i]);
  }
  ASSERT_EQ(out.deadlocked_nodes.size(), in.deadlocked_nodes.size());
  for (std::size_t i = 0; i < in.deadlocked_nodes.size(); ++i) {
    EXPECT_EQ(in.deadlocked_nodes[i], out.deadlocked_nodes[i]);
  }
  EXPECT_EQ(in.deadlock_detected, out.deadlock_detected);
}

TEST(RoundTrip, MissionEvent) {
  MissionEvent in;
  in.stamp = MakeStamp();
  in.mission_id = "m-1";
  in.order_id = "o-9";
  in.event = "finished";
  in.from_state = "ACTIVE";
  in.to_state = "FINISHED";
  in.reason = "goal_reached";

  amr_dispatcher::proto::MissionEvent p;
  amr_dispatcher_ros::proto_projection::ProjectMissionEvent(in, &p);
  MissionEvent out;
  amr_dispatcher_proto_bridge::reverse_projection::RestoreMissionEvent(p, &out);

  ExpectStampEq(in.stamp, out.stamp);
  EXPECT_EQ(in.mission_id, out.mission_id);
  EXPECT_EQ(in.order_id, out.order_id);
  EXPECT_EQ(in.event, out.event);
  EXPECT_EQ(in.from_state, out.from_state);
  EXPECT_EQ(in.to_state, out.to_state);
  EXPECT_EQ(in.reason, out.reason);
}

// ---------------------------------------------------------------------------
// 话题名派生：发布端与订阅端必须派生出同一个名字
// ---------------------------------------------------------------------------
// 两侧各有一份 kProtoTopicSuffix（bridge 不能依赖 ros 包，见两处头文件
// 注释）。漂移的后果是「订阅成功但永远收不到」——DDS 不报错，只是没有
// 匹配的发布者，属于最难现场定位的一类故障。这个用例是唯一的守卫。
TEST(RoundTrip, ProtoTopicNameParityAcrossPackages) {
  const std::vector<std::string> topics = {
    "/chassis/link_health",
    "/system/aggregated_link_health",
    "/dispatcher/state",
    "/dispatcher/events",
    "/dispatcher/topology_state",
    "/safety/state",
    "/tracking/error",
    "/tracking/error/amr_01",
  };
  for (const auto & t : topics) {
    EXPECT_EQ(
      amr_dispatcher_ros::ProtoTopicName(t),
      amr_dispatcher_proto_bridge::ProtoTopicName(t))
      << "topic name drift for " << t;
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
