// 反向投影（proto -> rosidl）的逐字段单测。
//
// ============================================================================
// 为什么必须逐字段覆盖，而不是只测「能编译、能跑」
// ============================================================================
// 阶段 2 的正向投影已经证明：字段漏写不会报错。proto3 的 setter 存在即可
// 编译通过，运行时缺字段只会得到一个默认值——而默认值往往在语义上「看起来
// 正常」（0、false、空串）。等到阶段 4 删掉 rosidl 侧，就再没有对照物了。
//
// 因此本文件对 7 个消息逐个字段断言，并把三处**有损点**单独拎出来测。
//
// ============================================================================
// 与 test_proto_projection.cpp 的关系
// ============================================================================
// 那个文件测 rosidl -> proto（正向），本文件测 proto -> rosidl（反向）。
// 完整的往返（rosidl -> proto -> rosidl 恒等）需要两者同时链接，因此放在
// amr_dispatcher_ros/test/test_proto_round_trip.cpp——本包不能依赖
// amr_dispatcher_ros（那会让 tools 也拖进 ros 包的全部重量级依赖）。
#include <gtest/gtest.h>

#include <limits>
#include <string>

#include "amr_dispatcher_proto_bridge/proto_reverse_projection.hpp"
// ProtoTopicName / kProtoTopicSuffix 定义在双订阅器头里（与反向投影分开：
// 前者依赖 rclcpp 的订阅模板，后者是纯函数，不该被强制一起包含）。
#include "amr_dispatcher_proto_bridge/proto_dual_subscriber.hpp"

namespace rp = amr_dispatcher_proto_bridge::reverse_projection;

using amr_dispatcher_interfaces::msg::ChassisLinkHealth;
using amr_dispatcher_interfaces::msg::DispatcherState;
using amr_dispatcher_interfaces::msg::MissionEvent;
using amr_dispatcher_interfaces::msg::SafetyState;
using amr_dispatcher_interfaces::msg::TopologyState;
using amr_dispatcher_interfaces::msg::TrackingError;
using amr_dispatcher_interfaces::msg::VehicleLinkState;

// ---------------------------------------------------------------------------
// 枚举翻译：三处有损点之二、之三
// ---------------------------------------------------------------------------

TEST(ReverseEnum, TierKnownValues) {
  // 已知枚举值一律返回规范字符串，且**忽略** tier_text。
  // 这与正向投影对称：正向在已知值时不写 tier_text（留空以免每帧多占字节）。
  EXPECT_EQ(rp::ToTierString(amr_dispatcher::proto::ChassisLinkHealth::TIER_HEALTHY, ""),
            "HEALTHY");
  EXPECT_EQ(rp::ToTierString(amr_dispatcher::proto::ChassisLinkHealth::TIER_DEGRADED, ""),
            "DEGRADED");
  EXPECT_EQ(rp::ToTierString(amr_dispatcher::proto::ChassisLinkHealth::TIER_CRITICAL, ""),
            "CRITICAL");

  // 即使 tier_text 被误填，已知枚举也不该被它覆盖——否则一个陈旧的
  // tier_text 会让消费者看到与枚举矛盾的字符串。
  EXPECT_EQ(rp::ToTierString(amr_dispatcher::proto::ChassisLinkHealth::TIER_HEALTHY,
                             "STALE_TEXT"),
            "HEALTHY");
}

TEST(ReverseEnum, TierUnknownFallsBackToText) {
  // 未知值：原文在 tier_text 里，必须还原。
  EXPECT_EQ(rp::ToTierString(amr_dispatcher::proto::ChassisLinkHealth::TIER_UNKNOWN,
                             "SATURATED"),
            "SATURATED");
  // 原文为空（正向投影对 "UNKNOWN" 刻意留空）时才返回规范串。
  EXPECT_EQ(rp::ToTierString(amr_dispatcher::proto::ChassisLinkHealth::TIER_UNKNOWN, ""),
            "UNKNOWN");
}

TEST(ReverseEnum, SeverityKnownValues) {
  EXPECT_EQ(rp::ToSeverityString(amr_dispatcher::proto::SafetyState::SEVERITY_INFO, ""), "INFO");
  EXPECT_EQ(rp::ToSeverityString(amr_dispatcher::proto::SafetyState::SEVERITY_WARN, ""), "WARN");
  EXPECT_EQ(rp::ToSeverityString(amr_dispatcher::proto::SafetyState::SEVERITY_CRITICAL, ""),
            "CRITICAL");
  EXPECT_EQ(rp::ToSeverityString(amr_dispatcher::proto::SafetyState::SEVERITY_INFO, "STALE"),
            "INFO");
}

TEST(ReverseEnum, SeverityUnknownFallsBackToText) {
  EXPECT_EQ(rp::ToSeverityString(amr_dispatcher::proto::SafetyState::SEVERITY_UNKNOWN, "PANIC"),
            "PANIC");
  EXPECT_EQ(rp::ToSeverityString(amr_dispatcher::proto::SafetyState::SEVERITY_UNKNOWN, ""),
            "UNKNOWN");
}

// ---------------------------------------------------------------------------
// Header / Time
// ---------------------------------------------------------------------------

TEST(ReverseHeader, StampAndFrameId) {
  amr_dispatcher::proto::Header ph;
  ph.mutable_stamp()->set_sec(1730000000);
  ph.mutable_stamp()->set_nanosec(123456789u);
  ph.set_frame_id("base_link");

  std_msgs::msg::Header h;
  rp::RestoreHeader(ph, &h);

  EXPECT_EQ(h.stamp.sec, 1730000000);
  EXPECT_EQ(h.stamp.nanosec, 123456789u);
  EXPECT_EQ(h.frame_id, "base_link");
}

// ---------------------------------------------------------------------------
// ChassisLinkHealth
// ---------------------------------------------------------------------------

TEST(ReverseChassisLinkHealth, AllFields) {
  amr_dispatcher::proto::ChassisLinkHealth p;
  p.mutable_header()->mutable_stamp()->set_sec(42);
  p.mutable_header()->mutable_stamp()->set_nanosec(7u);
  p.mutable_header()->set_frame_id("chassis");
  p.set_backend_name("serial");
  p.set_tier(amr_dispatcher::proto::ChassisLinkHealth::TIER_DEGRADED);
  p.set_frames_total(1000u);
  p.set_frames_success(900u);
  p.set_frames_lost(100u);
  p.set_frames_crc_error(5u);
  p.set_loss_rate(0.1);
  p.set_avg_latency_ms(12.5);
  p.set_max_latency_ms(80.0);
  p.set_is_healthy(false);
  p.set_command_delivery_rate_valid(true);
  p.set_command_delivery_rate(0.97);
  p.set_commands_sent(500u);
  p.set_commands_delivered(485u);

  ChassisLinkHealth m;
  rp::RestoreChassisLinkHealth(p, &m);

  EXPECT_EQ(m.header.stamp.sec, 42);
  EXPECT_EQ(m.header.stamp.nanosec, 7u);
  EXPECT_EQ(m.header.frame_id, "chassis");
  EXPECT_EQ(m.backend_name, "serial");
  EXPECT_EQ(m.tier, "DEGRADED");
  EXPECT_EQ(m.frames_total, 1000u);
  EXPECT_EQ(m.frames_success, 900u);
  EXPECT_EQ(m.frames_lost, 100u);
  EXPECT_EQ(m.frames_crc_error, 5u);
  EXPECT_DOUBLE_EQ(m.loss_rate, 0.1);
  EXPECT_DOUBLE_EQ(m.avg_latency_ms, 12.5);
  EXPECT_DOUBLE_EQ(m.max_latency_ms, 80.0);
  EXPECT_FALSE(m.is_healthy);
  EXPECT_DOUBLE_EQ(m.command_delivery_rate, 0.97);
  EXPECT_EQ(m.commands_sent, 500u);
  EXPECT_EQ(m.commands_delivered, 485u);
}

TEST(ReverseChassisLinkHealth, DeliveryRateInvalidRestoresMinusOneSentinel) {
  // 有损点 1：valid=false 必须还原成 -1.0，**不能**是 0.0。
  //
  // 这是本文件最重要的断言：0.0 表示「命令送达率 0%（全丢）」，-1 表示
  // 「不适用（文本协议 / MCU 未回显）」。两者在运维上结论相反。正向投影
  // 已经用 < 0.0 把 -1 翻译成 valid=false，反向若还原成 0.0，就会让
  // /metrics 与告警把「没这个能力」误报成「全线丢命令」。
  amr_dispatcher::proto::ChassisLinkHealth p;
  p.set_command_delivery_rate_valid(false);
  p.set_command_delivery_rate(0.0);  // 正向投影正是写 0.0，反向必须无视它

  ChassisLinkHealth m;
  rp::RestoreChassisLinkHealth(p, &m);

  EXPECT_DOUBLE_EQ(m.command_delivery_rate, -1.0);
}

TEST(ReverseChassisLinkHealth, TierUnknownUsesText) {
  // 有损点 2
  amr_dispatcher::proto::ChassisLinkHealth p;
  p.set_tier(amr_dispatcher::proto::ChassisLinkHealth::TIER_UNKNOWN);
  p.set_tier_text("SATURATED");

  ChassisLinkHealth m;
  rp::RestoreChassisLinkHealth(p, &m);

  EXPECT_EQ(m.tier, "SATURATED");
}

TEST(ReverseChassisLinkHealth, TierUnknownWithoutTextIsLiteralUnknown) {
  // 经典 "UNKNOWN" 走的是「正向不写 tier_text」这条路径，反向必须还原成
  // 字面量 "UNKNOWN"（而不是空串）。
  amr_dispatcher::proto::ChassisLinkHealth p;
  p.set_tier(amr_dispatcher::proto::ChassisLinkHealth::TIER_UNKNOWN);

  ChassisLinkHealth m;
  rp::RestoreChassisLinkHealth(p, &m);

  EXPECT_EQ(m.tier, "UNKNOWN");
}

// ---------------------------------------------------------------------------
// TrackingError
// ---------------------------------------------------------------------------

TEST(ReverseTrackingError, AllFields) {
  amr_dispatcher::proto::TrackingError p;
  p.mutable_header()->mutable_stamp()->set_sec(99);
  p.mutable_header()->set_frame_id("map");
  p.set_controller_name("stanley");
  p.set_lateral_error_m(0.03);
  p.set_heading_error_rad(-0.02);
  p.set_lookahead_distance_m(0.6);
  p.set_target_linear_x_mps(0.5);
  p.set_target_angular_z_radps(0.1);
  p.set_actual_linear_x_mps(0.49);
  p.set_actual_angular_z_radps(0.09);
  p.set_nearest_waypoint_index(17);
  p.set_arrived(true);

  TrackingError m;
  rp::RestoreTrackingError(p, &m);

  EXPECT_EQ(m.header.stamp.sec, 99);
  EXPECT_EQ(m.header.frame_id, "map");
  EXPECT_EQ(m.controller_name, "stanley");
  EXPECT_DOUBLE_EQ(m.lateral_error_m, 0.03);
  EXPECT_DOUBLE_EQ(m.heading_error_rad, -0.02);
  EXPECT_DOUBLE_EQ(m.lookahead_distance_m, 0.6);
  EXPECT_DOUBLE_EQ(m.target_linear_x_mps, 0.5);
  EXPECT_DOUBLE_EQ(m.target_angular_z_radps, 0.1);
  EXPECT_DOUBLE_EQ(m.actual_linear_x_mps, 0.49);
  EXPECT_DOUBLE_EQ(m.actual_angular_z_radps, 0.09);
  EXPECT_EQ(m.nearest_waypoint_index, 17);
  EXPECT_TRUE(m.arrived);
}

// ---------------------------------------------------------------------------
// VehicleLinkState
// ---------------------------------------------------------------------------

TEST(ReverseVehicleLinkState, AllFields) {
  amr_dispatcher::proto::VehicleLinkState p;
  p.set_vehicle_id("amr_01");
  p.set_backend("vda5050");
  p.set_online(true);
  p.set_busy(true);
  p.set_quarantined(false);
  p.set_position_valid(true);
  p.set_current_station_id("ST_12");
  p.set_battery_percent(87.5);
  p.set_state_age_ms(1234);
  p.set_last_error("timeout");

  VehicleLinkState m;
  rp::RestoreVehicleLinkState(p, &m);

  EXPECT_EQ(m.vehicle_id, "amr_01");
  EXPECT_EQ(m.backend, "vda5050");
  EXPECT_TRUE(m.online);
  EXPECT_TRUE(m.busy);
  EXPECT_FALSE(m.quarantined);
  EXPECT_TRUE(m.position_valid);
  EXPECT_EQ(m.current_station_id, "ST_12");
  EXPECT_DOUBLE_EQ(m.battery_percent, 87.5);
  EXPECT_EQ(m.state_age_ms, 1234);
  EXPECT_EQ(m.last_error, "timeout");
}

// ---------------------------------------------------------------------------
// DispatcherState
// ---------------------------------------------------------------------------

TEST(ReverseDispatcherState, AllScalarFields) {
  amr_dispatcher::proto::DispatcherState p;
  p.mutable_header()->mutable_stamp()->set_sec(7);
  p.mutable_header()->set_frame_id("map");
  p.set_current_comparator("nearest");
  p.set_queue_size(3u);
  p.set_queue_capacity(10u);
  p.set_active_missions_count(2u);
  p.set_paused_missions_count(1u);
  p.set_deadlock_detected(true);
  p.set_active_mission_id("m-42");
  p.set_dispatch_total(100u);
  p.set_dispatch_rejected_total(5u);
  p.set_dispatch_deferred_total(8u);
  p.set_lock_contention_total(11u);
  p.set_event_total(250u);
  p.set_overlay_active(true);
  p.set_overlay_revision(9u);
  p.set_blocked_edges(2u);
  p.set_one_way_edges(1u);

  DispatcherState m;
  rp::RestoreDispatcherState(p, &m);

  EXPECT_EQ(m.header.stamp.sec, 7);
  EXPECT_EQ(m.header.frame_id, "map");
  EXPECT_EQ(m.current_comparator, "nearest");
  EXPECT_EQ(m.queue_size, 3u);
  EXPECT_EQ(m.queue_capacity, 10u);
  EXPECT_EQ(m.active_missions_count, 2u);
  EXPECT_EQ(m.paused_missions_count, 1u);
  EXPECT_TRUE(m.deadlock_detected);
  EXPECT_EQ(m.active_mission_id, "m-42");
  EXPECT_EQ(m.dispatch_total, 100u);
  EXPECT_EQ(m.dispatch_rejected_total, 5u);
  EXPECT_EQ(m.dispatch_deferred_total, 8u);
  EXPECT_EQ(m.lock_contention_total, 11u);
  EXPECT_EQ(m.event_total, 250u);
  EXPECT_TRUE(m.overlay_active);
  EXPECT_EQ(m.overlay_revision, 9u);
  EXPECT_EQ(m.blocked_edges, 2u);
  EXPECT_EQ(m.one_way_edges, 1u);
}

TEST(ReverseDispatcherState, RepeatedStringsKeepOrderAndCount) {
  amr_dispatcher::proto::DispatcherState p;
  p.add_active_mission_ids("m-1");
  p.add_active_mission_ids("m-2");
  p.add_active_mission_ids("m-3");

  DispatcherState m;
  rp::RestoreDispatcherState(p, &m);

  ASSERT_EQ(m.active_mission_ids.size(), 3u);
  EXPECT_EQ(m.active_mission_ids[0], "m-1");
  EXPECT_EQ(m.active_mission_ids[1], "m-2");
  EXPECT_EQ(m.active_mission_ids[2], "m-3");
}

TEST(ReverseDispatcherState, VehicleLinksKeepOrderAndCount) {
  // 嵌套消息数组：条数与顺序必须严格一致。正向投影逐条 add、不重排，
  // 反向若漏一条或少一条，一致性检查会失败（这是正确行为）。
  amr_dispatcher::proto::DispatcherState p;
  auto * l1 = p.add_vehicle_links();
  l1->set_vehicle_id("amr_01");
  l1->set_online(true);
  auto * l2 = p.add_vehicle_links();
  l2->set_vehicle_id("amr_02");
  l2->set_quarantined(true);

  DispatcherState m;
  rp::RestoreDispatcherState(p, &m);

  ASSERT_EQ(m.vehicle_links.size(), 2u);
  EXPECT_EQ(m.vehicle_links[0].vehicle_id, "amr_01");
  EXPECT_TRUE(m.vehicle_links[0].online);
  EXPECT_FALSE(m.vehicle_links[0].quarantined);
  EXPECT_EQ(m.vehicle_links[1].vehicle_id, "amr_02");
  EXPECT_TRUE(m.vehicle_links[1].quarantined);
}

TEST(ReverseDispatcherState, ReuseClearsRepeatedFields) {
  // 本函数承诺「复用 out 安全」（见头文件）。若不先 clear，第二次调用
  // 会让 repeated 字段累积翻倍——这类缺陷只在复用对象的调用点暴露。
  amr_dispatcher::proto::DispatcherState p1;
  p1.add_active_mission_ids("old");
  p1.add_vehicle_links()->set_vehicle_id("old_v");

  DispatcherState m;
  rp::RestoreDispatcherState(p1, &m);
  ASSERT_EQ(m.active_mission_ids.size(), 1u);

  amr_dispatcher::proto::DispatcherState p2;
  p2.add_active_mission_ids("new");

  rp::RestoreDispatcherState(p2, &m);

  ASSERT_EQ(m.active_mission_ids.size(), 1u);
  EXPECT_EQ(m.active_mission_ids[0], "new");
  EXPECT_TRUE(m.vehicle_links.empty());
}

// ---------------------------------------------------------------------------
// SafetyState
// ---------------------------------------------------------------------------

TEST(ReverseSafetyState, AllFields) {
  amr_dispatcher::proto::SafetyState p;
  p.mutable_header()->set_frame_id("map");
  p.set_severity(amr_dispatcher::proto::SafetyState::SEVERITY_WARN);
  p.set_motion_allowed(true);
  p.set_speed_scale(0.3);
  p.set_active_stop_source("bumper");
  p.set_reason("obstacle");
  p.add_active_sources("bumper");
  p.add_active_sources("estop");

  SafetyState m;
  rp::RestoreSafetyState(p, &m);

  EXPECT_EQ(m.header.frame_id, "map");
  EXPECT_EQ(m.severity, "WARN");
  EXPECT_TRUE(m.motion_allowed);
  EXPECT_DOUBLE_EQ(m.speed_scale, 0.3);
  EXPECT_EQ(m.active_stop_source, "bumper");
  EXPECT_EQ(m.reason, "obstacle");
  ASSERT_EQ(m.active_sources.size(), 2u);
  EXPECT_EQ(m.active_sources[0], "bumper");
  EXPECT_EQ(m.active_sources[1], "estop");
}

TEST(ReverseSafetyState, SeverityCriticalIsPreserved) {
  // rest_gateway 与告警逻辑都用 severity == "CRITICAL" 做判定，字面量必须
  // 逐字相同（大小写敏感）。
  amr_dispatcher::proto::SafetyState p;
  p.set_severity(amr_dispatcher::proto::SafetyState::SEVERITY_CRITICAL);

  SafetyState m;
  rp::RestoreSafetyState(p, &m);

  EXPECT_EQ(m.severity, "CRITICAL");
}

TEST(ReverseSafetyState, SeverityUnknownUsesText) {
  // 有损点 3
  amr_dispatcher::proto::SafetyState p;
  p.set_severity(amr_dispatcher::proto::SafetyState::SEVERITY_UNKNOWN);
  p.set_severity_text("PANIC");

  SafetyState m;
  rp::RestoreSafetyState(p, &m);

  EXPECT_EQ(m.severity, "PANIC");
}

TEST(ReverseSafetyState, ReuseClearsActiveSources) {
  amr_dispatcher::proto::SafetyState p1;
  p1.add_active_sources("a");
  p1.add_active_sources("b");

  SafetyState m;
  rp::RestoreSafetyState(p1, &m);
  ASSERT_EQ(m.active_sources.size(), 2u);

  amr_dispatcher::proto::SafetyState p2;
  p2.add_active_sources("c");
  rp::RestoreSafetyState(p2, &m);

  ASSERT_EQ(m.active_sources.size(), 1u);
  EXPECT_EQ(m.active_sources[0], "c");
}

// ---------------------------------------------------------------------------
// TopologyState
// ---------------------------------------------------------------------------

TEST(ReverseTopologyState, AllFields) {
  amr_dispatcher::proto::TopologyState p;
  p.mutable_stamp()->set_sec(555);
  p.mutable_stamp()->set_nanosec(1u);
  p.add_active_edges("A-B");
  p.add_active_edges("B-C");
  p.add_locked_edges("C-D");
  p.add_edge_owners("amr_01");
  p.add_deadlocked_nodes("B");
  p.set_deadlock_detected(true);

  TopologyState m;
  rp::RestoreTopologyState(p, &m);

  EXPECT_EQ(m.stamp.sec, 555);
  EXPECT_EQ(m.stamp.nanosec, 1u);
  ASSERT_EQ(m.active_edges.size(), 2u);
  EXPECT_EQ(m.active_edges[0], "A-B");
  EXPECT_EQ(m.active_edges[1], "B-C");
  ASSERT_EQ(m.locked_edges.size(), 1u);
  EXPECT_EQ(m.locked_edges[0], "C-D");
  ASSERT_EQ(m.edge_owners.size(), 1u);
  EXPECT_EQ(m.edge_owners[0], "amr_01");
  ASSERT_EQ(m.deadlocked_nodes.size(), 1u);
  EXPECT_EQ(m.deadlocked_nodes[0], "B");
  EXPECT_TRUE(m.deadlock_detected);
}

TEST(ReverseTopologyState, EmptyRepeatedFields) {
  amr_dispatcher::proto::TopologyState p;

  TopologyState m;
  rp::RestoreTopologyState(p, &m);

  EXPECT_TRUE(m.active_edges.empty());
  EXPECT_TRUE(m.locked_edges.empty());
  EXPECT_TRUE(m.edge_owners.empty());
  EXPECT_TRUE(m.deadlocked_nodes.empty());
  EXPECT_FALSE(m.deadlock_detected);
}

// ---------------------------------------------------------------------------
// MissionEvent
// ---------------------------------------------------------------------------

TEST(ReverseMissionEvent, AllFields) {
  amr_dispatcher::proto::MissionEvent p;
  p.mutable_stamp()->set_sec(1234);
  p.mutable_stamp()->set_nanosec(5678u);
  p.set_mission_id("m-1");
  p.set_order_id("o-9");
  p.set_event("finished");
  p.set_from_state("ACTIVE");
  p.set_to_state("FINISHED");
  p.set_reason("goal_reached");

  MissionEvent m;
  rp::RestoreMissionEvent(p, &m);

  EXPECT_EQ(m.stamp.sec, 1234);
  EXPECT_EQ(m.stamp.nanosec, 5678u);
  EXPECT_EQ(m.mission_id, "m-1");
  EXPECT_EQ(m.order_id, "o-9");
  EXPECT_EQ(m.event, "finished");
  EXPECT_EQ(m.from_state, "ACTIVE");
  EXPECT_EQ(m.to_state, "FINISHED");
  EXPECT_EQ(m.reason, "goal_reached");
}

// ---------------------------------------------------------------------------
// 数值边界
// ---------------------------------------------------------------------------

TEST(ReverseNumeric, LargeUint64AndNegativeInt64Survive) {
  // 计数器会长时间累积（uint64），state_age_ms 理论上可为负（时钟回拨时的
  // 计算产物）。两侧类型宽度必须一致，否则在这里就会丢值。
  amr_dispatcher::proto::ChassisLinkHealth p;
  p.set_frames_total(std::numeric_limits<uint64_t>::max());

  ChassisLinkHealth m;
  rp::RestoreChassisLinkHealth(p, &m);
  EXPECT_EQ(m.frames_total, std::numeric_limits<uint64_t>::max());

  amr_dispatcher::proto::VehicleLinkState pv;
  pv.set_state_age_ms(-5000);

  VehicleLinkState mv;
  rp::RestoreVehicleLinkState(pv, &mv);
  EXPECT_EQ(mv.state_age_ms, -5000);
}

// ---------------------------------------------------------------------------
// 话题名派生：与发布端的常量一致性
// ---------------------------------------------------------------------------
// 本包与 amr_dispatcher_ros 各自定义了一份 kProtoTopicSuffix（见头文件注释：
// 共享同一个常量会让 tools 被迫依赖 ros 包）。重复定义的风险是两处漂移，
// 结果是「订阅成功但永远收不到消息」——DDS 不报错，只是没有匹配的发布者。
//
// amr_dispatcher_ros 侧的对应常量由 test_proto_projection.cpp 的
// ProtoTopicName 用例守住；本文件的用例守住 bridge 这一侧。
TEST(ProtoTopicNameParity, AppendsProtoSuffix) {
  namespace bridge = amr_dispatcher_proto_bridge;
  EXPECT_EQ(bridge::ProtoTopicName("/chassis/link_health"), "/chassis/link_health_proto");
  EXPECT_EQ(bridge::ProtoTopicName("/dispatcher/state"), "/dispatcher/state_proto");
  EXPECT_EQ(bridge::ProtoTopicName("/dispatcher/events"), "/dispatcher/events_proto");
  EXPECT_EQ(bridge::ProtoTopicName("/dispatcher/topology_state"),
            "/dispatcher/topology_state_proto");
  EXPECT_EQ(bridge::ProtoTopicName("/safety/state"), "/safety/state_proto");
  EXPECT_EQ(bridge::ProtoTopicName("/tracking/error"), "/tracking/error_proto");
  EXPECT_EQ(bridge::ProtoTopicName("/system/aggregated_link_health"),
            "/system/aggregated_link_health_proto");
}

TEST(ProtoTopicNameParity, PerVehicleTopicsStayDerived) {
  // 多车场景：发布端先 ResolveVehicleTopic 再追加后缀，订阅端必须拿到
  // 同一个最终名字。这里验证的正是「传入已解析名字再派生」这条约定。
  namespace bridge = amr_dispatcher_proto_bridge;
  EXPECT_EQ(bridge::ProtoTopicName("/plan/amr_01"), "/plan/amr_01_proto");
  EXPECT_EQ(bridge::ProtoTopicName("/tracking/error/amr_01"), "/tracking/error/amr_01_proto");
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
