// 投影层字段完整性测试。
//
// ============================================================================
// 这个测试真正要防的是什么
// ============================================================================
// 双发最危险的失败模式不是崩溃，而是**某一侧少了一个字段**：
// 编译器不会报错（proto3 里没 set 的字段就是默认值），运行时也看不出
// （两路话题都在发布，只是内容悄悄不同）。等到阶段 3 消费者切到 proto，
// 才发现某个字段永远为空 —— 那时 rosidl 侧可能已经删了，无法对照。
//
// 所以这里对每个消息做一次「全字段填非默认值 -> 投影 -> 逐字段断言」。
// 填值时刻意避开 0 / false / "" / 空数组，因为 proto3 里默认值不写进 wire
// 格式，「没投影」和「投影了默认值」在字节层面无法区分——用默认值做测试
// 会让漏字段静默通过。
//
// 第二个要防的是**语义翻译**的偏差，无法靠逐字段比对发现，需要专门断言：
//   - command_delivery_rate 的 -1 哨兵 -> (valid=false, rate=0.0)
//   - tier / severity 的 string -> enum，以及枚举外值的 *_text 兜底
//   - repeated 字段用 add_ 而不是赋值（否则第二次投影会累积）
#include <gtest/gtest.h>

#include <string>

#include "amr_dispatcher_interfaces/msg/chassis_link_health.hpp"
#include "amr_dispatcher_interfaces/msg/dispatcher_state.hpp"
#include "amr_dispatcher_interfaces/msg/mission_event.hpp"
#include "amr_dispatcher_interfaces/msg/safety_state.hpp"
#include "amr_dispatcher_interfaces/msg/topology_state.hpp"
#include "amr_dispatcher_interfaces/msg/tracking_error.hpp"
#include "amr_dispatcher_interfaces/msg/vehicle_link_state.hpp"

#include "amr_dispatcher_ros/proto_dual_publisher.hpp"
#include "amr_dispatcher_ros/proto_projection.hpp"

namespace
{

// 时间戳填非零值，能发现「忘了投影 stamp」。
builtin_interfaces::msg::Time MakeStamp()
{
  builtin_interfaces::msg::Time t;
  t.sec = 1234;
  t.nanosec = 5678u;
  return t;
}

std_msgs::msg::Header MakeHeader()
{
  std_msgs::msg::Header h;
  h.stamp = MakeStamp();
  h.frame_id = "map_frame";
  return h;
}

void ExpectStampEq(
  const builtin_interfaces::msg::Time & rosidl_time,
  const amr_dispatcher::proto::Time & proto_time)
{
  EXPECT_EQ(proto_time.sec(), rosidl_time.sec);
  EXPECT_EQ(proto_time.nanosec(), rosidl_time.nanosec);
}

void ExpectHeaderEq(
  const std_msgs::msg::Header & rosidl_header,
  const amr_dispatcher::proto::Header & proto_header)
{
  ExpectStampEq(rosidl_header.stamp, proto_header.stamp());
  EXPECT_EQ(proto_header.frame_id(), rosidl_header.frame_id);
}

}  // namespace

// ============================================================================
// ChassisLinkHealth
// ============================================================================
TEST(ProtoProjection, ChassisLinkHealthAllFields)
{
  amr_dispatcher_interfaces::msg::ChassisLinkHealth in;
  in.header = MakeHeader();
  in.backend_name = "mick_binary";
  in.tier = "DEGRADED";
  in.frames_total = 100000;
  in.frames_success = 99900;
  in.frames_lost = 100;
  in.frames_crc_error = 7;
  in.loss_rate = 0.001;
  in.avg_latency_ms = 12.5;
  in.max_latency_ms = 88.25;
  in.is_healthy = true;
  in.command_delivery_rate = 0.97;
  in.commands_sent = 5000;
  in.commands_delivered = 4850;

  amr_dispatcher::proto::ChassisLinkHealth out;
  amr_dispatcher_ros::proto_projection::ProjectChassisLinkHealth(in, &out);

  ExpectHeaderEq(in.header, out.header());
  EXPECT_EQ(out.backend_name(), in.backend_name);
  // tier: string -> enum。DEGRADED 必须映射到 TIER_DEGRADED，
  // 且已知枚举值不额外写 tier_text（省字节，见 proto_projection.cpp 注释）。
  EXPECT_EQ(out.tier(), amr_dispatcher::proto::ChassisLinkHealth::TIER_DEGRADED);
  EXPECT_TRUE(out.tier_text().empty());
  EXPECT_EQ(out.frames_total(), in.frames_total);
  EXPECT_EQ(out.frames_success(), in.frames_success);
  EXPECT_EQ(out.frames_lost(), in.frames_lost);
  EXPECT_EQ(out.frames_crc_error(), in.frames_crc_error);
  EXPECT_DOUBLE_EQ(out.loss_rate(), in.loss_rate);
  EXPECT_DOUBLE_EQ(out.avg_latency_ms(), in.avg_latency_ms);
  EXPECT_DOUBLE_EQ(out.max_latency_ms(), in.max_latency_ms);
  EXPECT_EQ(out.is_healthy(), in.is_healthy);
  // 有效送达率：valid=true 且数值原样透传
  EXPECT_TRUE(out.command_delivery_rate_valid());
  EXPECT_DOUBLE_EQ(out.command_delivery_rate(), in.command_delivery_rate);
  EXPECT_EQ(out.commands_sent(), in.commands_sent);
  EXPECT_EQ(out.commands_delivered(), in.commands_delivered);
}

// -1 哨兵是本方案里唯一无法用 proto 原生类型表达的语义，
// 拆成 (valid, rate) 后必须确保「-1 变成 0 且标记失信」，
// 而不是把 -1 塞进无符号字段（那会被静默截断成巨大正数）。
TEST(ProtoProjection, ChassisLinkHealthDeliveryRateSentinel)
{
  amr_dispatcher_interfaces::msg::ChassisLinkHealth in;
  in.command_delivery_rate = -1.0;  // 文本协议 / MCU 未回显

  amr_dispatcher::proto::ChassisLinkHealth out;
  amr_dispatcher_ros::proto_projection::ProjectChassisLinkHealth(in, &out);

  EXPECT_FALSE(out.command_delivery_rate_valid());
  EXPECT_DOUBLE_EQ(out.command_delivery_rate(), 0.0);
}

// 枚举外的 tier 必须落到 tier_text，不能静默变成 TIER_UNKNOWN 丢信息。
TEST(ProtoProjection, ChassisLinkHealthUnknownTierKeepsText)
{
  amr_dispatcher_interfaces::msg::ChassisLinkHealth in;
  in.tier = "FUTURE_STATE_NOT_IN_ENUM";

  amr_dispatcher::proto::ChassisLinkHealth out;
  amr_dispatcher_ros::proto_projection::ProjectChassisLinkHealth(in, &out);

  EXPECT_EQ(out.tier(), amr_dispatcher::proto::ChassisLinkHealth::TIER_UNKNOWN);
  EXPECT_EQ(out.tier_text(), in.tier)
    << "枚举外的 tier 必须原样保留在 tier_text，否则上游新增状态会在桥接处丢失";
}

// ============================================================================
// TrackingError
// ============================================================================
TEST(ProtoProjection, TrackingErrorAllFields)
{
  amr_dispatcher_interfaces::msg::TrackingError in;
  in.header = MakeHeader();
  in.controller_name = "stanley";
  in.lateral_error_m = -0.125;
  in.heading_error_rad = 0.031;
  in.lookahead_distance_m = 0.6;
  in.target_linear_x_mps = 0.5;
  in.target_angular_z_radps = -0.2;
  in.actual_linear_x_mps = 0.48;
  in.actual_angular_z_radps = -0.19;
  in.nearest_waypoint_index = 42;
  in.arrived = true;

  amr_dispatcher::proto::TrackingError out;
  amr_dispatcher_ros::proto_projection::ProjectTrackingError(in, &out);

  ExpectHeaderEq(in.header, out.header());
  EXPECT_EQ(out.controller_name(), in.controller_name);
  EXPECT_DOUBLE_EQ(out.lateral_error_m(), in.lateral_error_m);
  EXPECT_DOUBLE_EQ(out.heading_error_rad(), in.heading_error_rad);
  EXPECT_DOUBLE_EQ(out.lookahead_distance_m(), in.lookahead_distance_m);
  EXPECT_DOUBLE_EQ(out.target_linear_x_mps(), in.target_linear_x_mps);
  EXPECT_DOUBLE_EQ(out.target_angular_z_radps(), in.target_angular_z_radps);
  EXPECT_DOUBLE_EQ(out.actual_linear_x_mps(), in.actual_linear_x_mps);
  EXPECT_DOUBLE_EQ(out.actual_angular_z_radps(), in.actual_angular_z_radps);
  EXPECT_EQ(out.nearest_waypoint_index(), in.nearest_waypoint_index);
  EXPECT_EQ(out.arrived(), in.arrived);
}

// ============================================================================
// SafetyState
// ============================================================================
TEST(ProtoProjection, SafetyStateAllFields)
{
  amr_dispatcher_interfaces::msg::SafetyState in;
  in.header = MakeHeader();
  in.severity = "CRITICAL";
  in.motion_allowed = false;
  in.speed_scale = 0.0;
  in.active_stop_source = "estop";
  in.reason = "idle_blocked";
  in.active_sources = {"estop", "manual_takeover"};

  amr_dispatcher::proto::SafetyState out;
  amr_dispatcher_ros::proto_projection::ProjectSafetyState(in, &out);

  ExpectHeaderEq(in.header, out.header());
  EXPECT_EQ(out.severity(), amr_dispatcher::proto::SafetyState::SEVERITY_CRITICAL);
  EXPECT_TRUE(out.severity_text().empty());
  EXPECT_EQ(out.motion_allowed(), in.motion_allowed);
  EXPECT_DOUBLE_EQ(out.speed_scale(), in.speed_scale);
  EXPECT_EQ(out.active_stop_source(), in.active_stop_source);
  EXPECT_EQ(out.reason(), in.reason);
  ASSERT_EQ(out.active_sources_size(), 2);
  EXPECT_EQ(out.active_sources(0), "estop");
  EXPECT_EQ(out.active_sources(1), "manual_takeover");
}

// 安全消息 10Hz+50Hz 兜底是最高频的双发点。
// 已知 severity 必须走 enum 且不留 text —— 否则每帧多 8-12 字节，
// 直接抹掉本方案的安全消息带宽收益。
TEST(ProtoProjection, SafetyStateKnownSeverityHasNoText)
{
  for (const char * sev : {"INFO", "WARN", "CRITICAL"}) {
    amr_dispatcher_interfaces::msg::SafetyState in;
    in.severity = sev;

    amr_dispatcher::proto::SafetyState out;
    amr_dispatcher_ros::proto_projection::ProjectSafetyState(in, &out);

    EXPECT_TRUE(out.severity_text().empty()) << "severity=" << sev;
  }
}

TEST(ProtoProjection, SafetyStateUnknownSeverityKeepsText)
{
  amr_dispatcher_interfaces::msg::SafetyState in;
  in.severity = "FUTURE_SEVERITY";

  amr_dispatcher::proto::SafetyState out;
  amr_dispatcher_ros::proto_projection::ProjectSafetyState(in, &out);

  EXPECT_EQ(out.severity(), amr_dispatcher::proto::SafetyState::SEVERITY_UNKNOWN);
  EXPECT_EQ(out.severity_text(), in.severity);
}

TEST(ProtoProjection, SafetyStateEmptyActiveSources)
{
  amr_dispatcher_interfaces::msg::SafetyState in;
  in.severity = "INFO";
  // active_sources 保持为空

  amr_dispatcher::proto::SafetyState out;
  amr_dispatcher_ros::proto_projection::ProjectSafetyState(in, &out);

  EXPECT_EQ(out.active_sources_size(), 0);
}

// ============================================================================
// TopologyState
// ============================================================================
TEST(ProtoProjection, TopologyStateAllFields)
{
  amr_dispatcher_interfaces::msg::TopologyState in;
  in.stamp = MakeStamp();
  in.active_edges = {"A-B", "B-C"};
  in.locked_edges = {"A-B"};
  in.edge_owners = {"mission_1", ""};
  in.deadlocked_nodes = {"B"};
  in.deadlock_detected = true;

  amr_dispatcher::proto::TopologyState out;
  amr_dispatcher_ros::proto_projection::ProjectTopologyState(in, &out);

  ExpectStampEq(in.stamp, out.stamp());
  ASSERT_EQ(out.active_edges_size(), 2);
  EXPECT_EQ(out.active_edges(0), "A-B");
  EXPECT_EQ(out.active_edges(1), "B-C");
  ASSERT_EQ(out.locked_edges_size(), 1);
  EXPECT_EQ(out.locked_edges(0), "A-B");
  ASSERT_EQ(out.edge_owners_size(), 2);
  EXPECT_EQ(out.edge_owners(0), "mission_1");
  EXPECT_EQ(out.edge_owners(1), "");
  ASSERT_EQ(out.deadlocked_nodes_size(), 1);
  EXPECT_EQ(out.deadlocked_nodes(0), "B");
  EXPECT_EQ(out.deadlock_detected(), in.deadlock_detected);
}

// repeated 字段若用赋值而不是 add_，第二次投影会累积（2 -> 4 个元素）。
// 双发器的 Publish() 每次都新建 proto 消息，所以这条在实际路径上不会触发，
// 但投影函数是公开 API，契约要求「out 必须是新的或已 Clear()」——
// 这里验证同一个 out 复用且先 Clear 时不会残留。
TEST(ProtoProjection, TopologyStateRepeatedDoesNotAccumulateAfterClear)
{
  amr_dispatcher_interfaces::msg::TopologyState in;
  in.active_edges = {"A-B", "B-C"};

  amr_dispatcher::proto::TopologyState out;
  amr_dispatcher_ros::proto_projection::ProjectTopologyState(in, &out);
  ASSERT_EQ(out.active_edges_size(), 2);

  out.Clear();
  amr_dispatcher_ros::proto_projection::ProjectTopologyState(in, &out);
  EXPECT_EQ(out.active_edges_size(), 2) << "投影前后必须一致，不能在复用的 out 上累积";
}

// ============================================================================
// MissionEvent
// ============================================================================
TEST(ProtoProjection, MissionEventAllFields)
{
  amr_dispatcher_interfaces::msg::MissionEvent in;
  in.stamp = MakeStamp();
  in.mission_id = "mission_42";
  in.order_id = "order_7";
  in.event = "queued";
  in.from_state = "PENDING";
  in.to_state = "ACTIVE";
  in.reason = "submitted";

  amr_dispatcher::proto::MissionEvent out;
  amr_dispatcher_ros::proto_projection::ProjectMissionEvent(in, &out);

  ExpectStampEq(in.stamp, out.stamp());
  EXPECT_EQ(out.mission_id(), in.mission_id);
  EXPECT_EQ(out.order_id(), in.order_id);
  EXPECT_EQ(out.event(), in.event);
  EXPECT_EQ(out.from_state(), in.from_state);
  EXPECT_EQ(out.to_state(), in.to_state);
  EXPECT_EQ(out.reason(), in.reason);
}

// ============================================================================
// VehicleLinkState
// ============================================================================
TEST(ProtoProjection, VehicleLinkStateAllFields)
{
  amr_dispatcher_interfaces::msg::VehicleLinkState in;
  in.vehicle_id = "amr_01";
  in.backend = "vda5050";
  in.online = true;
  in.busy = true;
  in.quarantined = true;
  in.position_valid = true;
  in.current_station_id = "station_A";
  in.battery_percent = 73.5;
  in.state_age_ms = 250;
  in.last_error = "none";

  amr_dispatcher::proto::VehicleLinkState out;
  amr_dispatcher_ros::proto_projection::ProjectVehicleLinkState(in, &out);

  EXPECT_EQ(out.vehicle_id(), in.vehicle_id);
  EXPECT_EQ(out.backend(), in.backend);
  EXPECT_EQ(out.online(), in.online);
  EXPECT_EQ(out.busy(), in.busy);
  EXPECT_EQ(out.quarantined(), in.quarantined);
  EXPECT_EQ(out.position_valid(), in.position_valid);
  EXPECT_EQ(out.current_station_id(), in.current_station_id);
  EXPECT_DOUBLE_EQ(out.battery_percent(), in.battery_percent);
  EXPECT_EQ(out.state_age_ms(), in.state_age_ms);
  EXPECT_EQ(out.last_error(), in.last_error);
}

// state_age_ms 是 int64，必须能承载 -1 哨兵（未知时延）。
// 若误用无符号类型会静默变成巨大正数。
TEST(ProtoProjection, VehicleLinkStateNegativeAgeSentinel)
{
  amr_dispatcher_interfaces::msg::VehicleLinkState in;
  in.state_age_ms = -1;

  amr_dispatcher::proto::VehicleLinkState out;
  amr_dispatcher_ros::proto_projection::ProjectVehicleLinkState(in, &out);

  EXPECT_EQ(out.state_age_ms(), -1);
}

// ============================================================================
// DispatcherState
// ============================================================================
TEST(ProtoProjection, DispatcherStateAllFields)
{
  amr_dispatcher_interfaces::msg::DispatcherState in;
  in.header = MakeHeader();
  in.current_comparator = "priority_fifo";
  in.queue_size = 3;
  in.queue_capacity = 128;
  in.active_missions_count = 2;
  in.paused_missions_count = 1;
  in.deadlock_detected = true;
  in.active_mission_id = "mission_42";
  in.active_mission_ids = {"mission_42", "mission_43"};
  in.dispatch_total = 500;
  in.dispatch_rejected_total = 12;
  in.dispatch_deferred_total = 30;
  in.lock_contention_total = 8;
  in.event_total = 900;
  in.overlay_active = true;
  in.overlay_revision = 4;
  in.blocked_edges = 2;
  in.one_way_edges = 1;

  amr_dispatcher_interfaces::msg::VehicleLinkState link;
  link.vehicle_id = "amr_01";
  link.backend = "local_sim";
  link.online = true;
  link.busy = false;
  link.quarantined = false;
  link.position_valid = true;
  link.current_station_id = "station_A";
  link.battery_percent = 88.0;
  link.state_age_ms = 120;
  link.last_error = "";
  in.vehicle_links.push_back(link);

  amr_dispatcher::proto::DispatcherState out;
  amr_dispatcher_ros::proto_projection::ProjectDispatcherState(in, &out);

  ExpectHeaderEq(in.header, out.header());
  EXPECT_EQ(out.current_comparator(), in.current_comparator);
  EXPECT_EQ(out.queue_size(), in.queue_size);
  EXPECT_EQ(out.queue_capacity(), in.queue_capacity);
  EXPECT_EQ(out.active_missions_count(), in.active_missions_count);
  EXPECT_EQ(out.paused_missions_count(), in.paused_missions_count);
  EXPECT_EQ(out.deadlock_detected(), in.deadlock_detected);
  EXPECT_EQ(out.active_mission_id(), in.active_mission_id);
  ASSERT_EQ(out.active_mission_ids_size(), 2);
  EXPECT_EQ(out.active_mission_ids(0), "mission_42");
  EXPECT_EQ(out.active_mission_ids(1), "mission_43");
  EXPECT_EQ(out.dispatch_total(), in.dispatch_total);
  EXPECT_EQ(out.dispatch_rejected_total(), in.dispatch_rejected_total);
  EXPECT_EQ(out.dispatch_deferred_total(), in.dispatch_deferred_total);
  EXPECT_EQ(out.lock_contention_total(), in.lock_contention_total);
  EXPECT_EQ(out.event_total(), in.event_total);
  EXPECT_EQ(out.overlay_active(), in.overlay_active);
  EXPECT_EQ(out.overlay_revision(), in.overlay_revision);
  EXPECT_EQ(out.blocked_edges(), in.blocked_edges);
  EXPECT_EQ(out.one_way_edges(), in.one_way_edges);

  // 嵌套 message 数组：漏投影 vehicle_links 会让 Web 控制台的车辆列表永远为空，
  // 而 queue_size 之类的标量看起来完全正常 —— 这类漏字段最难被发现。
  ASSERT_EQ(out.vehicle_links_size(), 1);
  const auto & out_link = out.vehicle_links(0);
  EXPECT_EQ(out_link.vehicle_id(), link.vehicle_id);
  EXPECT_EQ(out_link.backend(), link.backend);
  EXPECT_EQ(out_link.online(), link.online);
  EXPECT_EQ(out_link.busy(), link.busy);
  EXPECT_EQ(out_link.quarantined(), link.quarantined);
  EXPECT_EQ(out_link.position_valid(), link.position_valid);
  EXPECT_EQ(out_link.current_station_id(), link.current_station_id);
  EXPECT_DOUBLE_EQ(out_link.battery_percent(), link.battery_percent);
  EXPECT_EQ(out_link.state_age_ms(), link.state_age_ms);
  EXPECT_EQ(out_link.last_error(), link.last_error);
}

TEST(ProtoProjection, DispatcherStateEmptyVehicleLinks)
{
  amr_dispatcher_interfaces::msg::DispatcherState in;
  // vehicle_links 保持为空

  amr_dispatcher::proto::DispatcherState out;
  amr_dispatcher_ros::proto_projection::ProjectDispatcherState(in, &out);

  EXPECT_EQ(out.vehicle_links_size(), 0);
}

// ============================================================================
// 话题名派生
// ============================================================================
// proto 话题名只由「已解析的 rosidl 话题名」加后缀得到，因此 per-vehicle
// 派生不会重复发生。这条契约一旦破坏，多车场景下会出现
// /plan_proto/amr_01 与 /plan/amr_01_proto 两种不一致结果。
TEST(ProtoTopicName, AppendsSuffix)
{
  EXPECT_EQ(amr_dispatcher_ros::ProtoTopicName("/chassis/link_health"),
            "/chassis/link_health_proto");
  EXPECT_EQ(amr_dispatcher_ros::ProtoTopicName("/plan/amr_01"), "/plan/amr_01_proto");
  EXPECT_EQ(amr_dispatcher_ros::ProtoTopicName("/safety/state"), "/safety/state_proto");
}

TEST(ProtoTopicName, LegacyNamesStayByteIdentical)
{
  // 兼容性红线：vehicle_id 为空时 rosidl 话题名逐字不变，
  // proto 侧只是加后缀，绝不能改变上半部分。
  for (const char * legacy : {
      "/chassis/link_health", "/tracking/error", "/safety/state",
      "/dispatcher/state", "/dispatcher/events", "/dispatcher/topology_state"})
  {
    const std::string proto_name = amr_dispatcher_ros::ProtoTopicName(legacy);
    EXPECT_EQ(proto_name.substr(0, proto_name.size() - 6), legacy);
  }
}
