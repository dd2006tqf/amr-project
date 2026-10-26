#include "amr_dispatcher_ros/chassis_driver_node.hpp"

#include <chrono>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include "amr_dispatcher_ros/vehicle_topic_names.hpp"

using namespace std::chrono_literals;

namespace amr_dispatcher_ros {

ChassisDriverNode::ChassisDriverNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("chassis_driver_node", options) {
  // 声明参数
  declare_parameter("serial_device", "/dev/ttyUSB0");
  declare_parameter("serial_baud", 115200);
  declare_parameter("udp_host", "192.168.1.10");
  declare_parameter("udp_port", 9000);
  declare_parameter("odom_frame_id", "odom");
  declare_parameter("base_frame_id", "base_link");
  declare_parameter("publish_tf", true);
  declare_parameter("poll_rate_hz", 50.0);
  // B1：健康上报提速到 5Hz（心跳 500ms 超时下留足余量）
  declare_parameter("health_rate_hz", 5.0);
  declare_parameter("cmd_stale_timeout_ms", 500);
  // ---- P0.6 / A4：per-vehicle 话题与 TF 隔离 ----
  declare_parameter("vehicle_id", "");
  declare_parameter("cmd_vel_topic", "");
  declare_parameter("odom_topic", "");
  declare_parameter("link_health_topic", "");
  declare_parameter("heartbeat_topic", "");
  // TF frame 前缀（显式覆盖用）。默认空 -> 由 vehicle_id 派生（{vid}/odom）。
  declare_parameter("odom_frame_prefix", "");
  declare_parameter("base_frame_prefix", "");

  odom_frame_id_ = get_parameter("odom_frame_id").as_string();
  base_frame_id_ = get_parameter("base_frame_id").as_string();
  publish_tf_ = get_parameter("publish_tf").as_bool();
  cmd_stale_timeout_ =
      std::chrono::milliseconds(get_parameter("cmd_stale_timeout_ms").as_int());

  // ---- P0.6 / A4：解析 vehicle_id 与最终话题名 ----
  vehicle_id_ = get_parameter("vehicle_id").as_string();
  if (!vehicle_id_.empty() && !IsValidVehicleId(vehicle_id_)) {
    RCLCPP_ERROR(get_logger(),
                 "Invalid vehicle_id '%s' (must match [A-Za-z_][A-Za-z0-9_]*); "
                 "falling back to legacy global topics",
                 vehicle_id_.c_str());
    vehicle_id_.clear();
  }
  cmd_vel_topic_ =
      ResolveVehicleTopic(get_parameter("cmd_vel_topic").as_string(), "/cmd_vel_safe", vehicle_id_);
  odom_topic_ = ResolveVehicleTopic(get_parameter("odom_topic").as_string(), "/odom", vehicle_id_);
  link_health_topic_ = ResolveVehicleTopic(get_parameter("link_health_topic").as_string(),
                                           "/chassis/link_health", vehicle_id_);
  heartbeat_topic_ = ResolveVehicleTopic(get_parameter("heartbeat_topic").as_string(),
                                         "/safety/heartbeat/chassis_driver", vehicle_id_);

  // ---- P0.6 / A4：TF frame 唯一性（硬约束）----
  // 两台车发布同一个 odom frame 是 TF 硬错误（tree 冲突）。规则（自上而下）：
  //   1) odom_frame_id / base_frame_id 被显式改成非默认值 -> 直接采用（用户负责唯一性）；
  //   2) 否则 odom_frame_prefix / base_frame_prefix 非空 -> {prefix}/odom、{prefix}/base_link；
  //   3) 否则 vehicle_id 非空 -> {vid}/odom、{vid}/base_link；
  //   4) 否则 -> 沿用默认 "odom" / "base_link"（单车兼容，frame 名逐字不变）。
  const std::string odom_frame_prefix = get_parameter("odom_frame_prefix").as_string();
  const std::string base_frame_prefix = get_parameter("base_frame_prefix").as_string();
  if (odom_frame_id_ == "odom") {
    if (!odom_frame_prefix.empty()) {
      odom_frame_id_ = odom_frame_prefix + "/odom";
    } else if (!vehicle_id_.empty()) {
      odom_frame_id_ = vehicle_id_ + "/odom";
    }
  }
  if (base_frame_id_ == "base_link") {
    if (!base_frame_prefix.empty()) {
      base_frame_id_ = base_frame_prefix + "/base_link";
    } else if (!vehicle_id_.empty()) {
      base_frame_id_ = vehicle_id_ + "/base_link";
    }
  }

  // 配置 preferred list 后端
  amr_dispatcher_core::chassis::ChassisBackendConfig cfg;
  cfg.serial_device = get_parameter("serial_device").as_string();
  cfg.serial_baud = get_parameter("serial_baud").as_int();
  cfg.udp_host = get_parameter("udp_host").as_string();
  cfg.udp_port = get_parameter("udp_port").as_int();

  backend_ctrl_ = std::make_unique<amr_dispatcher_core::chassis::BackendDegradationController>(
      amr_dispatcher_core::chassis::DefaultBackendDescriptors(),
      std::vector<amr_dispatcher_core::chassis::ChassisBackendConfig>{cfg, cfg, cfg});

  // 通信端点
  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, 10,
      std::bind(&ChassisDriverNode::CmdVelCallback, this, std::placeholders::_1));

  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 20);
  health_pub_ = create_publisher<amr_dispatcher_interfaces::msg::ChassisLinkHealth>(
      link_health_topic_, 10);
  // B1：心跳分源，只喂 chassis_driver 自己的看门狗通道
  heartbeat_pub_ = create_publisher<std_msgs::msg::Bool>(heartbeat_topic_, 10);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  const double poll_hz = get_parameter("poll_rate_hz").as_double();
  poll_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / poll_hz),
      std::bind(&ChassisDriverNode::ReadPollLoop, this));

  const double health_hz = get_parameter("health_rate_hz").as_double();
  health_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / health_hz),
      std::bind(&ChassisDriverNode::PublishHealth, this));

  RCLCPP_INFO(get_logger(), "ChassisDriverNode initialized with backend: %s",
              backend_ctrl_->current() ? backend_ctrl_->current()->Name().c_str() : "none");
  // P0.6：打印最终话题名与 TF frame——多车现场核对接线的唯一依据。
  RCLCPP_INFO(get_logger(),
              "ChassisDriverNode topics (vehicle_id=%s): cmd_vel_safe=%s odom=%s "
              "link_health=%s heartbeat=%s | TF frames: odom=%s base=%s",
              vehicle_id_.empty() ? "<legacy>" : vehicle_id_.c_str(), cmd_vel_topic_.c_str(),
              odom_topic_.c_str(), link_health_topic_.c_str(), heartbeat_topic_.c_str(),
              odom_frame_id_.c_str(), base_frame_id_.c_str());
}

ChassisDriverNode::~ChassisDriverNode() = default;

void ChassisDriverNode::CmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
  // B1：先记接收时刻再判后端——指令龄反映的是上游是否还在发，与写入成功与否无关。
  last_cmd_stamp_ = std::chrono::steady_clock::now();
  has_cmd_stamp_ = true;
  if (!backend_ctrl_ || !backend_ctrl_->current() || !backend_ctrl_->current()->IsOpen()) {
    return;
  }
  amr_dispatcher_core::chassis::ChassisCommand cmd;
  cmd.linear_x_mps = msg->linear.x;
  cmd.linear_y_mps = msg->linear.y;
  cmd.angular_z_radps = msg->angular.z;

  std::string err;
  const auto t0 = std::chrono::steady_clock::now();
  if (!backend_ctrl_->current()->WriteCommand(cmd, &err)) {
    backend_ctrl_->RecordLoss();
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "WriteCommand failed: %s", err.c_str());
  } else {
    const auto dt_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t0).count();
    backend_ctrl_->RecordSuccess(dt_ms);
  }
}

void ChassisDriverNode::ReadPollLoop() {
  if (!backend_ctrl_) return;
  if (!backend_ctrl_->current() || !backend_ctrl_->current()->IsOpen()) {
    // 连接已断（切档 Open 失败或后端关闭）：仍需驱动控制器，
    // 否则 LinkRecoveryPolicy 的退避重连永远不会被推进。
    std::string reason;
    if (backend_ctrl_->EvaluateAndMaybeSwitch(&reason)) {
      RCLCPP_WARN(get_logger(), "Chassis backend reconnected: %s", reason.c_str());
    }
    return;
  }

  // B1 指令龄兜底：后端在线但上游超过 cmd_stale_timeout_ms 未再发指令，
  // 主动写零帧，防止底盘沿用最后一次非零指令继续运动。
  if (has_cmd_stamp_) {
    const auto age = std::chrono::steady_clock::now() - last_cmd_stamp_;
    if (age > cmd_stale_timeout_) {
      amr_dispatcher_core::chassis::ChassisCommand zero_cmd;
      std::string zero_err;
      backend_ctrl_->current()->WriteCommand(zero_cmd, &zero_err);
      if (!stale_zero_active_) {
        stale_zero_active_ = true;
        RCLCPP_WARN(get_logger(),
                    "cmd_vel_safe stale for %lld ms, writing zero-frame fallback",
                    static_cast<long long>(
                        std::chrono::duration_cast<std::chrono::milliseconds>(age).count()));
      }
    } else if (stale_zero_active_) {
      stale_zero_active_ = false;  // 上游恢复，复位一次性告警
    }
  }

  std::string err;
  auto data = backend_ctrl_->current()->Read(&err);
  if (!data || data->empty()) {
    // 读路径失败计入丢帧：链路不活跃，链路质量滑窗需要感知
    if (!err.empty()) {
      backend_ctrl_->RecordLoss();
    }
    return;
  }
  text_stream_.Append(*data);
  const auto packets = text_stream_.DrainPackets();
  const auto now = this->now();

  // 解析失败行计入丢帧
  const auto invalid_lines = text_stream_.InvalidLineCount();
  for (std::size_t i = 0; i < invalid_lines; ++i) {
    backend_ctrl_->RecordLoss();
  }
  if (invalid_lines > 0) {
    text_stream_.ClearInvalidLineCount();
  }

  // 成功解析出有效包即证明链路活跃：读路径持续喂入成功率样本，
  // 否则无 cmd_vel 时 frames_total 恒为 0，链路质量永远停在 UNKNOWN。
  for (std::size_t i = 0; i < packets.size(); ++i) {
    backend_ctrl_->RecordSuccess(0.0);
  }

  for (const auto& pkt : packets) {
    if (pkt.kind != amr_dispatcher_core::chassis::ChassisPacketKind::kOdometry) {
      continue;
    }
    const auto& o = pkt.odometry;

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = now;
    odom_msg.header.frame_id = odom_frame_id_;
    odom_msg.child_frame_id = base_frame_id_;
    odom_msg.pose.pose.position.x = o.x_m;
    odom_msg.pose.pose.position.y = o.y_m;
    odom_msg.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, o.yaw_rad);
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    odom_msg.twist.twist.linear.x = o.linear_x_mps;
    odom_msg.twist.twist.angular.z = o.angular_z_radps;

    odom_pub_->publish(odom_msg);

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = now;
      tf.header.frame_id = odom_frame_id_;
      tf.child_frame_id = base_frame_id_;
      tf.transform.translation.x = o.x_m;
      tf.transform.translation.y = o.y_m;
      tf.transform.translation.z = 0.0;
      tf.transform.rotation = odom_msg.pose.pose.orientation;
      tf_broadcaster_->sendTransform(tf);
    }
  }

  // 周期性评估降级与回切
  std::string reason;
  if (backend_ctrl_->EvaluateAndMaybeSwitch(&reason)) {
    RCLCPP_WARN(get_logger(), "Chassis backend switched: %s", reason.c_str());
  }
}

void ChassisDriverNode::PublishHealth() {
  if (!backend_ctrl_) return;
  const auto snap = backend_ctrl_->quality().Snapshot();
  amr_dispatcher_interfaces::msg::ChassisLinkHealth msg;
  msg.header.stamp = this->now();
  msg.backend_name = backend_ctrl_->current() ? backend_ctrl_->current()->Name() : "none";
  msg.frames_total = snap.frames_total;
  msg.frames_success = snap.frames_success;
  msg.frames_lost = snap.frames_lost;
  msg.frames_crc_error = snap.frames_crc_error;
  msg.loss_rate = snap.loss_rate;
  msg.avg_latency_ms = snap.average_latency_ms;
  msg.max_latency_ms = snap.max_latency_ms;
  msg.is_healthy = backend_ctrl_->quality().IsHealthy();

  switch (snap.tier) {
    case amr_dispatcher_core::chassis::LinkHealthTier::kHealthy:
      msg.tier = "HEALTHY";
      break;
    case amr_dispatcher_core::chassis::LinkHealthTier::kDegraded:
      msg.tier = "DEGRADED";
      break;
    case amr_dispatcher_core::chassis::LinkHealthTier::kCritical:
      msg.tier = "CRITICAL";
      break;
    case amr_dispatcher_core::chassis::LinkHealthTier::kUnknown:
    default:
      msg.tier = "UNKNOWN";
      break;
  }
  health_pub_->publish(msg);

  if (heartbeat_pub_) {
    std_msgs::msg::Bool hb;
    hb.data = true;
    heartbeat_pub_->publish(hb);
  }
}

}  // namespace amr_dispatcher_ros

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<amr_dispatcher_ros::ChassisDriverNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
