#include "amr_dispatcher_ros/chassis_driver_node.hpp"

#include <chrono>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

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
  declare_parameter("health_rate_hz", 2.0);

  odom_frame_id_ = get_parameter("odom_frame_id").as_string();
  base_frame_id_ = get_parameter("base_frame_id").as_string();
  publish_tf_ = get_parameter("publish_tf").as_bool();

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
      "/cmd_vel_safe", 10,
      std::bind(&ChassisDriverNode::CmdVelCallback, this, std::placeholders::_1));

  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 20);
  health_pub_ = create_publisher<amr_dispatcher_interfaces::msg::ChassisLinkHealth>(
      "/chassis/link_health", 10);
  heartbeat_pub_ = create_publisher<std_msgs::msg::Bool>("/safety/heartbeat", 10);
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
}

ChassisDriverNode::~ChassisDriverNode() = default;

void ChassisDriverNode::CmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
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
  if (!backend_ctrl_ || !backend_ctrl_->current() || !backend_ctrl_->current()->IsOpen()) {
    return;
  }
  std::string err;
  auto data = backend_ctrl_->current()->Read(&err);
  if (!data || data->empty()) {
    return;
  }
  text_stream_.Append(*data);
  const auto packets = text_stream_.DrainPackets();
  const auto now = this->now();

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

  // 周期性评估是否降级
  std::string reason;
  if (backend_ctrl_->EvaluateAndMaybeSwitch(&reason)) {
    RCLCPP_WARN(get_logger(), "Chassis backend degraded/switched: %s", reason.c_str());
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
