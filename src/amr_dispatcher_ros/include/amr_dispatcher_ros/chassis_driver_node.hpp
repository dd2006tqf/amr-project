#pragma once

#include <memory>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "amr_dispatcher_core/chassis/backend_degradation.hpp"
#include "amr_dispatcher_core/chassis/chassis_packet_stream.hpp"
#include "amr_dispatcher_interfaces/msg/chassis_link_health.hpp"

namespace amr_dispatcher_ros {

class ChassisDriverNode : public rclcpp::Node {
 public:
  explicit ChassisDriverNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~ChassisDriverNode() override;

 private:
  void CmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void ReadPollLoop();
  void PublishHealth();

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<amr_dispatcher_interfaces::msg::ChassisLinkHealth>::SharedPtr health_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr heartbeat_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::TimerBase::SharedPtr poll_timer_;
  rclcpp::TimerBase::SharedPtr health_timer_;

  std::unique_ptr<amr_dispatcher_core::chassis::BackendDegradationController> backend_ctrl_;
  amr_dispatcher_core::chassis::ChassisPacketStream text_stream_;

  std::string odom_frame_id_ = "odom";
  std::string base_frame_id_ = "base_link";
  bool publish_tf_ = true;
};

}  // namespace amr_dispatcher_ros
