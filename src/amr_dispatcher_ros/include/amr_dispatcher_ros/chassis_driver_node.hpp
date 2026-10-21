#pragma once

#include <chrono>
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
  // B1：最近一次收到 /cmd_vel_safe 的时刻，用于指令龄超时零速兜底
  std::chrono::steady_clock::time_point last_cmd_stamp_{};
  bool has_cmd_stamp_ = false;
  // B1：指令龄超时阈值（毫秒），ReadPollLoop 超时后主动写零帧
  std::chrono::milliseconds cmd_stale_timeout_{500};
  // B1：本轮失联零速是否已告警/已写帧（恢复后重置，避免每次 poll 刷屏）
  bool stale_zero_active_ = false;

  // ---- P0.6 / A4：per-vehicle 话题与 TF 隔离 ----
  // vehicle_id 空 = 单车兼容模式（话题名与 frame 名逐字不变）；
  // 非空 = 话题按 {vid} 派生，TF frame 按 {vid}/odom、{vid}/base_link 派生。
  std::string vehicle_id_;
  std::string cmd_vel_topic_{"/cmd_vel_safe"};
  std::string odom_topic_{"/odom"};
  std::string link_health_topic_{"/chassis/link_health"};
  std::string heartbeat_topic_{"/safety/heartbeat/chassis_driver"};
};

}  // namespace amr_dispatcher_ros
