#pragma once

#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include "amr_dispatcher_core/path_tracking/path_geometry.hpp"
#include "amr_dispatcher_core/path_tracking/pure_pursuit.hpp"
#include "amr_dispatcher_core/path_tracking/stanley.hpp"
#include "amr_dispatcher_interfaces/msg/tracking_error.hpp"

namespace amr_dispatcher_ros {

class PathTrackerNode : public rclcpp::Node {
 public:
  explicit PathTrackerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~PathTrackerNode() override;

 private:
  void PathCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void ControlLoop();

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<amr_dispatcher_interfaces::msg::TrackingError>::SharedPtr error_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr heartbeat_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::string controller_type_ = "pure_pursuit";  // pure_pursuit or stanley
  std::unique_ptr<amr_dispatcher_core::path_tracking::PurePursuitController> pp_controller_;
  std::unique_ptr<amr_dispatcher_core::path_tracking::StanleyController> stanley_controller_;

  std::vector<amr_dispatcher_core::path_tracking::Pose2D> waypoints_;
  amr_dispatcher_core::path_tracking::Pose2D current_pose_;
  double current_linear_speed_ = 0.0;
  bool has_odom_ = false;
  bool has_path_ = false;

  // ---- P0.6 / A4：per-vehicle 话题隔离 ----
  // vehicle_id 为空（默认）= 单车兼容模式，话题名逐字沿用改动前的全局名。
  // 非空时按 vehicle_topic_names.hpp 的规则派生 /plan/{vid} 等话题。
  std::string vehicle_id_;
  // 已解析后的最终话题名（启动期一次性定稿并打印，便于现场核对接线）
  std::string plan_topic_{"/plan"};
  std::string odom_topic_{"/odom"};
  std::string cmd_vel_topic_{"/cmd_vel_raw"};
  std::string tracking_error_topic_{"/tracking/error"};
  std::string heartbeat_topic_{"/safety/heartbeat/path_tracker"};
};

}  // namespace amr_dispatcher_ros
