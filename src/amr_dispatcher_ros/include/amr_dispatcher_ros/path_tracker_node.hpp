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

  // 里程计原点在 map 帧下的静态位姿（默认全 0 = 沿用旧行为：odom 原点即 map 原点）。
  //
  // 为什么需要：dispatcher 发布的 /plan 在 map 帧，坐标直接取 stations.yaml；
  // 而本节点把 /odom 的位姿当作当前位置直接用，没有任何 tf 查询。
  // 仿真里成立（PTY 模拟器可指定任意初始位姿，直接摆在站点上），
  // 真机却从编码器 0 开始 —— 车放在 A 站，odom 是 (0,0,0) 而 map 认为 A 在 (1,2)，
  // 于是车会朝反方向开出 2 米多。给一台位置固定的台架，这层偏移就是常量。
  double odom_origin_x_ = 0.0;
  double odom_origin_y_ = 0.0;
  double odom_origin_yaw_ = 0.0;
  bool warned_frame_mismatch_ = false;

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
