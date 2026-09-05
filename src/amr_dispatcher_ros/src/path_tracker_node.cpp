#include "amr_dispatcher_ros/path_tracker_node.hpp"

#include <chrono>
#include <cmath>

#include "amr_dispatcher_core/path_tracking/geometry_utils.hpp"

using namespace std::chrono_literals;

namespace amr_dispatcher_ros {

PathTrackerNode::PathTrackerNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("path_tracker_node", options) {
  declare_parameter("controller_type", "pure_pursuit");
  declare_parameter("lookahead_distance", 0.6);
  declare_parameter("target_speed", 0.5);
  declare_parameter("stanley_k", 0.5);
  declare_parameter("control_rate_hz", 20.0);

  controller_type_ = get_parameter("controller_type").as_string();

  amr_dispatcher_core::path_tracking::PurePursuitController::Config pp_cfg;
  pp_cfg.lookahead_distance = get_parameter("lookahead_distance").as_double();
  pp_cfg.target_speed = get_parameter("target_speed").as_double();
  pp_controller_ = std::make_unique<amr_dispatcher_core::path_tracking::PurePursuitController>(pp_cfg);

  amr_dispatcher_core::path_tracking::StanleyController::Config st_cfg;
  st_cfg.gain = get_parameter("stanley_k").as_double();
  st_cfg.target_speed = get_parameter("target_speed").as_double();
  stanley_controller_ = std::make_unique<amr_dispatcher_core::path_tracking::StanleyController>(st_cfg);

  path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/plan", 1,
      std::bind(&PathTrackerNode::PathCallback, this, std::placeholders::_1));

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&PathTrackerNode::OdomCallback, this, std::placeholders::_1));

  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_raw", 10);
  error_pub_ = create_publisher<amr_dispatcher_interfaces::msg::TrackingError>(
      "/tracking/error", 10);
  heartbeat_pub_ = create_publisher<std_msgs::msg::Bool>("/safety/heartbeat", 10);

  const double rate_hz = get_parameter("control_rate_hz").as_double();
  timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_hz),
      std::bind(&PathTrackerNode::ControlLoop, this));

  RCLCPP_INFO(get_logger(), "PathTrackerNode initialized with controller: %s", controller_type_.c_str());
}

PathTrackerNode::~PathTrackerNode() = default;

void PathTrackerNode::PathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
  waypoints_.clear();
  for (const auto& pose_stamped : msg->poses) {
    amr_dispatcher_core::path_tracking::Pose2D wp;
    wp.x = pose_stamped.pose.position.x;
    wp.y = pose_stamped.pose.position.y;
    wp.yaw = amr_dispatcher_core::path_tracking::YawFromQuaternion(
        pose_stamped.pose.orientation.z, pose_stamped.pose.orientation.w);
    waypoints_.push_back(wp);
  }
  has_path_ = !waypoints_.empty();
  if (pp_controller_) pp_controller_->SetPath(waypoints_);
  if (stanley_controller_) stanley_controller_->SetPath(waypoints_);
  RCLCPP_INFO(get_logger(), "Received path with %zu waypoints", waypoints_.size());
}

void PathTrackerNode::OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  current_pose_.x = msg->pose.pose.position.x;
  current_pose_.y = msg->pose.pose.position.y;
  current_pose_.yaw = amr_dispatcher_core::path_tracking::YawFromQuaternion(
      msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  current_linear_speed_ = msg->twist.twist.linear.x;
  has_odom_ = true;
}

void PathTrackerNode::ControlLoop() {
  if (!has_odom_ || !has_path_ || waypoints_.empty()) {
    return;
  }

  geometry_msgs::msg::Twist cmd;
  amr_dispatcher_interfaces::msg::TrackingError err_msg;
  err_msg.header.stamp = now();
  err_msg.controller_name = controller_type_;

  if (controller_type_ == "stanley") {
    auto res = stanley_controller_->Compute(current_pose_);
    cmd.linear.x = res.cmd.linear_x;
    cmd.angular.z = res.cmd.angular_z;

    err_msg.lateral_error_m = res.lateral_error;
    err_msg.heading_error_rad = res.heading_error;
    err_msg.target_linear_x_mps = res.cmd.linear_x;
    err_msg.target_angular_z_radps = res.cmd.angular_z;
    err_msg.actual_linear_x_mps = current_linear_speed_;
    err_msg.nearest_waypoint_index = static_cast<int32_t>(res.nearest_index);
    err_msg.arrived = res.goal_reached;
  } else {
    auto res = pp_controller_->Compute(current_pose_);
    cmd.linear.x = res.cmd.linear_x;
    cmd.angular.z = res.cmd.angular_z;

    err_msg.lateral_error_m = res.lateral_error;
    err_msg.heading_error_rad = res.heading_error;
    err_msg.target_linear_x_mps = res.cmd.linear_x;
    err_msg.target_angular_z_radps = res.cmd.angular_z;
    err_msg.actual_linear_x_mps = current_linear_speed_;
    err_msg.nearest_waypoint_index = static_cast<int32_t>(res.nearest_index);
    err_msg.arrived = res.goal_reached;
  }

  cmd_vel_pub_->publish(cmd);
  error_pub_->publish(err_msg);

  if (heartbeat_pub_) {
    std_msgs::msg::Bool hb;
    hb.data = true;
    heartbeat_pub_->publish(hb);
  }

  if (err_msg.arrived) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Goal reached, stopping robot");
    has_path_ = false;
  }
}

}  // namespace amr_dispatcher_ros

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<amr_dispatcher_ros::PathTrackerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
