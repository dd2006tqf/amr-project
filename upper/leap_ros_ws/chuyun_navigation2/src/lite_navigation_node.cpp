#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <deque>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include "chuyun_navigation2/a_star_planner.hpp"
#include "chuyun_navigation2/correlative_scan_matcher.hpp"
#include "chuyun_navigation2/grid_map.hpp"
#include "chuyun_navigation2/path_follower.hpp"
#include "chuyun_navigation2/types.hpp"

namespace chuyun_navigation2
{
namespace
{

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & quaternion)
{
  return std::atan2(
    2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
    1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z));
}

geometry_msgs::msg::Quaternion quaternionFromYaw(double yaw)
{
  geometry_msgs::msg::Quaternion quaternion;
  quaternion.z = std::sin(yaw * 0.5);
  quaternion.w = std::cos(yaw * 0.5);
  return quaternion;
}

geometry_msgs::msg::Twist toTwist(const Velocity2D & velocity)
{
  geometry_msgs::msg::Twist message;
  message.linear.x = velocity.x;
  message.linear.y = velocity.y;
  message.angular.z = velocity.yaw;
  return message;
}

}  // namespace

class LiteNavigationNode : public rclcpp::Node
{
public:
  LiteNavigationNode()
  : Node("lite_navigation"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_),
    tf_broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(*this))
  {
    const std::string map_yaml = declare_parameter<std::string>("map_yaml", "");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odometry/filtered");
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");

    control_frequency_ = declare_parameter<double>("control_frequency", 12.0);
    localization_frequency_ = declare_parameter<double>("localization_frequency", 4.0);
    scan_timeout_ = declare_parameter<double>("scan_timeout", 0.8);
    laser_min_range_ = declare_parameter<double>("localization_laser_min_range", 0.10);
    laser_max_range_ = declare_parameter<double>("localization_laser_max_range", 8.0);
    tf_lookup_timeout_ = declare_parameter<double>("tf_lookup_timeout", 0.05);

    const double inflation_radius = declare_parameter<double>("inflation_radius", 0.24);
    const double likelihood_max_distance =
      declare_parameter<double>("likelihood_max_distance", 2.0);

    PlannerParameters planner_parameters;
    planner_parameters.path_point_spacing =
      declare_parameter<double>("path_point_spacing", 0.06);
    planner_parameters.path_smoothing_iterations =
      declare_parameter<int>("path_smoothing_iterations", 2);
    planner_parameters.preferred_clearance =
      declare_parameter<double>("preferred_clearance", 0.40);
    planner_parameters.clearance_cost_weight =
      declare_parameter<double>("clearance_cost_weight", 6.0);
    planner_parameters.nearest_free_radius =
      declare_parameter<double>("nearest_free_radius", 0.60);
    planner_parameters.temporary_obstacle_radius =
      declare_parameter<double>("dynamic_obstacle_radius", 0.38);
    planner_parameters.temporary_clearance_radius =
      declare_parameter<double>("dynamic_clearance_radius", 0.70);
    planner_parameters.temporary_clearance_cost_weight =
      declare_parameter<double>("dynamic_clearance_cost_weight", 10.0);

    ControllerParameters controller_parameters;
    controller_parameters.max_vx = declare_parameter<double>("max_vx", 0.18);
    controller_parameters.max_vy = declare_parameter<double>("max_vy", 0.12);
    controller_parameters.max_reverse_speed =
      declare_parameter<double>("max_reverse_speed", 0.08);
    controller_parameters.max_wz = declare_parameter<double>("max_wz", 0.70);
    controller_parameters.max_xy_speed = declare_parameter<double>("max_xy_speed", 0.18);
    controller_parameters.max_ax = declare_parameter<double>("max_ax", 0.35);
    controller_parameters.max_ay = declare_parameter<double>("max_ay", 0.30);
    controller_parameters.max_aw = declare_parameter<double>("max_aw", 1.00);
    controller_parameters.linear_gain = declare_parameter<double>("linear_gain", 1.0);
    controller_parameters.angular_gain = declare_parameter<double>("angular_gain", 1.4);
    controller_parameters.lookahead_distance =
      declare_parameter<double>("lookahead_distance", 0.24);
    controller_parameters.rotate_before_translate_angle =
      declare_parameter<double>("rotate_before_translate_angle", 1.05);
    controller_parameters.heading_deadband =
      declare_parameter<double>("heading_deadband", 0.70);
    controller_parameters.path_search_window =
      declare_parameter<int>("path_search_window", 80);
    controller_parameters.goal_position_tolerance =
      declare_parameter<double>("goal_position_tolerance", 0.10);
    controller_parameters.goal_yaw_tolerance =
      declare_parameter<double>("goal_yaw_tolerance", 0.12);

    LocalPlannerParameters local_planner_parameters;
    local_planner_parameters.max_vx = controller_parameters.max_vx;
    local_planner_parameters.max_vy = controller_parameters.max_vy;
    local_planner_parameters.max_wz = controller_parameters.max_wz;
    local_planner_parameters.max_reverse_speed = controller_parameters.max_reverse_speed;
    local_planner_parameters.robot_radius =
      declare_parameter<double>("local_robot_radius", 0.18);
    local_planner_parameters.safety_margin =
      declare_parameter<double>("local_safety_margin", 0.03);
    local_planner_parameters.activation_distance =
      declare_parameter<double>("local_activation_distance", 0.75);
    local_planner_parameters.preferred_clearance =
      declare_parameter<double>("local_preferred_clearance", 0.22);
    local_planner_parameters.prediction_horizon =
      declare_parameter<double>("local_prediction_horizon", 1.20);
    local_planner_parameters.simulation_step =
      declare_parameter<double>("local_simulation_step", 0.12);
    local_planner_parameters.braking_deceleration =
      declare_parameter<double>("local_braking_deceleration", 0.35);
    local_planner_parameters.vx_samples = declare_parameter<int>("local_vx_samples", 5);
    local_planner_parameters.vy_samples = declare_parameter<int>("local_vy_samples", 7);
    local_planner_parameters.wz_samples = declare_parameter<int>("local_wz_samples", 9);
    local_planner_parameters.target_weight =
      declare_parameter<double>("local_target_weight", 3.0);
    local_planner_parameters.heading_weight =
      declare_parameter<double>("local_heading_weight", 0.8);
    local_planner_parameters.clearance_weight =
      declare_parameter<double>("local_clearance_weight", 1.2);
    local_planner_parameters.desired_velocity_weight =
      declare_parameter<double>("local_desired_velocity_weight", 0.7);
    local_planner_parameters.progress_weight =
      declare_parameter<double>("local_progress_weight", 2.0);
    local_planner_parameters.change_weight =
      declare_parameter<double>("local_change_weight", 0.25);
    local_planner_parameters.hysteresis_weight =
      declare_parameter<double>("local_hysteresis_weight", 0.35);
    local_planner_parameters.avoidance_commitment_weight =
      declare_parameter<double>("local_avoidance_commitment_weight", 0.60);
    local_planner_parameters.reverse_weight =
      declare_parameter<double>("local_reverse_weight", 0.80);
    local_planner_parameters.stalled_translation_penalty =
      declare_parameter<double>("local_stalled_translation_penalty", 0.70);
    local_planner_parameters.recovery_lateral_weight =
      declare_parameter<double>("local_recovery_lateral_weight", 1.40);
    local_planner_parameters.recovery_speed =
      declare_parameter<double>("local_recovery_speed", 0.08);
    local_planner_parameters.recovery_distance =
      declare_parameter<double>("local_recovery_distance", 0.35);
    local_planner_parameters.recovery_minimum_improvement =
      declare_parameter<double>("local_recovery_minimum_improvement", 0.03);
    local_planner_parameters.recovery_direction_samples =
      declare_parameter<int>("local_recovery_direction_samples", 24);
    dynamic_replan_delay_ = declare_parameter<double>("dynamic_replan_delay", 1.5);
    dynamic_replan_cooldown_ = declare_parameter<double>("dynamic_replan_cooldown", 8.0);
    dynamic_replan_obstacle_range_ =
      declare_parameter<double>("dynamic_replan_obstacle_range", 2.5);
    dynamic_replan_path_corridor_ =
      declare_parameter<double>("dynamic_replan_path_corridor", 0.45);
    dynamic_replan_static_filter_distance_ =
      declare_parameter<double>("dynamic_replan_static_filter_distance", 0.18);
    dynamic_replan_equivalence_distance_ =
      declare_parameter<double>("dynamic_replan_equivalence_distance", 0.16);
    dynamic_replan_rearm_clear_time_ =
      declare_parameter<double>("dynamic_replan_rearm_clear_time", 1.0);
    dynamic_replan_progress_distance_ =
      declare_parameter<double>("dynamic_replan_progress_distance", 0.10);
    dynamic_replan_goal_progress_ =
      declare_parameter<double>("dynamic_replan_goal_progress", 0.06);
    dynamic_replan_min_obstacle_points_ =
      declare_parameter<int>("dynamic_replan_min_obstacle_points", 4);

    ScanMatcherParameters matcher_parameters;
    matcher_parameters.search_translation =
      declare_parameter<double>("localization_search_xy", 0.20);
    matcher_parameters.search_rotation =
      declare_parameter<double>("localization_search_yaw", 0.16);
    matcher_parameters.coarse_translation_step =
      declare_parameter<double>("localization_step_xy", 0.05);
    matcher_parameters.coarse_rotation_step =
      declare_parameter<double>("localization_step_yaw", 0.04);
    matcher_parameters.fine_translation_step =
      declare_parameter<double>("localization_fine_step_xy", 0.01);
    matcher_parameters.fine_rotation_step =
      declare_parameter<double>("localization_fine_step_yaw", 0.01);
    matcher_parameters.likelihood_sigma =
      declare_parameter<double>("localization_likelihood_sigma", 0.10);
    matcher_parameters.minimum_score =
      declare_parameter<double>("localization_min_score", 0.32);
    matcher_parameters.minimum_valid_fraction =
      declare_parameter<double>("localization_min_valid_fraction", 0.45);
    matcher_parameters.prior_weight =
      declare_parameter<double>("localization_prior_weight", 0.025);
    matcher_parameters.correction_gain =
      declare_parameter<double>("localization_correction_gain", 0.35);
    matcher_parameters.minimum_points =
      declare_parameter<int>("localization_min_points", 25);
    matcher_parameters.maximum_points =
      declare_parameter<int>("localization_max_points", 160);

    map_.load(map_yaml, map_frame_);
    map_.buildFields(inflation_radius, likelihood_max_distance);
    planner_ = std::make_unique<AStarPlanner>(map_, planner_parameters);
    matcher_ = std::make_unique<CorrelativeScanMatcher>(map_, matcher_parameters);
    follower_ = std::make_unique<PathFollower>(
      controller_parameters, local_planner_parameters);

    const auto latched_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    map_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/map", latched_qos);
    costmap_publisher_ =
      create_publisher<nav_msgs::msg::OccupancyGrid>("/global_costmap/costmap", latched_qos);
    plan_publisher_ = create_publisher<nav_msgs::msg::Path>("/plan", rclcpp::QoS(1).reliable());
    command_publisher_ =
      create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", rclcpp::QoS(5).reliable());
    feedback_publisher_ = create_publisher<std_msgs::msg::Float32>(
      "/lite_navigation/feedback", 5);
    status_publisher_ = create_publisher<std_msgs::msg::String>(
      "/lite_navigation/status", latched_qos);
    pose_publisher_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/amcl_pose", 5);
    localization_score_publisher_ = create_publisher<std_msgs::msg::Float32>(
      "/lite_localization/score", 5);

    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LiteNavigationNode::odometryCallback, this, std::placeholders::_1));
    scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LiteNavigationNode::scanCallback, this, std::placeholders::_1));
    goal_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 5,
      std::bind(&LiteNavigationNode::goalCallback, this, std::placeholders::_1));
    initial_pose_subscription_ =
      create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", 5,
      std::bind(&LiteNavigationNode::initialPoseCallback, this, std::placeholders::_1));
    cancel_subscription_ = create_subscription<std_msgs::msg::Empty>(
      "/lite_navigation/cancel", 5,
      std::bind(&LiteNavigationNode::cancelCallback, this, std::placeholders::_1));

    control_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(1.0, control_frequency_)),
      std::bind(&LiteNavigationNode::controlTick, this));

    publishMaps();
    setStatus("IDLE");
    RCLCPP_INFO(
      get_logger(),
      "模块化轻量导航已启动：地图 %ux%u @ %.3f m，输入 %s + %s，底盘坐标系 %s",
      map_.occupancyGrid().info.width, map_.occupancyGrid().info.height,
      map_.occupancyGrid().info.resolution, odom_topic_.c_str(), scan_topic_.c_str(),
      base_frame_.c_str());
  }

  ~LiteNavigationNode() override
  {
    publishStop();
  }

private:
  struct TimedPose
  {
    rclcpp::Time stamp;
    Pose2D pose;
  };

  void publishMaps()
  {
    const auto stamp = now();
    auto occupancy_grid = map_.occupancyGrid();
    occupancy_grid.header.stamp = stamp;
    occupancy_grid.info.map_load_time = stamp;
    auto inflated_grid = map_.inflatedGrid();
    inflated_grid.header.stamp = stamp;
    inflated_grid.info.map_load_time = stamp;
    map_publisher_->publish(occupancy_grid);
    costmap_publisher_->publish(inflated_grid);
  }

  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    odometry_pose_ = {
      message->pose.pose.position.x,
      message->pose.pose.position.y,
      yawFromQuaternion(message->pose.pose.orientation)};
    have_odometry_ = true;

    rclcpp::Time stamp(message->header.stamp);
    if (stamp.nanoseconds() == 0) stamp = now();
    if (!odometry_history_.empty() && stamp < odometry_history_.back().stamp) {
      odometry_history_.clear();
    }
    odometry_history_.push_back({stamp, odometry_pose_});
    while (odometry_history_.size() > 200) odometry_history_.pop_front();

    if (pending_initial_pose_) {
      applyInitialPose(pending_pose_);
      pending_initial_pose_ = false;
    }
  }

  bool odometryPoseAt(const rclcpp::Time & stamp, Pose2D & pose) const
  {
    if (odometry_history_.empty()) return false;
    if (stamp.nanoseconds() == 0 || stamp <= odometry_history_.front().stamp) {
      pose = odometry_history_.front().pose;
      return true;
    }
    for (size_t i = 1; i < odometry_history_.size(); ++i) {
      if (stamp <= odometry_history_[i].stamp) {
        const double interval =
          (odometry_history_[i].stamp - odometry_history_[i - 1].stamp).seconds();
        if (interval <= 1e-6) {
          pose = odometry_history_[i].pose;
        } else {
          const double ratio =
            (stamp - odometry_history_[i - 1].stamp).seconds() / interval;
          pose = interpolate(
            odometry_history_[i - 1].pose, odometry_history_[i].pose, ratio);
        }
        return true;
      }
    }
    pose = odometry_history_.back().pose;
    return true;
  }

  void initialPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message)
  {
    if (!message->header.frame_id.empty() && message->header.frame_id != map_frame_) {
      RCLCPP_WARN(
        get_logger(), "初始位姿坐标系必须是 %s，收到 %s",
        map_frame_.c_str(), message->header.frame_id.c_str());
      return;
    }
    const Pose2D requested{
      message->pose.pose.position.x,
      message->pose.pose.position.y,
      yawFromQuaternion(message->pose.pose.orientation)};
    if (!have_odometry_) {
      pending_pose_ = requested;
      pending_initial_pose_ = true;
      RCLCPP_INFO(get_logger(), "初始位姿已缓存，等待里程计");
      return;
    }
    applyInitialPose(requested);
  }

  void applyInitialPose(const Pose2D & requested)
  {
    map_to_odom_ = compose(requested, inverse(odometry_pose_));
    localized_ = true;
    position_variance_ = 0.04;
    yaw_variance_ = 0.03;
    follower_->clear();
    resetDynamicReplanState();
    publishStop();
    setStatus("LOCALIZED");
    RCLCPP_INFO(
      get_logger(), "初始位置已设置：(%.3f, %.3f, %.3f)",
      requested.x, requested.y, requested.yaw);
  }

  void cancelCallback(const std_msgs::msg::Empty::SharedPtr)
  {
    follower_->clear();
    resetDynamicReplanState();
    publishStop();
    publishPlan({});
    setStatus(localized_ ? "LOCALIZED" : "IDLE");
    RCLCPP_INFO(get_logger(), "已停止当前导航目标，定位节点继续运行");
  }

  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr message)
  {
    if (!message->header.frame_id.empty() && message->header.frame_id != map_frame_) {
      RCLCPP_WARN(
        get_logger(), "目标坐标系必须是 %s，收到 %s",
        map_frame_.c_str(), message->header.frame_id.c_str());
      return;
    }
    if (!have_odometry_) {
      RCLCPP_WARN(get_logger(), "尚未收到融合里程计，忽略目标点");
      return;
    }
    const Pose2D requested_goal{
      message->pose.position.x,
      message->pose.position.y,
      yawFromQuaternion(message->pose.orientation)};
    const Pose2D current = compose(map_to_odom_, odometry_pose_);
    setStatus("PLANNING");
    PlanResult result = planner_->plan(current, requested_goal);
    if (!result.success) {
      follower_->clear();
      publishStop();
      setStatus("PLAN_FAILED");
      RCLCPP_WARN(get_logger(), "%s", result.message.c_str());
      return;
    }

    follower_->setPath(std::move(result.path), result.adjusted_goal);
    resetDynamicReplanState();
    publishPlan(follower_->path());
    setStatus("TRACKING");
    RCLCPP_INFO(
      get_logger(), "接收目标：(%.3f, %.3f, %.3f)，路径点 %zu",
      result.adjusted_goal.x, result.adjusted_goal.y, result.adjusted_goal.yaw,
      follower_->path().size());
  }

  bool scanPointsInBase(
    const sensor_msgs::msg::LaserScan & scan,
    std::vector<Point2D> & obstacle_points,
    std::vector<Point2D> & localization_points)
  {
    Pose2D base_from_laser;
    if (scan.header.frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "LaserScan 缺少 frame_id，无法定位");
      return false;
    }
    if (scan.header.frame_id != base_frame_) {
      try {
        const auto transform = tf_buffer_.lookupTransform(
          base_frame_, scan.header.frame_id, tf2::TimePointZero,
          tf2::durationFromSec(std::max(0.0, tf_lookup_timeout_)));
        base_from_laser = {
          transform.transform.translation.x,
          transform.transform.translation.y,
          yawFromQuaternion(transform.transform.rotation)};
      } catch (const tf2::TransformException & error) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "等待激光坐标系 %s 到 %s 的 TF：%s",
          scan.header.frame_id.c_str(), base_frame_.c_str(), error.what());
        return false;
      }
    }

    obstacle_points.clear();
    localization_points.clear();
    obstacle_points.reserve(scan.ranges.size());
    localization_points.reserve(scan.ranges.size());
    const double maximum_range = std::min(
      laser_max_range_, static_cast<double>(scan.range_max));
    const double c = std::cos(base_from_laser.yaw);
    const double s = std::sin(base_from_laser.yaw);
    for (size_t i = 0; i < scan.ranges.size(); ++i) {
      const double range = scan.ranges[i];
      if (!std::isfinite(range) || range < scan.range_min || range > maximum_range) continue;
      if (range >= static_cast<double>(scan.range_max) - 1e-3) continue;
      const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
      const double laser_x = range * std::cos(angle);
      const double laser_y = range * std::sin(angle);
      const Point2D point{
        base_from_laser.x + c * laser_x - s * laser_y,
        base_from_laser.y + s * laser_x + c * laser_y};
      obstacle_points.push_back(point);
      if (range >= laser_min_range_) localization_points.push_back(point);
    }
    return !obstacle_points.empty();
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr message)
  {
    std::vector<Point2D> obstacle_points;
    std::vector<Point2D> localization_points;
    if (!scanPointsInBase(*message, obstacle_points, localization_points)) return;
    latest_obstacle_points_ = std::move(obstacle_points);
    latest_localization_points_ = std::move(localization_points);
    last_scan_receive_time_ = now();
    have_scan_ = true;
    if (!have_odometry_) return;

    const rclcpp::Time current_time = now();
    if ((current_time - last_localization_time_).seconds() <
      1.0 / std::max(1.0, localization_frequency_))
    {
      return;
    }
    last_localization_time_ = current_time;

    Pose2D scan_odometry_pose;
    if (!odometryPoseAt(rclcpp::Time(message->header.stamp), scan_odometry_pose)) return;
    const Pose2D predicted = compose(map_to_odom_, scan_odometry_pose);
    const ScanMatchResult result = matcher_->match(predicted, latest_localization_points_);

    std_msgs::msg::Float32 score_message;
    score_message.data = static_cast<float>(result.score);
    localization_score_publisher_->publish(score_message);
    if (!result.accepted) {
      position_variance_ = std::min(0.50, position_variance_ * 1.02 + 0.001);
      yaw_variance_ = std::min(0.25, yaw_variance_ * 1.02 + 0.0005);
      return;
    }

    map_to_odom_ = compose(result.corrected_pose, inverse(scan_odometry_pose));
    localized_ = true;
    position_variance_ = result.position_variance;
    yaw_variance_ = result.yaw_variance;
    RCLCPP_DEBUG(
      get_logger(), "定位匹配：score=%.3f coverage=%.2f points=%zu",
      result.score, result.valid_fraction, result.points_used);
  }

  void controlTick()
  {
    broadcastTransform();
    publishLocalizedPose();
    if (!have_odometry_ || !follower_->active()) return;

    const Pose2D current = compose(map_to_odom_, odometry_pose_);
    const bool scan_is_fresh = have_scan_ &&
      (now() - last_scan_receive_time_).seconds() <= scan_timeout_;
    const ControlResult result = follower_->update(
      current, latest_obstacle_points_, scan_is_fresh,
      1.0 / std::max(1.0, control_frequency_));

    std::string status = result.status;
    const rclcpp::Time tick_time = now();
    // Local avoidance is normal for an omnidirectional base.  While the robot
    // is making measurable translational or goal progress, restart the stall
    // timer instead of replacing the global path.
    const bool avoidance_active =
      result.local_planner_active || result.local_planner_blocked;
    if (avoidance_active) {
      replan_clear_since_ = rclcpp::Time(0, 0, tick_time.get_clock_type());
      if (!tracking_avoidance_progress_) {
        tracking_avoidance_progress_ = true;
        avoidance_progress_pose_ = current;
        avoidance_progress_goal_distance_ = result.goal_distance;
        local_avoidance_since_ = tick_time;
      } else {
        const double movement = std::hypot(
          current.x - avoidance_progress_pose_.x,
          current.y - avoidance_progress_pose_.y);
        const double goal_progress =
          avoidance_progress_goal_distance_ - result.goal_distance;
        if (movement >= dynamic_replan_progress_distance_ ||
          goal_progress >= dynamic_replan_goal_progress_)
        {
          avoidance_progress_pose_ = current;
          avoidance_progress_goal_distance_ = result.goal_distance;
          local_avoidance_since_ = tick_time;
        }
      }
      const bool delay_elapsed =
        (tick_time - local_avoidance_since_).seconds() >= dynamic_replan_delay_;
      const bool cooldown_elapsed = last_dynamic_replan_.nanoseconds() == 0 ||
        (tick_time - last_dynamic_replan_).seconds() >= dynamic_replan_cooldown_;
      if (replan_armed_ && delay_elapsed && cooldown_elapsed) {
        last_dynamic_replan_ = tick_time;
        if (replanAroundCurrentObstacles(current)) {
          status = "DYNAMIC_REPLAN";
          replan_armed_ = false;
          tracking_avoidance_progress_ = false;
          local_avoidance_since_ = rclcpp::Time(0, 0, tick_time.get_clock_type());
        }
      }
    } else {
      local_avoidance_since_ = rclcpp::Time(0, 0, tick_time.get_clock_type());
      tracking_avoidance_progress_ = false;
      if (!replan_armed_) {
        if (replan_clear_since_.nanoseconds() == 0) replan_clear_since_ = tick_time;
        if ((tick_time - replan_clear_since_).seconds() >=
          dynamic_replan_rearm_clear_time_)
        {
          replan_armed_ = true;
          replan_clear_since_ = rclcpp::Time(0, 0, tick_time.get_clock_type());
        }
      }
    }

    std_msgs::msg::Float32 feedback;
    feedback.data = static_cast<float>(result.goal_distance);
    feedback_publisher_->publish(feedback);
    command_publisher_->publish(toTwist(result.velocity));
    setStatus(status);
    if (result.arrived) {
      feedback.data = 0.0F;
      feedback_publisher_->publish(feedback);
    }
  }

  bool replanAroundCurrentObstacles(const Pose2D & current)
  {
    const std::vector<Pose2D> & current_path = follower_->path();
    if (current_path.empty()) return false;
    size_t remaining_start = 0;
    double nearest_path_distance = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < current_path.size(); ++i) {
      const double distance = std::hypot(
        current_path[i].x - current.x, current_path[i].y - current.y);
      if (distance < nearest_path_distance) {
        nearest_path_distance = distance;
        remaining_start = i;
      }
    }

    const auto point_to_segment_distance = [](
        const Point2D & point, const Pose2D & from, const Pose2D & to)
      {
        const double dx = to.x - from.x;
        const double dy = to.y - from.y;
        const double length_squared = dx * dx + dy * dy;
        const double ratio = length_squared <= 1e-9 ? 0.0 : clamp(
          ((point.x - from.x) * dx + (point.y - from.y) * dy) /
          length_squared, 0.0, 1.0);
        return std::hypot(
          point.x - (from.x + ratio * dx),
          point.y - (from.y + ratio * dy));
      };

    std::vector<Point2D> map_obstacles;
    map_obstacles.reserve(latest_obstacle_points_.size());
    const double c = std::cos(current.yaw);
    const double s = std::sin(current.yaw);
    for (const Point2D & point : latest_obstacle_points_) {
      if (std::hypot(point.x, point.y) > dynamic_replan_obstacle_range_) continue;
      const Point2D map_point{
        current.x + c * point.x - s * point.y,
        current.y + s * point.x + c * point.y};

      // Static walls already exist in the map.  Adding their scan returns as
      // temporary obstacles inflates them twice and creates large zig-zag
      // detours, especially with small localization errors.
      double static_clearance = 0.0;
      if (!map_.distanceAtWorld(map_point.x, map_point.y, static_clearance) ||
        static_clearance <= dynamic_replan_static_filter_distance_)
      {
        continue;
      }

      double path_distance = std::hypot(
        map_point.x - current_path[remaining_start].x,
        map_point.y - current_path[remaining_start].y);
      for (size_t i = remaining_start + 1;
        i < current_path.size() && path_distance > dynamic_replan_path_corridor_;
        ++i)
      {
        path_distance = std::min(
          path_distance,
          point_to_segment_distance(map_point, current_path[i - 1], current_path[i]));
      }
      if (path_distance > dynamic_replan_path_corridor_) continue;
      map_obstacles.push_back(map_point);
    }
    if (map_obstacles.size() <
      static_cast<size_t>(std::max(1, dynamic_replan_min_obstacle_points_)))
    {
      RCLCPP_DEBUG(
        get_logger(), "动态重规划忽略稀疏或路径外障碍点：%zu", map_obstacles.size());
      return false;
    }

    PlanResult replanned = planner_->plan(current, follower_->goal(), map_obstacles);
    if (!replanned.success) {
      RCLCPP_WARN(get_logger(), "动态障碍重规划失败：%s", replanned.message.c_str());
      return false;
    }
    if (pathsEquivalent(
        current_path, replanned.path, remaining_start,
        dynamic_replan_equivalence_distance_))
    {
      RCLCPP_DEBUG(get_logger(), "动态重规划结果与当前路径等价，保持当前路径");
      return false;
    }
    follower_->setPath(std::move(replanned.path), replanned.adjusted_goal);
    publishPlan(follower_->path());
    RCLCPP_INFO(
      get_logger(), "动态障碍触发重规划，新路径点 %zu", follower_->path().size());
    return true;
  }

  void resetDynamicReplanState()
  {
    const auto clock_type = get_clock()->get_clock_type();
    local_avoidance_since_ = rclcpp::Time(0, 0, clock_type);
    last_dynamic_replan_ = rclcpp::Time(0, 0, clock_type);
    replan_clear_since_ = rclcpp::Time(0, 0, clock_type);
    tracking_avoidance_progress_ = false;
    replan_armed_ = true;
  }

  static bool pathsEquivalent(
    const std::vector<Pose2D> & first,
    const std::vector<Pose2D> & second,
    size_t first_start,
    double maximum_mean_distance)
  {
    if (first.empty() || second.empty()) return false;
    const auto mean_nearest_distance = [](
        const std::vector<Pose2D> & source,
        size_t source_start,
        const std::vector<Pose2D> & target,
        size_t target_start)
      {
        double distance_sum = 0.0;
        size_t samples = 0;
        const size_t source_count = source.size() - source_start;
        const size_t stride = std::max<size_t>(1, source_count / 30);
        for (size_t i = source_start; i < source.size(); i += stride) {
          double nearest = std::numeric_limits<double>::infinity();
          for (size_t j = target_start; j < target.size(); ++j) {
            const Pose2D & candidate = target[j];
            nearest = std::min(
              nearest,
              std::hypot(source[i].x - candidate.x, source[i].y - candidate.y));
          }
          distance_sum += nearest;
          ++samples;
        }
        return samples == 0 ? std::numeric_limits<double>::infinity() :
               distance_sum / samples;
      };
    first_start = std::min(first_start, first.size() - 1);
    const double threshold = std::max(0.01, maximum_mean_distance);
    return mean_nearest_distance(first, first_start, second, 0) < threshold &&
           mean_nearest_distance(second, 0, first, first_start) < threshold;
  }

  void publishPlan(const std::vector<Pose2D> & path)
  {
    nav_msgs::msg::Path message;
    message.header.frame_id = map_frame_;
    message.header.stamp = now();
    message.poses.reserve(path.size());
    for (size_t i = 0; i < path.size(); ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = message.header;
      pose.pose.position.x = path[i].x;
      pose.pose.position.y = path[i].y;
      double yaw = path[i].yaw;
      if (i + 1 < path.size()) {
        yaw = std::atan2(path[i + 1].y - path[i].y, path[i + 1].x - path[i].x);
      }
      pose.pose.orientation = quaternionFromYaw(yaw);
      message.poses.push_back(pose);
    }
    plan_publisher_->publish(message);
  }

  void broadcastTransform()
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = now();
    transform.header.frame_id = map_frame_;
    transform.child_frame_id = odom_frame_;
    transform.transform.translation.x = map_to_odom_.x;
    transform.transform.translation.y = map_to_odom_.y;
    transform.transform.rotation = quaternionFromYaw(map_to_odom_.yaw);
    tf_broadcaster_->sendTransform(transform);
  }

  void publishLocalizedPose()
  {
    if (!have_odometry_) return;
    const Pose2D pose = compose(map_to_odom_, odometry_pose_);
    geometry_msgs::msg::PoseWithCovarianceStamped message;
    message.header.stamp = now();
    message.header.frame_id = map_frame_;
    message.pose.pose.position.x = pose.x;
    message.pose.pose.position.y = pose.y;
    message.pose.pose.orientation = quaternionFromYaw(pose.yaw);
    message.pose.covariance[0] = localized_ ? position_variance_ : 0.25;
    message.pose.covariance[7] = localized_ ? position_variance_ : 0.25;
    message.pose.covariance[35] = localized_ ? yaw_variance_ : 0.10;
    pose_publisher_->publish(message);
  }

  void publishStop()
  {
    if (command_publisher_) command_publisher_->publish(geometry_msgs::msg::Twist());
  }

  void setStatus(const std::string & status)
  {
    if (status == last_status_) return;
    last_status_ = status;
    if (!status_publisher_) return;
    std_msgs::msg::String message;
    message.data = status;
    status_publisher_->publish(message);
  }

  std::string odom_topic_;
  std::string scan_topic_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  double control_frequency_{12.0};
  double localization_frequency_{4.0};
  double scan_timeout_{0.8};
  double laser_min_range_{0.10};
  double laser_max_range_{8.0};
  double tf_lookup_timeout_{0.05};
  double dynamic_replan_delay_{1.5};
  double dynamic_replan_cooldown_{8.0};
  double dynamic_replan_obstacle_range_{2.5};
  double dynamic_replan_path_corridor_{0.45};
  double dynamic_replan_static_filter_distance_{0.18};
  double dynamic_replan_equivalence_distance_{0.16};
  double dynamic_replan_rearm_clear_time_{1.0};
  double dynamic_replan_progress_distance_{0.10};
  double dynamic_replan_goal_progress_{0.06};
  int dynamic_replan_min_obstacle_points_{4};

  GridMap map_;
  std::unique_ptr<AStarPlanner> planner_;
  std::unique_ptr<CorrelativeScanMatcher> matcher_;
  std::unique_ptr<PathFollower> follower_;

  Pose2D odometry_pose_;
  Pose2D map_to_odom_;
  Pose2D pending_pose_;
  std::deque<TimedPose> odometry_history_;
  std::vector<Point2D> latest_obstacle_points_;
  std::vector<Point2D> latest_localization_points_;
  bool have_odometry_{false};
  bool have_scan_{false};
  bool localized_{false};
  bool pending_initial_pose_{false};
  double position_variance_{0.25};
  double yaw_variance_{0.10};
  rclcpp::Time last_scan_receive_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_localization_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time local_avoidance_since_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_dynamic_replan_{0, 0, RCL_ROS_TIME};
  rclcpp::Time replan_clear_since_{0, 0, RCL_ROS_TIME};
  Pose2D avoidance_progress_pose_;
  double avoidance_progress_goal_distance_{0.0};
  bool tracking_avoidance_progress_{false};
  bool replan_armed_{true};
  std::string last_status_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr feedback_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr localization_score_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initial_pose_subscription_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr cancel_subscription_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace chuyun_navigation2

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<chuyun_navigation2::LiteNavigationNode>());
  } catch (const std::exception & error) {
    std::cerr << "lite_navigation 启动失败: " << error.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
