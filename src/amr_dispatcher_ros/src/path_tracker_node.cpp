#include "amr_dispatcher_ros/path_tracker_node.hpp"

#include <chrono>
#include <cmath>

#include "amr_dispatcher_core/path_tracking/geometry_utils.hpp"
#include "amr_dispatcher_ros/vehicle_topic_names.hpp"

using namespace std::chrono_literals;

namespace amr_dispatcher_ros {

PathTrackerNode::PathTrackerNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("path_tracker_node", options) {
  declare_parameter("controller_type", "pure_pursuit");
  declare_parameter("lookahead_distance", 0.6);
  declare_parameter("target_speed", 0.5);
  declare_parameter("stanley_k", 0.5);
  declare_parameter("control_rate_hz", 20.0);
  // T4：终点判定容差（m）。此前写死在控制器默认值 0.10，台架标定偏几十厘米时
  // 车永远进不了到达窗口 -> 不 arrived -> 任务挂着 -> 10s 后被死锁检测判
  // kIdleActive。默认值保持 0.10（与改动前逐位一致），仅在本参数被覆盖时生效。
  declare_parameter("goal_tolerance", 0.10);
  // ---- P0.6 / A4：per-vehicle 话题隔离 ----
  // vehicle_id 空 = 单车兼容模式（话题名逐字不变）；非空 = 按 {vid} 派生。
  declare_parameter("vehicle_id", "");
  // 显式覆盖优先于派生；为空则按 vehicle_id 派生或回落到 legacy 全局名。
  declare_parameter("plan_topic", "");
  declare_parameter("odom_topic", "");
  declare_parameter("cmd_vel_topic", "");
  declare_parameter("tracking_error_topic", "");
  declare_parameter("heartbeat_topic", "");
  // odom 原点在 map 帧下的静态位姿。真机台架必须配置（车摆在哪个 map 坐标上上电），
  // 仿真保持 0 即可（PTY 模拟器直接给出 map 坐标）。
  declare_parameter("odom_origin_x", 0.0);
  declare_parameter("odom_origin_y", 0.0);
  declare_parameter("odom_origin_yaw", 0.0);
  // 阶段 2 双发总开关（默认开）。关闭只停 proto 侧，rosidl 侧不变。
  declare_parameter("proto_dual_publish", true);

  controller_type_ = get_parameter("controller_type").as_string();
  proto_dual_publish_ = get_parameter("proto_dual_publish").as_bool();
  odom_origin_x_ = get_parameter("odom_origin_x").as_double();
  odom_origin_y_ = get_parameter("odom_origin_y").as_double();
  odom_origin_yaw_ = get_parameter("odom_origin_yaw").as_double();

  amr_dispatcher_core::path_tracking::PurePursuitController::Config pp_cfg;
  pp_cfg.lookahead_distance = get_parameter("lookahead_distance").as_double();
  pp_cfg.target_speed = get_parameter("target_speed").as_double();
  pp_cfg.goal_tolerance = get_parameter("goal_tolerance").as_double();
  pp_controller_ = std::make_unique<amr_dispatcher_core::path_tracking::PurePursuitController>(pp_cfg);

  amr_dispatcher_core::path_tracking::StanleyController::Config st_cfg;
  st_cfg.gain = get_parameter("stanley_k").as_double();
  st_cfg.target_speed = get_parameter("target_speed").as_double();
  st_cfg.goal_tolerance = get_parameter("goal_tolerance").as_double();
  stanley_controller_ = std::make_unique<amr_dispatcher_core::path_tracking::StanleyController>(st_cfg);

  // ---- P0.6 / A4：解析最终话题名（显式覆盖 > per-vehicle 派生 > legacy 全局名）----
  vehicle_id_ = get_parameter("vehicle_id").as_string();
  if (!vehicle_id_.empty() && !IsValidVehicleId(vehicle_id_)) {
    RCLCPP_ERROR(get_logger(),
                 "Invalid vehicle_id '%s' (must match [A-Za-z_][A-Za-z0-9_]*); "
                 "falling back to legacy global topics",
                 vehicle_id_.c_str());
    vehicle_id_.clear();
  }
  plan_topic_ = ResolveVehicleTopic(get_parameter("plan_topic").as_string(), "/plan", vehicle_id_);
  odom_topic_ = ResolveVehicleTopic(get_parameter("odom_topic").as_string(), "/odom", vehicle_id_);
  cmd_vel_topic_ =
      ResolveVehicleTopic(get_parameter("cmd_vel_topic").as_string(), "/cmd_vel_raw", vehicle_id_);
  tracking_error_topic_ = ResolveVehicleTopic(get_parameter("tracking_error_topic").as_string(),
                                             "/tracking/error", vehicle_id_);
  heartbeat_topic_ = ResolveVehicleTopic(get_parameter("heartbeat_topic").as_string(),
                                         "/safety/heartbeat/path_tracker", vehicle_id_);

  path_sub_ = create_subscription<nav_msgs::msg::Path>(
      plan_topic_, 1,
      std::bind(&PathTrackerNode::PathCallback, this, std::placeholders::_1));

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 10,
      std::bind(&PathTrackerNode::OdomCallback, this, std::placeholders::_1));

  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
  error_pub_ = create_publisher<amr_dispatcher_interfaces::msg::TrackingError>(
      tracking_error_topic_, 10);
  // 阶段 2 双发：/tracking/error_proto，20Hz。QoS 深度与 rosidl 侧一致（10）。
  error_proto_pub_ = std::make_unique<ProtoDualPublisher<
      amr_dispatcher_interfaces::msg::TrackingError,
      amr_dispatcher::proto::TrackingError>>(
      this, tracking_error_topic_, 10, proto_dual_publish_);
  if (error_proto_pub_->enabled()) {
    RCLCPP_INFO(get_logger(), "proto dual-publish enabled: %s",
                error_proto_pub_->proto_topic().c_str());
  }
  // B1：心跳分源，只喂 path_tracker 自己的看门狗通道
  heartbeat_pub_ = create_publisher<std_msgs::msg::Bool>(heartbeat_topic_, 10);

  const double rate_hz = get_parameter("control_rate_hz").as_double();
  timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_hz),
      std::bind(&PathTrackerNode::ControlLoop, this));

  RCLCPP_INFO(get_logger(), "PathTrackerNode initialized with controller: %s", controller_type_.c_str());
  // P0.6：打印最终话题名——多车现场核对接线的唯一依据（每个 vehicle 一组）。
  RCLCPP_INFO(get_logger(),
              "PathTrackerNode topics (vehicle_id=%s): plan=%s odom=%s cmd_vel_raw=%s "
              "tracking_error=%s heartbeat=%s",
              vehicle_id_.empty() ? "<legacy>" : vehicle_id_.c_str(), plan_topic_.c_str(),
              odom_topic_.c_str(), cmd_vel_topic_.c_str(), tracking_error_topic_.c_str(),
              heartbeat_topic_.c_str());
}

PathTrackerNode::~PathTrackerNode() = default;

void PathTrackerNode::PathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
  // /plan 由 dispatcher 在 map 帧发布；本节点没有 tf 依赖，若对端换成了别的帧
  // （例如未配置 odom_origin_* 却收到 odom 帧的路径），这里只告警不猜测转换，
  // 避免静默算错方向。空 frame_id 视为未约定，与旧行为一致。
  if (!msg->header.frame_id.empty() && msg->header.frame_id != "map" &&
      !warned_frame_mismatch_) {
    warned_frame_mismatch_ = true;
    RCLCPP_WARN(get_logger(),
                "Path frame_id is '%s' but positions are applied on the map frame "
                "(odom_origin_x=%.3f y=%.3f yaw=%.3f). Check odom_origin_* parameters.",
                msg->header.frame_id.c_str(), odom_origin_x_, odom_origin_y_,
                odom_origin_yaw_);
  }
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
  // 把 odom 帧下的位姿搬进 map 帧：先按 odom_origin_yaw 旋转，再平移。
  // 偏移全 0 时结果与旧实现逐位相同（单车仿真兼容）。
  const double raw_x = msg->pose.pose.position.x;
  const double raw_y = msg->pose.pose.position.y;
  const double raw_yaw = amr_dispatcher_core::path_tracking::YawFromQuaternion(
      msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  const double c = std::cos(odom_origin_yaw_);
  const double s = std::sin(odom_origin_yaw_);
  current_pose_.x = odom_origin_x_ + raw_x * c - raw_y * s;
  current_pose_.y = odom_origin_y_ + raw_x * s + raw_y * c;
  current_pose_.yaw = std::atan2(std::sin(raw_yaw + odom_origin_yaw_),
                                 std::cos(raw_yaw + odom_origin_yaw_));
  current_linear_speed_ = msg->twist.twist.linear.x;
  has_odom_ = true;
}

void PathTrackerNode::ControlLoop() {
  // B1：心跳必须无条件先发——idle 早退若挡在心跳前，安全门会误判本节点失联。
  if (heartbeat_pub_) {
    std_msgs::msg::Bool hb;
    hb.data = true;
    heartbeat_pub_->publish(hb);
  }

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
  // 阶段 2 双发：从同一个 err_msg 投影，保证两路逐字段一致。
  if (error_proto_pub_) {
    error_proto_pub_->Publish(err_msg);
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
