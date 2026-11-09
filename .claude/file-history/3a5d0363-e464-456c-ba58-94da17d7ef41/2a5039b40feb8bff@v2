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
#include "amr_dispatcher_ros/proto_dual_publisher.hpp"

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
  // 阶段 4b：原先这里还有 error_pub_（节点自建的 rosidl 发布器）。
  // 已删除并收进 error_proto_pub_——留着它就会有两个 rosidl 发布器指向
  // /tracking/error，而「只发 proto」的开关也就随之失效。
  //
  // 阶段 2 双发：/tracking/error_proto，rosidl 侧同样由双发器持有。
  //
  // 这是全项目**频率最高**的双发点（control_rate_hz，默认 20Hz）。
  // 多一份 protobuf 序列化的 CPU/带宽成本，就是这个开关最该被看见的理由：
  // 现场若确认没有 proto 消费者，关掉 proto_dual_publish 即可回到原状。
  std::unique_ptr<ProtoDualPublisher<
      amr_dispatcher_interfaces::msg::TrackingError,
      amr_dispatcher::proto::TrackingError>> error_proto_pub_;
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

  // 阶段 2：proto 双发总开关（参数 proto_dual_publish）
  bool proto_dual_publish_{true};
  // 阶段 4b：rosidl 侧开关（参数 proto_rosidl_publish，默认开）。
  // false = 只发 /tracking/error_proto，DDS 里不再有 /tracking/error。
  bool proto_rosidl_publish_{true};
};

}  // namespace amr_dispatcher_ros
