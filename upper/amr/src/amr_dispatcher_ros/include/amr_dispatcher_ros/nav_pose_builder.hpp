#pragma once

#include <cmath>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>

namespace amr_dispatcher_ros {

// NavigateSequence 航点位姿构建（B3）：
// 从 HandleAccepted 中抽出的纯逻辑，可脱离 rclcpp 单测。
// 规则：
//   - 单航点：保留客户端给的原始四元数（不得用 atan2 覆盖）；
//   - 多航点：按相邻点 atan2 推航向并重写 orientation（与原实现一致）。
inline std::vector<geometry_msgs::msg::PoseStamped> BuildNavPoses(
    const std::vector<geometry_msgs::msg::PoseStamped>& waypoints,
    const std::string& frame_id = "map") {
  std::vector<geometry_msgs::msg::PoseStamped> out;
  const std::size_t n = waypoints.size();
  out.reserve(n);
  for (std::size_t i = 0; i < n; ++i) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = frame_id;
    ps.pose = waypoints[i].pose;
    if (n > 1) {
      // 末点沿用倒数第二段朝向（与原实现语义一致）
      const auto& a = (i + 1 < n) ? waypoints[i] : waypoints[i - 1];
      const auto& b = (i + 1 < n) ? waypoints[i + 1] : waypoints[i];
      const double yaw = std::atan2(b.pose.position.y - a.pose.position.y,
                                    b.pose.position.x - a.pose.position.x);
      ps.pose.orientation.x = 0.0;
      ps.pose.orientation.y = 0.0;
      ps.pose.orientation.z = std::sin(yaw / 2.0);
      ps.pose.orientation.w = std::cos(yaw / 2.0);
    }
    out.push_back(ps);
  }
  return out;
}

}  // namespace amr_dispatcher_ros
