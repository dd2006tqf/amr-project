#include "amr_dispatcher_core/path_tracking/stanley.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "amr_dispatcher_core/path_tracking/path_geometry.hpp"

namespace amr_dispatcher_core::path_tracking {

StanleyController::StanleyController(Config config) : config_(config) {
  config_.goal_tolerance = std::max(0.0, config_.goal_tolerance);
  config_.softening = std::max(0.0, config_.softening);
}

void StanleyController::SetPath(std::vector<Pose2D> path) { path_ = std::move(path); }

int StanleyController::NearestIndex(const Pose2D& robot_pose) const {
  int nearest_index = 0;
  double nearest_distance = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i < path_.size(); ++i) {
    const double dx = path_[i].x - robot_pose.x;
    const double dy = path_[i].y - robot_pose.y;
    const double distance = std::hypot(dx, dy);
    if (distance < nearest_distance) {
      nearest_distance = distance;
      nearest_index = static_cast<int>(i);
    }
  }
  return nearest_index;
}

StanleyResult StanleyController::Compute(const Pose2D& robot_pose) const {
  StanleyResult result;
  if (path_.size() < 2) {
    return result;
  }

  const int nearest_index = NearestIndex(robot_pose);
  result.nearest_index = nearest_index;

  const auto& nearest = path_[nearest_index];
  result.lateral_error =
      SignedLateralError(nearest.x, nearest.y, nearest.yaw, robot_pose.x, robot_pose.y);
  result.heading_error = NormalizeAngle(nearest.yaw - robot_pose.yaw);

  const auto& goal = path_.back();
  const double distance_to_goal = std::hypot(goal.x - robot_pose.x, robot_pose.y - robot_pose.y);
  if (IsFinalWaypointReached(
          nearest_index, path_.size(), distance_to_goal, config_.goal_tolerance)) {
    result.goal_reached = true;
    return result;
  }

  const double steer =
      result.heading_error +
      std::atan2(config_.gain * result.lateral_error, config_.target_speed + config_.softening);

  result.cmd.linear_x = config_.target_speed;
  result.cmd.angular_z = std::clamp(steer, -config_.max_angular_speed, config_.max_angular_speed);
  return result;
}

}  // namespace amr_dispatcher_core::path_tracking
