#include "amr_dispatcher_core/path_tracking/pure_pursuit.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "amr_dispatcher_core/path_tracking/path_geometry.hpp"

namespace amr_dispatcher_core::path_tracking {

PurePursuitController::PurePursuitController(Config config) : config_(config) {
  config_.lookahead_distance = std::max(0.05, config_.lookahead_distance);
  config_.goal_tolerance = std::max(0.0, config_.goal_tolerance);
}

void PurePursuitController::SetPath(std::vector<Pose2D> path) { path_ = std::move(path); }

int PurePursuitController::NearestIndex(const Pose2D& robot_pose) const {
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

std::size_t PurePursuitController::SelectTargetIndex(
    int nearest_index, const Pose2D& robot_pose) const {
  std::size_t first_forward_index = path_.size();
  for (std::size_t i = static_cast<std::size_t>(nearest_index); i < path_.size(); ++i) {
    const double dx = path_[i].x - robot_pose.x;
    const double dy = path_[i].y - robot_pose.y;
    if (ForwardProjection(robot_pose.yaw, dx, dy) <= 0.0) {
      continue;
    }
    if (first_forward_index == path_.size()) {
      first_forward_index = i;
    }
    if (std::hypot(dx, dy) >= config_.lookahead_distance) {
      return i;
    }
  }
  if (first_forward_index != path_.size()) {
    return first_forward_index;
  }
  return path_.size() - 1;
}

PurePursuitResult PurePursuitController::Compute(const Pose2D& robot_pose) const {
  PurePursuitResult result;
  if (path_.empty()) {
    return result;
  }

  const int nearest_index = NearestIndex(robot_pose);
  result.nearest_index = nearest_index;

  const auto& nearest = path_[nearest_index];
  result.lateral_error =
      SignedLateralError(nearest.x, nearest.y, nearest.yaw, robot_pose.x, robot_pose.y);

  const auto& goal = path_.back();
  const double distance_to_goal = std::hypot(goal.x - robot_pose.x, goal.y - robot_pose.y);
  if (IsFinalWaypointReached(
          nearest_index, path_.size(), distance_to_goal, config_.goal_tolerance)) {
    result.goal_reached = true;
    return result;  // 零速 + goal_reached
  }

  const std::size_t target_index = SelectTargetIndex(nearest_index, robot_pose);
  const auto& target = path_[target_index];
  const double dx = target.x - robot_pose.x;
  const double dy = target.y - robot_pose.y;
  result.heading_error = NormalizeAngle(std::atan2(dy, dx) - robot_pose.yaw);
  const double curvature = 2.0 * std::sin(result.heading_error) / config_.lookahead_distance;

  result.cmd.linear_x = config_.target_speed;
  result.cmd.angular_z =
      std::clamp(config_.target_speed * curvature, -config_.max_angular_speed, config_.max_angular_speed);
  return result;
}

}  // namespace amr_dispatcher_core::path_tracking
