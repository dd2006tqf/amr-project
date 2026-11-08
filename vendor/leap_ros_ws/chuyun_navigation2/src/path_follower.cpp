#include "chuyun_navigation2/path_follower.hpp"

#include <cmath>
#include <limits>
#include <utility>

namespace chuyun_navigation2
{

PathFollower::PathFollower(
  ControllerParameters parameters,
  LocalPlannerParameters local_planner_parameters)
: parameters_(parameters), local_planner_(local_planner_parameters)
{
  parameters_.max_vx = std::max(0.01, parameters_.max_vx);
  parameters_.max_vy = std::max(0.0, parameters_.max_vy);
  parameters_.max_reverse_speed = std::max(0.0, parameters_.max_reverse_speed);
  parameters_.heading_deadband = clamp(parameters_.heading_deadband, 0.0, kPi - 0.05);
  parameters_.rotate_before_translate_angle = clamp(
    parameters_.rotate_before_translate_angle, parameters_.heading_deadband, kPi - 0.01);
  parameters_.path_search_window = std::max(5, parameters_.path_search_window);
}

void PathFollower::setPath(std::vector<Pose2D> path, const Pose2D & goal)
{
  path_ = std::move(path);
  goal_ = goal;
  path_index_ = 0;
  previous_velocity_ = Velocity2D();
  local_planner_.reset();
  active_ = !path_.empty();
}

void PathFollower::clear()
{
  path_.clear();
  path_index_ = 0;
  previous_velocity_ = Velocity2D();
  local_planner_.reset();
  active_ = false;
}

void PathFollower::applyRateLimits(Velocity2D & velocity, double dt)
{
  dt = clamp(dt, 0.001, 0.5);
  velocity.x = previous_velocity_.x + clamp(
    velocity.x - previous_velocity_.x, -parameters_.max_ax * dt, parameters_.max_ax * dt);
  velocity.y = previous_velocity_.y + clamp(
    velocity.y - previous_velocity_.y, -parameters_.max_ay * dt, parameters_.max_ay * dt);
  velocity.yaw = previous_velocity_.yaw + clamp(
    velocity.yaw - previous_velocity_.yaw, -parameters_.max_aw * dt, parameters_.max_aw * dt);
  previous_velocity_ = velocity;
}

ControlResult PathFollower::update(
  const Pose2D & current_pose,
  const std::vector<Point2D> & obstacle_points,
  bool scan_is_fresh,
  double dt)
{
  ControlResult result;
  if (!active_ || path_.empty()) return result;

  result.goal_distance = std::hypot(goal_.x - current_pose.x, goal_.y - current_pose.y);
  if (result.goal_distance <= parameters_.goal_position_tolerance) {
    const double yaw_error = normalizeAngle(goal_.yaw - current_pose.yaw);
    if (std::abs(yaw_error) <= parameters_.goal_yaw_tolerance) {
      result.arrived = true;
      result.status = "ARRIVED";
      clear();
      return result;
    }
    if (!scan_is_fresh) {
      previous_velocity_ = Velocity2D();
      result.status = "SCAN_STALE_STOP";
      return result;
    }
    Velocity2D desired_velocity;
    desired_velocity.yaw = clamp(
      parameters_.angular_gain * yaw_error, -parameters_.max_wz, parameters_.max_wz);
    const double c = std::cos(current_pose.yaw);
    const double s = std::sin(current_pose.yaw);
    const Point2D target{
      c * (goal_.x - current_pose.x) + s * (goal_.y - current_pose.y),
      -s * (goal_.x - current_pose.x) + c * (goal_.y - current_pose.y)};
    const LocalTrajectoryResult local_result = local_planner_.plan(
      desired_velocity, target, obstacle_points, previous_velocity_);
    result.local_planner_active = local_result.avoiding;
    result.local_planner_blocked = local_result.blocked || !local_result.valid;
    if (result.local_planner_blocked) {
      previous_velocity_ = Velocity2D();
      result.status = "LOCAL_BLOCKED";
      return result;
    }
    result.velocity = local_result.velocity;
    applyRateLimits(result.velocity, dt);
    result.status = local_result.recovering ? "LOCAL_RECOVERY" :
      (local_result.avoiding ? "LOCAL_AVOIDING" : "TRACKING");
    return result;
  }

  size_t closest = path_index_;
  double closest_distance = std::numeric_limits<double>::infinity();
  // Do not search the entire remaining path.  On a folded route a geometrically
  // close, much later segment can otherwise be selected and make the command
  // alternate between two branches.
  const size_t search_end = std::min(
    path_.size(), path_index_ + static_cast<size_t>(parameters_.path_search_window) + 1);
  for (size_t i = path_index_; i < search_end; ++i) {
    const double distance = std::hypot(path_[i].x - current_pose.x, path_[i].y - current_pose.y);
    if (distance < closest_distance) {
      closest_distance = distance;
      closest = i;
    }
  }
  path_index_ = closest;
  size_t target_index = closest;
  // Use arc length instead of distance from the robot.  The selected target is
  // then stable on corners and while the local planner is moving sideways.
  double lookahead_arc = 0.0;
  while (target_index + 1 < path_.size() &&
    lookahead_arc < parameters_.lookahead_distance)
  {
    lookahead_arc += std::hypot(
      path_[target_index + 1].x - path_[target_index].x,
      path_[target_index + 1].y - path_[target_index].y);
    ++target_index;
  }

  const Pose2D & target = path_[target_index];
  const double dx = target.x - current_pose.x;
  const double dy = target.y - current_pose.y;
  const double desired_yaw = std::atan2(dy, dx);
  const double heading_error = normalizeAngle(desired_yaw - current_pose.yaw);
  const double c = std::cos(current_pose.yaw);
  const double s = std::sin(current_pose.yaw);
  const Point2D local_target{
    c * dx + s * dy,
    -s * dx + c * dy};
  const double requested_speed = std::min(
    parameters_.max_xy_speed, parameters_.linear_gain * std::hypot(dx, dy));
  const double target_distance = std::max(1e-6, std::hypot(local_target.x, local_target.y));
  // A mecanum/omnidirectional base does not need to face every path sample.
  // Keep translating in body X/Y, only slowing (never stopping) for a target
  // far behind the chassis.
  const double heading_magnitude = std::abs(heading_error);
  double translation_scale = 1.0;
  if (heading_magnitude > parameters_.rotate_before_translate_angle) {
    const double span = std::max(0.05, kPi - parameters_.rotate_before_translate_angle);
    translation_scale = clamp((kPi - heading_magnitude) / span, 0.25, 1.0);
  }
  const double velocity_scale = translation_scale * requested_speed / target_distance;
  result.velocity.x = clamp(
    local_target.x * velocity_scale,
    -parameters_.max_reverse_speed, parameters_.max_vx);
  result.velocity.y = clamp(
    local_target.y * velocity_scale, -parameters_.max_vy, parameters_.max_vy);
  const double xy_speed = std::hypot(result.velocity.x, result.velocity.y);
  if (xy_speed > parameters_.max_xy_speed) {
    result.velocity.x *= parameters_.max_xy_speed / xy_speed;
    result.velocity.y *= parameters_.max_xy_speed / xy_speed;
  }
  // A wide heading deadband lets lateral motion handle ordinary cross-track
  // error.  This removes continual left/right steering corrections.
  const double yaw_error = std::copysign(
    std::max(0.0, heading_magnitude - parameters_.heading_deadband), heading_error);
  result.velocity.yaw = clamp(
    parameters_.angular_gain * yaw_error, -parameters_.max_wz, parameters_.max_wz);

  if (!scan_is_fresh) {
    previous_velocity_ = Velocity2D();
    result.velocity = Velocity2D();
    result.status = "SCAN_STALE_STOP";
    return result;
  }
  const LocalTrajectoryResult local_result = local_planner_.plan(
    result.velocity, local_target, obstacle_points, previous_velocity_);
  result.local_planner_active = local_result.avoiding;
  result.local_planner_blocked = local_result.blocked || !local_result.valid;
  if (result.local_planner_blocked) {
    previous_velocity_ = Velocity2D();
    result.velocity = Velocity2D();
    result.status = "LOCAL_BLOCKED";
    return result;
  }
  result.velocity = local_result.velocity;
  applyRateLimits(result.velocity, dt);
  result.status = local_result.recovering ? "LOCAL_RECOVERY" :
    (local_result.avoiding ? "LOCAL_AVOIDING" : "TRACKING");
  return result;
}

}  // namespace chuyun_navigation2
