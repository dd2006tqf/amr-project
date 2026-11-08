#include "chuyun_navigation2/local_trajectory_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace chuyun_navigation2
{
namespace
{

double sampleValue(double minimum, double maximum, int sample, int sample_count)
{
  if (sample_count <= 1) return 0.5 * (minimum + maximum);
  return minimum + (maximum - minimum) * sample / (sample_count - 1);
}

double normalizedDifference(double first, double second, double scale)
{
  const double difference = (first - second) / std::max(0.01, scale);
  return difference * difference;
}

int turnSign(const Velocity2D & velocity)
{
  const double steering = std::abs(velocity.y) > 0.015 ? velocity.y : velocity.yaw;
  if (steering > 0.015) return 1;
  if (steering < -0.015) return -1;
  return 0;
}

}  // namespace

LocalTrajectoryPlanner::LocalTrajectoryPlanner(LocalPlannerParameters parameters)
: parameters_(parameters)
{
  parameters_.max_vx = std::max(0.01, parameters_.max_vx);
  parameters_.max_vy = std::max(0.0, parameters_.max_vy);
  parameters_.max_wz = std::max(0.01, parameters_.max_wz);
  parameters_.robot_radius = std::max(0.02, parameters_.robot_radius);
  parameters_.simulation_step = clamp(parameters_.simulation_step, 0.03, 0.30);
  parameters_.prediction_horizon = std::max(
    parameters_.simulation_step, parameters_.prediction_horizon);
  parameters_.vx_samples = std::max(2, parameters_.vx_samples);
  parameters_.vy_samples = parameters_.max_vy > 1e-3 ? std::max(3, parameters_.vy_samples) : 1;
  parameters_.wz_samples = std::max(3, parameters_.wz_samples);
  parameters_.recovery_speed = std::max(0.02, parameters_.recovery_speed);
  parameters_.recovery_distance = std::max(0.10, parameters_.recovery_distance);
  parameters_.recovery_direction_samples = std::max(8, parameters_.recovery_direction_samples);
}

void LocalTrajectoryPlanner::reset()
{
  preferred_turn_sign_ = 0;
  clear_cycles_ = 0;
}

LocalTrajectoryPlanner::Evaluation LocalTrajectoryPlanner::evaluate(
  const Velocity2D & velocity,
  const Velocity2D & desired_velocity,
  const Point2D & target,
  const std::vector<Point2D> & nearby_points,
  const Velocity2D & previous_velocity) const
{
  Evaluation result;
  result.clearance = parameters_.activation_distance;
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  const double speed = std::hypot(velocity.x, velocity.y);
  const double braking_distance = speed * speed /
    (2.0 * std::max(0.05, parameters_.braking_deceleration));
  const double required_clearance = parameters_.safety_margin + braking_distance;
  const int steps = std::max(
    1, static_cast<int>(std::ceil(
      parameters_.prediction_horizon / parameters_.simulation_step)));

  for (int step = 0; step < steps; ++step) {
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    x += (c * velocity.x - s * velocity.y) * parameters_.simulation_step;
    y += (s * velocity.x + c * velocity.y) * parameters_.simulation_step;
    yaw = normalizeAngle(yaw + velocity.yaw * parameters_.simulation_step);
    for (const Point2D & point : nearby_points) {
      const double clearance =
        std::hypot(point.x - x, point.y - y) - parameters_.robot_radius;
      result.clearance = std::min(result.clearance, clearance);
      if (clearance <= required_clearance) return result;
    }
  }

  result.collision_free = true;
  const double initial_target_distance = std::hypot(target.x, target.y);
  const double final_target_distance = std::hypot(target.x - x, target.y - y);
  result.progress = initial_target_distance - final_target_distance;
  const double velocity_difference =
    normalizedDifference(velocity.x, desired_velocity.x, parameters_.max_vx) +
    normalizedDifference(velocity.y, desired_velocity.y, std::max(0.05, parameters_.max_vy)) +
    normalizedDifference(velocity.yaw, desired_velocity.yaw, parameters_.max_wz);
  const double command_change =
    normalizedDifference(velocity.x, previous_velocity.x, parameters_.max_vx) +
    normalizedDifference(velocity.y, previous_velocity.y, std::max(0.05, parameters_.max_vy)) +
    normalizedDifference(velocity.yaw, previous_velocity.yaw, parameters_.max_wz);

  double hysteresis_penalty = 0.0;
  const int candidate_turn_sign = turnSign(velocity);
  if (preferred_turn_sign_ != 0 && candidate_turn_sign != 0 &&
    candidate_turn_sign != preferred_turn_sign_)
  {
    hysteresis_penalty = parameters_.hysteresis_weight;
  }
  // For a holonomic base, translation progress is preferable to turning the
  // chassis toward the path.  Only rotation beyond what the controller asked
  // for is penalized here.
  const double excess_rotation = std::max(
    0.0, std::abs(velocity.yaw) - std::abs(desired_velocity.yaw));
  const double rotation_penalty = parameters_.heading_weight *
    excess_rotation / parameters_.max_wz;
  const double reverse_penalty = parameters_.reverse_weight *
    std::max(0.0, -velocity.x) / std::max(0.02, parameters_.max_reverse_speed);
  const double stalled_penalty = speed < 0.03 ? parameters_.stalled_translation_penalty : 0.0;
  result.cost =
    parameters_.target_weight * final_target_distance +
    parameters_.clearance_weight * std::min(result.clearance, parameters_.activation_distance) -
    parameters_.progress_weight * result.progress +
    parameters_.desired_velocity_weight * velocity_difference +
    parameters_.change_weight * command_change +
    hysteresis_penalty + rotation_penalty + reverse_penalty + stalled_penalty;
  return result;
}

LocalTrajectoryResult LocalTrajectoryPlanner::recoveryPlan(
  const Point2D & target,
  const std::vector<Point2D> & nearby_points,
  const Velocity2D & previous_velocity)
{
  LocalTrajectoryResult result;
  result.avoiding = true;
  result.recovering = true;
  const auto clearance_at = [&](double x, double y) {
      double clearance = parameters_.activation_distance;
      for (const Point2D & point : nearby_points) {
        clearance = std::min(
          clearance,
          std::hypot(point.x - x, point.y - y) - parameters_.robot_radius);
      }
      return clearance;
    };

  const double initial_clearance = clearance_at(0.0, 0.0);
  const double target_distance = std::hypot(target.x, target.y);
  const bool obstacle_ahead = std::any_of(
    nearby_points.begin(), nearby_points.end(), [&](const Point2D & point) {
      return point.x > 0.0 && point.x < parameters_.activation_distance &&
             std::abs(point.y) < parameters_.robot_radius + parameters_.safety_margin;
    });
  double best_score = -std::numeric_limits<double>::infinity();
  Velocity2D best_velocity;
  double best_final_clearance = initial_clearance;

  for (int sample = 0; sample < parameters_.recovery_direction_samples; ++sample) {
    const double direction =
      -kPi + 2.0 * kPi * sample / parameters_.recovery_direction_samples;
    Velocity2D candidate;
    candidate.x = parameters_.recovery_speed * std::cos(direction);
    candidate.y = parameters_.recovery_speed * std::sin(direction);
    candidate.x = clamp(
      candidate.x, -parameters_.max_reverse_speed, parameters_.max_vx);
    candidate.y = clamp(candidate.y, -parameters_.max_vy, parameters_.max_vy);
    if (std::hypot(candidate.x, candidate.y) < 0.015) continue;

    const double speed = std::hypot(candidate.x, candidate.y);
    const int steps = std::max(
      2, static_cast<int>(std::ceil(
        parameters_.recovery_distance /
        (speed * parameters_.simulation_step))));
    double x = 0.0;
    double y = 0.0;
    double previous_clearance = initial_clearance;
    bool moves_away = true;
    for (int step = 0; step < steps; ++step) {
      x += candidate.x * parameters_.simulation_step;
      y += candidate.y * parameters_.simulation_step;
      const double clearance = clearance_at(x, y);
      if (clearance + 0.005 < previous_clearance) {
        moves_away = false;
        break;
      }
      previous_clearance = clearance;
    }
    if (!moves_away ||
      previous_clearance < initial_clearance + parameters_.recovery_minimum_improvement)
    {
      continue;
    }

    const double final_target_distance = std::hypot(target.x - x, target.y - y);
    const double progress = target_distance - final_target_distance;
    const int candidate_sign = turnSign(candidate);
    const double hysteresis_penalty =
      preferred_turn_sign_ != 0 && candidate_sign != 0 &&
      candidate_sign != preferred_turn_sign_ ? parameters_.hysteresis_weight : 0.0;
    const double command_change =
      normalizedDifference(candidate.x, previous_velocity.x, parameters_.max_vx) +
      normalizedDifference(candidate.y, previous_velocity.y, std::max(0.05, parameters_.max_vy));
    const double lateral_ratio = std::abs(candidate.y) / std::max(0.02, parameters_.max_vy);
    const double reverse_ratio =
      std::max(0.0, -candidate.x) / std::max(0.02, parameters_.max_reverse_speed);
    const double score =
      3.0 * (previous_clearance - initial_clearance) +
      0.8 * progress -
      0.15 * command_change -
      hysteresis_penalty +
      (obstacle_ahead ? parameters_.recovery_lateral_weight * lateral_ratio : 0.0) -
      parameters_.reverse_weight * reverse_ratio;
    if (score > best_score) {
      best_score = score;
      best_velocity = candidate;
      best_final_clearance = previous_clearance;
    }
  }

  if (!std::isfinite(best_score)) {
    result.blocked = true;
    return result;
  }
  result.valid = true;
  result.velocity = best_velocity;
  result.minimum_clearance = best_final_clearance;
  const int selected_sign = turnSign(best_velocity);
  if (selected_sign != 0) preferred_turn_sign_ = selected_sign;
  return result;
}

LocalTrajectoryResult LocalTrajectoryPlanner::plan(
  const Velocity2D & desired_velocity,
  const Point2D & target_in_base_frame,
  const std::vector<Point2D> & obstacle_points,
  const Velocity2D & previous_velocity)
{
  LocalTrajectoryResult result;
  std::vector<Point2D> nearby_points;
  const double relevant_distance = parameters_.activation_distance +
    parameters_.robot_radius +
    parameters_.max_vx * parameters_.prediction_horizon;
  nearby_points.reserve(obstacle_points.size());
  for (const Point2D & point : obstacle_points) {
    if (std::hypot(point.x, point.y) <= relevant_distance) nearby_points.push_back(point);
  }

  if (nearby_points.empty()) {
    ++clear_cycles_;
    if (clear_cycles_ > 5) preferred_turn_sign_ = 0;
    result.velocity = desired_velocity;
    result.valid = true;
    result.minimum_clearance = relevant_distance;
    return result;
  }
  clear_cycles_ = 0;

  const bool obstacle_ahead = std::any_of(
    nearby_points.begin(), nearby_points.end(), [&](const Point2D & point) {
      return point.x > 0.0 && point.x < parameters_.activation_distance &&
             std::abs(point.y) < parameters_.robot_radius + parameters_.preferred_clearance;
    });

  const Evaluation desired_evaluation = evaluate(
    desired_velocity, desired_velocity, target_in_base_frame, nearby_points, previous_velocity);
  if (desired_evaluation.collision_free &&
    desired_evaluation.clearance >= parameters_.preferred_clearance)
  {
    result.velocity = desired_velocity;
    result.valid = true;
    result.minimum_clearance = desired_evaluation.clearance;
    return result;
  }

  double best_cost = std::numeric_limits<double>::infinity();
  Evaluation best_evaluation;
  Velocity2D best_velocity;
  const auto consider = [&](const Velocity2D & candidate) {
      Evaluation evaluation = evaluate(
        candidate, desired_velocity, target_in_base_frame, nearby_points, previous_velocity);
      if (evaluation.collision_free && turnSign(candidate) == 0 &&
        std::hypot(desired_velocity.x, desired_velocity.y) > 0.03)
      {
        evaluation.cost += parameters_.avoidance_commitment_weight;
      }
      if (evaluation.collision_free && obstacle_ahead && parameters_.max_vy > 0.02) {
        const double lateral_ratio = std::min(
          1.0, std::abs(candidate.y) / parameters_.max_vy);
        evaluation.cost += parameters_.recovery_lateral_weight * (1.0 - lateral_ratio);
      }
      if (evaluation.collision_free && evaluation.cost < best_cost) {
        best_cost = evaluation.cost;
        best_evaluation = evaluation;
        best_velocity = candidate;
      }
    };

  consider(desired_velocity);
  consider(Velocity2D());
  for (int x_sample = 0; x_sample < parameters_.vx_samples; ++x_sample) {
    const double vx = sampleValue(
      -parameters_.max_reverse_speed, parameters_.max_vx,
      x_sample, parameters_.vx_samples);
    for (int y_sample = 0; y_sample < parameters_.vy_samples; ++y_sample) {
      const double vy = parameters_.vy_samples == 1 ? 0.0 : sampleValue(
        -parameters_.max_vy, parameters_.max_vy,
        y_sample, parameters_.vy_samples);
      for (int yaw_sample = 0; yaw_sample < parameters_.wz_samples; ++yaw_sample) {
        const double wz = sampleValue(
          -parameters_.max_wz, parameters_.max_wz,
          yaw_sample, parameters_.wz_samples);
        consider({vx, vy, wz});
      }
    }
  }

  result.avoiding = true;
  if (!std::isfinite(best_cost)) {
    return recoveryPlan(target_in_base_frame, nearby_points, previous_velocity);
  }
  result.velocity = best_velocity;
  result.valid = true;
  result.minimum_clearance = best_evaluation.clearance;
  const double desired_motion = std::hypot(desired_velocity.x, desired_velocity.y) +
    std::abs(desired_velocity.yaw);
  const double selected_motion = std::hypot(best_velocity.x, best_velocity.y) +
    std::abs(best_velocity.yaw);
  result.blocked = desired_motion > 0.04 && selected_motion < 0.02;

  if (result.blocked) {
    const LocalTrajectoryResult recovery = recoveryPlan(
      target_in_base_frame, nearby_points, previous_velocity);
    if (recovery.valid) return recovery;
  }

  const int selected_turn_sign = turnSign(best_velocity);
  if (selected_turn_sign != 0) preferred_turn_sign_ = selected_turn_sign;
  return result;
}

}  // namespace chuyun_navigation2
