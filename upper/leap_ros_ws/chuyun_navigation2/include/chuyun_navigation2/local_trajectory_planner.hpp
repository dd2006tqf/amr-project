#pragma once

#include <vector>

#include "chuyun_navigation2/types.hpp"

namespace chuyun_navigation2
{

struct LocalPlannerParameters
{
  double max_vx{0.18};
  double max_vy{0.12};
  double max_wz{0.70};
  double max_reverse_speed{0.08};
  double robot_radius{0.18};
  double safety_margin{0.03};
  double activation_distance{0.75};
  double preferred_clearance{0.22};
  double prediction_horizon{1.20};
  double simulation_step{0.12};
  double braking_deceleration{0.35};
  int vx_samples{5};
  int vy_samples{7};
  int wz_samples{9};
  double target_weight{3.0};
  double heading_weight{0.8};
  double clearance_weight{1.2};
  double desired_velocity_weight{0.7};
  double progress_weight{2.0};
  double change_weight{0.25};
  double hysteresis_weight{0.35};
  double avoidance_commitment_weight{0.60};
  double reverse_weight{0.80};
  double stalled_translation_penalty{0.70};
  double recovery_lateral_weight{1.40};
  double recovery_speed{0.08};
  double recovery_distance{0.35};
  double recovery_minimum_improvement{0.03};
  int recovery_direction_samples{24};
};

struct LocalTrajectoryResult
{
  Velocity2D velocity;
  bool valid{false};
  bool avoiding{false};
  bool recovering{false};
  bool blocked{false};
  double minimum_clearance{0.0};
};

// Samples short, constant-velocity trajectories against the latest scan.
// It is intentionally small enough for low-power ARM computers and remembers
// the selected passing side to prevent left/right oscillation near obstacles.
class LocalTrajectoryPlanner
{
public:
  explicit LocalTrajectoryPlanner(LocalPlannerParameters parameters);

  LocalTrajectoryResult plan(
    const Velocity2D & desired_velocity,
    const Point2D & target_in_base_frame,
    const std::vector<Point2D> & obstacle_points,
    const Velocity2D & previous_velocity);
  void reset();

private:
  struct Evaluation
  {
    bool collision_free{false};
    double cost{0.0};
    double clearance{0.0};
    double progress{0.0};
  };

  Evaluation evaluate(
    const Velocity2D & velocity,
    const Velocity2D & desired_velocity,
    const Point2D & target,
    const std::vector<Point2D> & nearby_points,
    const Velocity2D & previous_velocity) const;
  LocalTrajectoryResult recoveryPlan(
    const Point2D & target,
    const std::vector<Point2D> & nearby_points,
    const Velocity2D & previous_velocity);

  LocalPlannerParameters parameters_;
  int preferred_turn_sign_{0};
  int clear_cycles_{0};
};

}  // namespace chuyun_navigation2
