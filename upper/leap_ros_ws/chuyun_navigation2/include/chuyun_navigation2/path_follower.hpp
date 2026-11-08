#pragma once

#include <string>
#include <vector>

#include "chuyun_navigation2/local_trajectory_planner.hpp"
#include "chuyun_navigation2/types.hpp"

namespace chuyun_navigation2
{

struct ControllerParameters
{
  double max_vx{0.18};
  double max_vy{0.12};
  double max_reverse_speed{0.08};
  double max_wz{0.70};
  double max_xy_speed{0.18};
  double max_ax{0.35};
  double max_ay{0.30};
  double max_aw{0.50};
  double linear_gain{1.0};
  double angular_gain{1.4};
  double lookahead_distance{0.24};
  double rotate_before_translate_angle{1.05};
  double heading_deadband{0.70};
  int path_search_window{80};
  double goal_position_tolerance{0.10};
  double goal_yaw_tolerance{0.12};
};

struct ControlResult
{
  Velocity2D velocity;
  std::string status{"IDLE"};
  double goal_distance{0.0};
  bool arrived{false};
  bool local_planner_active{false};
  bool local_planner_blocked{false};
};

class PathFollower
{
public:
  explicit PathFollower(
    ControllerParameters parameters,
    LocalPlannerParameters local_planner_parameters = LocalPlannerParameters());

  void setPath(std::vector<Pose2D> path, const Pose2D & goal);
  void clear();
  bool active() const {return active_;}
  const std::vector<Pose2D> & path() const {return path_;}
  const Pose2D & goal() const {return goal_;}

  ControlResult update(
    const Pose2D & current_pose,
    const std::vector<Point2D> & obstacle_points,
    bool scan_is_fresh,
    double dt);

private:
  void applyRateLimits(Velocity2D & velocity, double dt);

  ControllerParameters parameters_;
  LocalTrajectoryPlanner local_planner_;
  std::vector<Pose2D> path_;
  size_t path_index_{0};
  Pose2D goal_;
  Velocity2D previous_velocity_;
  bool active_{false};
};

}  // namespace chuyun_navigation2
