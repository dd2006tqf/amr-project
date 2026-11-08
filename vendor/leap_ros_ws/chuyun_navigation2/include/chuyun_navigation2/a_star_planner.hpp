#pragma once

#include <string>
#include <vector>

#include "chuyun_navigation2/grid_map.hpp"
#include "chuyun_navigation2/types.hpp"

namespace chuyun_navigation2
{

struct PlannerParameters
{
  double path_point_spacing{0.06};
  int path_smoothing_iterations{2};
  double preferred_clearance{0.40};
  double clearance_cost_weight{6.0};
  double nearest_free_radius{0.60};
  double temporary_obstacle_radius{0.38};
  double temporary_clearance_radius{0.70};
  double temporary_clearance_cost_weight{10.0};
};

struct PlanResult
{
  bool success{false};
  std::string message;
  Pose2D adjusted_goal;
  std::vector<Pose2D> path;
};

class AStarPlanner
{
public:
  AStarPlanner(const GridMap & map, PlannerParameters parameters);
  PlanResult plan(
    const Pose2D & start,
    const Pose2D & goal,
    const std::vector<Point2D> & temporary_obstacles = {}) const;

private:
  const GridMap & map_;
  PlannerParameters parameters_;
};

}  // namespace chuyun_navigation2
