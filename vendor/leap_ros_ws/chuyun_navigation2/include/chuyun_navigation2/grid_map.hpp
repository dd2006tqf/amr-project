#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "chuyun_navigation2/types.hpp"

namespace chuyun_navigation2
{

class GridMap
{
public:
  void load(const std::string & yaml_path, const std::string & frame_id);
  void buildFields(double inflation_radius, double maximum_distance);

  const nav_msgs::msg::OccupancyGrid & occupancyGrid() const {return occupancy_grid_;}
  const nav_msgs::msg::OccupancyGrid & inflatedGrid() const {return inflated_grid_;}

  bool worldToCell(double world_x, double world_y, int & cell_x, int & cell_y) const;
  Point2D cellToWorld(int cell_x, int cell_y) const;
  bool isFree(int cell_x, int cell_y) const;
  bool nearestFree(int & cell_x, int & cell_y, int maximum_radius_cells) const;
  bool lineIsFree(int x0, int y0, int x1, int y1) const;
  bool distanceAtWorld(double world_x, double world_y, double & distance) const;
  double distanceAtCell(int cell_x, int cell_y) const;

private:
  bool inBounds(int cell_x, int cell_y) const;
  size_t index(int cell_x, int cell_y) const;

  nav_msgs::msg::OccupancyGrid occupancy_grid_;
  nav_msgs::msg::OccupancyGrid inflated_grid_;
  std::vector<float> distance_field_;
  double map_origin_yaw_{0.0};
  double maximum_distance_{2.0};
};

}  // namespace chuyun_navigation2
