#include "chuyun_navigation2/a_star_planner.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

namespace chuyun_navigation2
{
namespace
{

struct QueueEntry
{
  float cost;
  int index;
  bool operator<(const QueueEntry & other) const {return cost > other.cost;}
};

}  // namespace

AStarPlanner::AStarPlanner(const GridMap & map, PlannerParameters parameters)
: map_(map), parameters_(parameters)
{
}

PlanResult AStarPlanner::plan(
  const Pose2D & start,
  const Pose2D & goal,
  const std::vector<Point2D> & temporary_obstacles) const
{
  PlanResult result;
  result.adjusted_goal = goal;
  int start_x = 0;
  int start_y = 0;
  int goal_x = 0;
  int goal_y = 0;
  if (!map_.worldToCell(start.x, start.y, start_x, start_y) ||
    !map_.worldToCell(goal.x, goal.y, goal_x, goal_y))
  {
    result.message = "起点或目标点位于地图范围外";
    return result;
  }

  const int width = static_cast<int>(map_.occupancyGrid().info.width);
  const int height = static_cast<int>(map_.occupancyGrid().info.height);
  const double resolution = map_.occupancyGrid().info.resolution;
  std::vector<uint8_t> temporary_blocked(static_cast<size_t>(width) * height, 0);
  std::vector<float> temporary_clearance_cost(static_cast<size_t>(width) * height, 0.0F);
  const double hard_radius = std::max(0.0, parameters_.temporary_obstacle_radius);
  const double clearance_radius = std::max(
    hard_radius, parameters_.temporary_clearance_radius);
  const int clearance_radius_cells = std::max(
    0, static_cast<int>(std::ceil(clearance_radius / resolution)));
  for (const Point2D & obstacle : temporary_obstacles) {
    int obstacle_x = 0;
    int obstacle_y = 0;
    if (!map_.worldToCell(obstacle.x, obstacle.y, obstacle_x, obstacle_y)) continue;
    for (int dy = -clearance_radius_cells; dy <= clearance_radius_cells; ++dy) {
      for (int dx = -clearance_radius_cells; dx <= clearance_radius_cells; ++dx) {
        const double distance = std::hypot(dx, dy) * resolution;
        if (distance > clearance_radius) continue;
        const int x = obstacle_x + dx;
        const int y = obstacle_y + dy;
        if (x < 0 || y < 0 || x >= width || y >= height) continue;
        const size_t cell_index = static_cast<size_t>(y) * width + x;
        if (distance <= hard_radius) temporary_blocked[cell_index] = 1;
        const double ratio = clamp(
          (clearance_radius - distance) /
          std::max(resolution, clearance_radius - hard_radius), 0.0, 1.0);
        temporary_clearance_cost[cell_index] = std::max(
          temporary_clearance_cost[cell_index], static_cast<float>(ratio));
      }
    }
  }
  const auto is_free = [&](int x, int y) {
      return x >= 0 && y >= 0 && x < width && y < height && map_.isFree(x, y) &&
             !temporary_blocked[static_cast<size_t>(y) * width + x];
    };
  const auto nearest_free = [&](int & x, int & y, int maximum_radius) {
      if (is_free(x, y)) return true;
      for (int radius = 1; radius <= maximum_radius; ++radius) {
        for (int dy = -radius; dy <= radius; ++dy) {
          for (int dx = -radius; dx <= radius; ++dx) {
            if (std::abs(dx) != radius && std::abs(dy) != radius) continue;
            if (is_free(x + dx, y + dy)) {
              x += dx;
              y += dy;
              return true;
            }
          }
        }
      }
      return false;
    };
  const auto line_is_free = [&] (
      int x0, int y0, int x1, int y1, float maximum_dynamic_cost) {
      const int dx = std::abs(x1 - x0);
      const int sx = x0 < x1 ? 1 : -1;
      const int dy = -std::abs(y1 - y0);
      const int sy = y0 < y1 ? 1 : -1;
      int error = dx + dy;
      while (true) {
        if (!is_free(x0, y0) ||
          temporary_clearance_cost[static_cast<size_t>(y0) * width + x0] >
          maximum_dynamic_cost)
        {
          return false;
        }
        if (x0 == x1 && y0 == y1) return true;
        const int twice_error = 2 * error;
        if (twice_error >= dy) {
          error += dy;
          x0 += sx;
        }
        if (twice_error <= dx) {
          error += dx;
          y0 += sy;
        }
      }
    };

  const int nearest_radius = std::max(
    1, static_cast<int>(std::ceil(parameters_.nearest_free_radius / resolution)));
  if (!nearest_free(start_x, start_y, nearest_radius) ||
    !nearest_free(goal_x, goal_y, nearest_radius))
  {
    result.message = "起点或目标点附近没有可通行区域";
    return result;
  }

  const int start_index = start_y * width + start_x;
  const int goal_index = goal_y * width + goal_x;
  std::vector<float> costs(static_cast<size_t>(width) * height, std::numeric_limits<float>::infinity());
  std::vector<int> parents(costs.size(), -1);
  std::vector<uint8_t> closed(costs.size(), 0);
  std::priority_queue<QueueEntry> open;
  costs[start_index] = 0.0F;
  open.push({0.0F, start_index});

  const std::array<int, 8> dx{-1, 0, 1, -1, 1, -1, 0, 1};
  const std::array<int, 8> dy{-1, -1, -1, 0, 0, 1, 1, 1};
  const std::array<float, 8> step{
    1.41421356F, 1.0F, 1.41421356F, 1.0F,
    1.0F, 1.41421356F, 1.0F, 1.41421356F};

  bool found = false;
  while (!open.empty()) {
    const int current = open.top().index;
    open.pop();
    if (closed[current]) continue;
    closed[current] = 1;
    if (current == goal_index) {
      found = true;
      break;
    }
    const int x = current % width;
    const int y = current / width;
    for (size_t direction = 0; direction < dx.size(); ++direction) {
      const int next_x = x + dx[direction];
      const int next_y = y + dy[direction];
      if (!is_free(next_x, next_y)) continue;
      if (dx[direction] != 0 && dy[direction] != 0 &&
        (!is_free(x + dx[direction], y) || !is_free(x, y + dy[direction])))
      {
        continue;
      }
      const int next = next_y * width + next_x;
      const double clearance = map_.distanceAtCell(next_x, next_y);
      const double clearance_ratio = clamp(
        (parameters_.preferred_clearance - clearance) /
        std::max(0.01, parameters_.preferred_clearance), 0.0, 1.0);
      const double dynamic_clearance = temporary_clearance_cost[next];
      const float travel_cost = step[direction] * static_cast<float>(
        1.0 + parameters_.clearance_cost_weight * clearance_ratio * clearance_ratio +
        parameters_.temporary_clearance_cost_weight *
        dynamic_clearance * dynamic_clearance);
      const float candidate = costs[current] + travel_cost;
      if (candidate >= costs[next]) continue;
      costs[next] = candidate;
      parents[next] = current;
      const float heuristic = std::hypot(
        static_cast<float>(goal_x - next_x), static_cast<float>(goal_y - next_y));
      open.push({candidate + heuristic, next});
    }
  }
  if (!found) {
    result.message = "A* 没有找到可通行路径";
    return result;
  }

  std::vector<std::pair<int, int>> cells;
  for (int current = goal_index; current >= 0; current = parents[current]) {
    cells.emplace_back(current % width, current / width);
    if (current == start_index) break;
  }
  std::reverse(cells.begin(), cells.end());

  std::vector<std::pair<int, int>> simplified;
  simplified.push_back(cells.front());
  size_t anchor = 0;
  while (anchor + 1 < cells.size()) {
    size_t farthest = anchor + 1;
    for (size_t candidate = anchor + 2; candidate < cells.size(); ++candidate) {
      if (!line_is_free(
          cells[anchor].first, cells[anchor].second,
          cells[candidate].first, cells[candidate].second, 0.70F))
      {
        break;
      }
      farthest = candidate;
    }
    simplified.push_back(cells[farthest]);
    anchor = farthest;
  }

  std::vector<Pose2D> sparse_path;
  sparse_path.reserve(simplified.size());
  for (const auto & cell : simplified) {
    const auto world = map_.cellToWorld(cell.first, cell.second);
    sparse_path.push_back({world.x, world.y, 0.0});
  }

  // String pulling removes unnecessary grid turns, but its remaining corners
  // are still sharp.  A few collision-checked Chaikin passes round those
  // corners without allowing the path to cut through inflated obstacles.
  for (int iteration = 0;
    iteration < std::max(0, parameters_.path_smoothing_iterations) &&
    sparse_path.size() > 2; ++iteration)
  {
    std::vector<Pose2D> candidate;
    candidate.reserve(sparse_path.size() * 2);
    candidate.push_back(sparse_path.front());
    for (size_t i = 0; i + 1 < sparse_path.size(); ++i) {
      const Pose2D & from = sparse_path[i];
      const Pose2D & to = sparse_path[i + 1];
      candidate.push_back({
        0.75 * from.x + 0.25 * to.x,
        0.75 * from.y + 0.25 * to.y,
        0.0});
      candidate.push_back({
        0.25 * from.x + 0.75 * to.x,
        0.25 * from.y + 0.75 * to.y,
        0.0});
    }
    candidate.push_back(sparse_path.back());

    bool safe = true;
    for (size_t i = 1; i < candidate.size(); ++i) {
      int x0 = 0;
      int y0 = 0;
      int x1 = 0;
      int y1 = 0;
      if (!map_.worldToCell(candidate[i - 1].x, candidate[i - 1].y, x0, y0) ||
        !map_.worldToCell(candidate[i].x, candidate[i].y, x1, y1) ||
        !line_is_free(x0, y0, x1, y1, 1.0F))
      {
        safe = false;
        break;
      }
    }
    if (!safe) break;
    sparse_path = std::move(candidate);
  }

  result.path.push_back(sparse_path.front());
  for (size_t i = 1; i < sparse_path.size(); ++i) {
    const Pose2D & from = sparse_path[i - 1];
    const Pose2D & to = sparse_path[i];
    const double length = std::hypot(to.x - from.x, to.y - from.y);
    const int samples = std::max(
      1, static_cast<int>(std::ceil(length / std::max(0.01, parameters_.path_point_spacing))));
    for (int sample = 1; sample <= samples; ++sample) {
      const double ratio = static_cast<double>(sample) / samples;
      result.path.push_back({
        from.x + ratio * (to.x - from.x),
        from.y + ratio * (to.y - from.y),
        0.0});
    }
  }

  const auto adjusted_goal = map_.cellToWorld(goal_x, goal_y);
  result.adjusted_goal = {adjusted_goal.x, adjusted_goal.y, goal.yaw};
  result.path.back().yaw = goal.yaw;
  result.success = true;
  result.message = "OK";
  return result;
}

}  // namespace chuyun_navigation2
