#pragma once

#include <algorithm>
#include <cstddef>
#include <cmath>

namespace amr_dispatcher_core::path_tracking {

// 路径几何公共工具：角度归一化、横向误差、前向投影、终点判定。
// 纯函数集合，无外部依赖，供 Pure Pursuit / Stanley 控制器复用。

inline double NormalizeAngle(double angle) {
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

// 机器人在路径切向坐标系下的带符号横向偏差。
// 正值表示路径点在机器人左侧，负值表示右侧。
inline double SignedLateralError(
    double path_x, double path_y, double path_yaw, double robot_x, double robot_y) {
  const double dx = path_x - robot_x;
  const double dy = path_y - robot_y;
  return std::sin(path_yaw) * dx - std::cos(path_yaw) * dy;
}

// 目标点相对机器人朝向的前向投影分量，用于过滤身后的路径点。
inline double ForwardProjection(double robot_yaw, double dx, double dy) {
  return std::cos(robot_yaw) * dx + std::sin(robot_yaw) * dy;
}

// 终点判定：最近点已是路径末端且距终点小于容差。
inline bool IsFinalWaypointReached(
    int nearest_index, std::size_t path_size, double distance_to_goal, double goal_tolerance) {
  return path_size > 0 && nearest_index == static_cast<int>(path_size - 1) &&
         distance_to_goal <= std::max(0.0, goal_tolerance);
}

}  // namespace amr_dispatcher_core::path_tracking
