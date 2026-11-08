#pragma once

#include <cmath>
#include "amr_dispatcher_core/path_tracking/path_geometry.hpp"

namespace amr_dispatcher_core::path_tracking {

// 角度差计算 (target - current)，自动归一化
inline double AngleDifference(double target, double current) {
  return NormalizeAngle(target - current);
}

// 二维点间欧几里得距离
inline double EuclideanDistance(double x1, double y1, double x2, double y2) {
  const double dx = x1 - x2;
  const double dy = y1 - y2;
  return std::sqrt(dx * dx + dy * dy);
}

// 四元数简化表示 (2D 平面偏航角 yaw 转换)
struct Quaternion2D {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double w = 1.0;
};

inline Quaternion2D QuaternionFromYaw(double yaw) {
  Quaternion2D q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

inline double YawFromQuaternion(double z, double w) {
  return NormalizeAngle(2.0 * std::atan2(z, w));
}

}  // namespace amr_dispatcher_core::path_tracking
