#pragma once

#include <algorithm>
#include <cmath>

namespace chuyun_navigation2
{

constexpr double kPi = 3.14159265358979323846;

struct Point2D
{
  double x{0.0};
  double y{0.0};
};

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct Velocity2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

inline double clamp(double value, double low, double high)
{
  return std::max(low, std::min(high, value));
}

inline double normalizeAngle(double angle)
{
  while (angle > kPi) angle -= 2.0 * kPi;
  while (angle < -kPi) angle += 2.0 * kPi;
  return angle;
}

inline Pose2D compose(const Pose2D & first, const Pose2D & second)
{
  const double c = std::cos(first.yaw);
  const double s = std::sin(first.yaw);
  return {
    first.x + c * second.x - s * second.y,
    first.y + s * second.x + c * second.y,
    normalizeAngle(first.yaw + second.yaw)};
}

inline Pose2D inverse(const Pose2D & pose)
{
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  return {
    -c * pose.x - s * pose.y,
    s * pose.x - c * pose.y,
    normalizeAngle(-pose.yaw)};
}

inline Pose2D interpolate(const Pose2D & from, const Pose2D & to, double ratio)
{
  ratio = clamp(ratio, 0.0, 1.0);
  return {
    from.x + ratio * (to.x - from.x),
    from.y + ratio * (to.y - from.y),
    normalizeAngle(from.yaw + ratio * normalizeAngle(to.yaw - from.yaw))};
}

}  // namespace chuyun_navigation2
