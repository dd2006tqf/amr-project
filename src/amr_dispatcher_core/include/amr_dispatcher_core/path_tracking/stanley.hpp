#pragma once

#include <cstddef>
#include <vector>

#include "amr_dispatcher_core/path_tracking/pure_pursuit.hpp"

namespace amr_dispatcher_core::path_tracking {

// Stanley 横向误差控制一次的输出。
struct StanleyResult {
  Twist2D cmd;
  double lateral_error = 0.0;
  double heading_error = 0.0;
  int nearest_index = 0;
  bool goal_reached = false;
};

// Stanley 控制器：航向误差 + atan(k·e / (v + softening)) 横向补偿。
// 与 Pure Pursuit 共用路径缓存与最近点搜索，ROS 无关。
class StanleyController {
 public:
  struct Config {
    double gain = 1.0;              // 横向误差增益 k
    double softening = 0.2;         // 软化项，防止低速时 atan 饱和
    double target_speed = 0.35;     // 期望线速度 (m/s)
    double max_angular_speed = 1.2; // 角速度限幅 (rad/s)
    double goal_tolerance = 0.10;   // 终点判定容差 (m)
  };

  explicit StanleyController(Config config);
  StanleyController() : StanleyController(Config{}) {}

  void SetPath(std::vector<Pose2D> path);
  bool HasPath() const { return path_.size() >= 2; }
  void ClearPath() { path_.clear(); }

  StanleyResult Compute(const Pose2D& robot_pose) const;

  const Config& config() const { return config_; }

 private:
  int NearestIndex(const Pose2D& robot_pose) const;

  Config config_;
  std::vector<Pose2D> path_;
};

}  // namespace amr_dispatcher_core::path_tracking
