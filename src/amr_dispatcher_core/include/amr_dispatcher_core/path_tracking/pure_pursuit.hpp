#pragma once

#include <cstddef>
#include <vector>

namespace amr_dispatcher_core::path_tracking {

struct Pose2D {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

struct Twist2D {
  double linear_x = 0.0;
  double angular_z = 0.0;
};

// Pure Pursuit 跟踪一次的完整输出，供上层发布误差话题。
struct PurePursuitResult {
  Twist2D cmd;
  double lateral_error = 0.0;
  double heading_error = 0.0;
  int nearest_index = 0;
  bool goal_reached = false;
};

// Pure Pursuit（前置点跟踪）控制器。
// 输入为 (x, y) 平面路径点序列；算法与 ROS 解耦，节点层只做话题搬运。
class PurePursuitController {
 public:
  struct Config {
    double lookahead_distance = 0.8;   // 前置距离 (m)
    double target_speed = 0.35;        // 期望线速度 (m/s)
    double max_angular_speed = 1.2;    // 角速度限幅 (rad/s)
    double goal_tolerance = 0.10;      // 终点判定容差 (m)
  };

  explicit PurePursuitController(Config config);
  PurePursuitController() : PurePursuitController(Config{}) {}

  void SetPath(std::vector<Pose2D> path);
  bool HasPath() const { return !path_.empty(); }
  void ClearPath() { path_.clear(); }

  // 计算一次控制量；path 为空时返回 goal_reached=false 且零速。
  PurePursuitResult Compute(const Pose2D& robot_pose) const;

  const Config& config() const { return config_; }

 private:
  // 从最近点向前寻找满足前置距离的首个路径点；找不到时退化为首个前向点或末点。
  std::size_t SelectTargetIndex(int nearest_index, const Pose2D& robot_pose) const;
  int NearestIndex(const Pose2D& robot_pose) const;

  Config config_;
  std::vector<Pose2D> path_;
};

}  // namespace amr_dispatcher_core::path_tracking
