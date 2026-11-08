#pragma once

#include <string>
#include <vector>

#include "amr_dispatcher_core/planning/path_planner_interface.hpp"

namespace amr_dispatcher_core::path_tracking {

struct TwistCommand {
  double linear_x{0.0};          // 期望线速度 (m/s)
  double angular_z{0.0};         // 期望角速度 (rad/s)
  bool reached_goal{false};      // 是否已到达目标点
  double cross_track_error{0.0}; // 横向跟踪误差 (m)
  double heading_error{0.0};     // 航向角误差 (rad)
};

/**
 * @brief 路径跟踪控制器抽象基类 (IoC 控制反转接口)
 * 无论底盘是使用纯几何前视 Pure Pursuit、Stanley，还是未来的 MPC / LQR，
 * 节点与上层逻辑均面向此抽象接口交互。
 */
class IPathTracker {
 public:
  virtual ~IPathTracker() = default;

  /**
   * @brief 设置或更新待跟踪路径
   */
  virtual void SetPath(std::vector<amr_dispatcher_core::Pose2D> path) = 0;

  /**
   * @brief 清空当前跟踪路径
   */
  virtual void ClearPath() = 0;

  /**
   * @brief 是否持有有效目标路径
   */
  virtual bool HasPath() const = 0;

  /**
   * @brief 根据机器人当前位姿解算速度控制指令
   */
  virtual TwistCommand ComputeControl(const amr_dispatcher_core::Pose2D& current_pose) const = 0;

  /**
   * @brief 获取控制器名称
   */
  virtual std::string GetTrackerName() const = 0;
};

}  // namespace amr_dispatcher_core::path_tracking
