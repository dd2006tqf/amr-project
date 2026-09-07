#pragma once

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "amr_dispatcher_core/planning/path_planner_interface.hpp"

namespace amr_dispatcher_core {

/**
 * @brief 轻量基准线性路径规划器 (Mock / Baseline 实现)
 * 在起点与终点之间做等距几何插值，不依赖外部代价地图与复杂搜索算法。
 * 用于系统基础设施链路验证、极速单测与全流程调度跑通。
 */
class MockLinearPlanner : public IPathPlanner {
 public:
  explicit MockLinearPlanner(double step_size = 0.1) : step_size_(step_size) {
    if (step_size_ <= 0.0) {
      step_size_ = 0.1;
    }
  }

  std::vector<Pose2D> Plan(const Pose2D& start, const Pose2D& goal,
                           const std::string& /*edge_id*/ = "") override {
    std::vector<Pose2D> path;
    const double dx = goal.x - start.x;
    const double dy = goal.y - start.y;
    const double distance = std::hypot(dx, dy);

    if (distance < 1e-4) {
      path.push_back(start);
      return path;
    }

    const double heading = std::atan2(dy, dx);
    const int num_steps = static_cast<int>(std::ceil(distance / step_size_));

    path.reserve(num_steps + 1);
    for (int i = 0; i <= num_steps; ++i) {
      const double ratio = static_cast<double>(i) / num_steps;
      Pose2D p;
      p.x = start.x + ratio * dx;
      p.y = start.y + ratio * dy;
      p.theta = (i == num_steps) ? goal.theta : heading;
      path.push_back(p);
    }

    return path;
  }

  std::string GetPlannerName() const override { return "MockLinearPlanner"; }

  double GetStepSize() const { return step_size_; }
  void SetStepSize(double step_size) {
    if (step_size > 0.0) {
      step_size_ = step_size;
    }
  }

 private:
  double step_size_{0.1};
};

}  // namespace amr_dispatcher_core
