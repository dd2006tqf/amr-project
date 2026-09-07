#pragma once

#include <cmath>
#include <string>
#include <vector>

namespace amr_dispatcher_core {

struct Point2D {
  double x{0.0};
  double y{0.0};
};

struct Pose2D {
  double x{0.0};
  double y{0.0};
  double theta{0.0};  // 航向角 (rad), 范围 [-M_PI, M_PI]
};

/**
 * @brief 路径规划器抽象基类 (IoC 控制反转接口)
 * 调度系统与业务核心仅依赖此纯虚接口，不依赖任何具体的图搜索、几何或数值优化算法。
 * 留出标准扩展插槽，未来可派生实现 A*、Hybrid A*、Nav2 插件等。
 */
class IPathPlanner {
 public:
  virtual ~IPathPlanner() = default;

  /**
   * @brief 在起点与终点之间规划离散二维路径点序列
   * @param start 起点二维位姿 (x, y, theta)
   * @param goal 终点二维位姿 (x, y, theta)
   * @param edge_id 可选的拓扑边编号，供基于图先验规划使用
   * @return std::vector<Pose2D> 生成的路径序列；若规划失败返回空序列
   */
  virtual std::vector<Pose2D> Plan(const Pose2D& start, const Pose2D& goal,
                                   const std::string& edge_id = "") = 0;

  /**
   * @brief 获取规划器实现名称
   */
  virtual std::string GetPlannerName() const = 0;
};

}  // namespace amr_dispatcher_core
