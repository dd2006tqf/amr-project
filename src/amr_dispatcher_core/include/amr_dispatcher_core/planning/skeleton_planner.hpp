#pragma once

#include <string>
#include <utility>
#include <vector>

#include "amr_dispatcher_core/catalog/station_catalog.hpp"
#include "amr_dispatcher_core/planning/path_planner_interface.hpp"

namespace amr_dispatcher_core::planning {

/**
 * @brief 真实折线路径规划器 (Skeleton Planner)
 * 基于 StationCatalog 混合图（stations + road_nodes）：先由 FindRoutePath
 * 在混合图上执行 Dijkstra 选出节点序列，再顺序拼接各边的真实几何折线
 * （支持双向边反向展开与缺失折线的直线退化），最后按 step_size 等距插值
 * 输出带航向角的 Pose2D 路点序列，供底盘循迹到站。
 */
class SkeletonPlanner : public IPathPlanner {
 public:
  struct Config {
    double step_size = 0.1;  // 采样重插值步长 (米)
  };

  // GCC 不允许在类定义内的默认参数中对含 NSDMI 的嵌套 Config 做
  // `= {}` 聚合值初始化（"default member initializer ... required before the
  // end of its enclosing class"），故按仓库惯例（PurePursuitController）
  // 用委托构造暴露单参形式。
  explicit SkeletonPlanner(catalog::StationCatalog catalog, Config config);
  SkeletonPlanner(catalog::StationCatalog catalog)
      : SkeletonPlanner(std::move(catalog), Config{}) {}

  void UpdateCatalog(catalog::StationCatalog catalog);
  const catalog::StationCatalog& catalog() const { return catalog_; }

  // 实现 IPathPlanner：吸附最近路网节点规划；start/goal 与节点间的间隙
  // 自动拼接接驳直线段，首尾路点严格等于 start/goal。
  std::vector<Pose2D> Plan(const Pose2D& start, const Pose2D& goal,
                           const std::string& edge_id = "") override;
  std::string GetPlannerName() const override { return "SkeletonPlanner"; }

  // 支持直接按站点 / 路网节点 ID 规划（不做接驳，路点全部落在图折线上）
  std::vector<Pose2D> PlanBetweenNodes(const std::string& from_node,
                                       const std::string& to_node);

 private:
  catalog::StationCatalog catalog_;
  Config config_;
};

}  // namespace amr_dispatcher_core::planning
