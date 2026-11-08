#include "amr_dispatcher_core/planning/skeleton_planner.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace amr_dispatcher_core::planning {
namespace {

constexpr double kEps = 1e-6;

catalog::Station MakeStation(const std::string& id, double x, double y,
                             double yaw = 0.0) {
  catalog::Station station;
  station.id = id;
  station.x = x;
  station.y = y;
  station.yaw = yaw;
  return station;
}

catalog::RoadNode MakeRoadNode(const std::string& id, double x, double y,
                               double yaw = 0.0) {
  catalog::RoadNode node;
  node.id = id;
  node.x = x;
  node.y = y;
  node.yaw = yaw;
  return node;
}

catalog::CatalogStationEdge MakeEdge(
    const std::string& from, const std::string& to, double distance_m,
    std::vector<std::pair<double, double>> polyline = {},
    bool bidirectional = true) {
  catalog::CatalogStationEdge edge;
  edge.from = from;
  edge.to = to;
  edge.distance_m = distance_m;
  edge.bidirectional = bidirectional;
  edge.enabled = true;
  edge.polyline = std::move(polyline);
  return edge;
}

// L 弯拓扑：A(0,0) -- Junc(0,5) -- B(5,5)。
// A、B 为站点，Junc 为路网节点。直线 A->B 会穿过 (2.5,2.5) 的"墙"。
catalog::StationCatalog MakeLCorridorCatalog() {
  catalog::StationCatalog catalog;
  catalog.stations.push_back(MakeStation("A", 0.0, 0.0));
  catalog.stations.push_back(MakeStation("B", 5.0, 5.0));
  catalog.road_nodes.push_back(MakeRoadNode("Junc", 0.0, 5.0));
  catalog.edges.push_back(
      MakeEdge("A", "Junc", 5.0, {{0.0, 0.0}, {0.0, 5.0}}));
  catalog.edges.push_back(
      MakeEdge("Junc", "B", 5.0, {{0.0, 5.0}, {5.0, 5.0}}));
  return catalog;
}

double MinDistanceTo(const std::vector<Pose2D>& path, double x, double y) {
  double best = std::numeric_limits<double>::infinity();
  for (const auto& pose : path) {
    best = std::min(best, std::hypot(pose.x - x, pose.y - y));
  }
  return best;
}

// ---------------------------------------------------------------------------
// 用例 1：L 弯走廊避障 —— 绝不直线穿墙。
// 轨迹必须严格沿 x=0 或 y=5 前进，任何路点都不允许落在 (2.5,2.5) 附近。
// ---------------------------------------------------------------------------
TEST(SkeletonPlannerTest, LCorridor_StraightLineNeverCutThroughWall) {
  SkeletonPlanner planner(MakeLCorridorCatalog());

  auto path = planner.Plan(Pose2D{0.0, 0.0, 0.0}, Pose2D{5.0, 5.0, 0.0});
  ASSERT_FALSE(path.empty());
  EXPECT_NEAR(path.front().x, 0.0, kEps);
  EXPECT_NEAR(path.front().y, 0.0, kEps);
  EXPECT_NEAR(path.back().x, 5.0, kEps);
  EXPECT_NEAR(path.back().y, 5.0, kEps);

  for (const auto& pose : path) {
    const bool on_vertical = std::fabs(pose.x) < kEps;
    const bool on_horizontal = std::fabs(pose.y - 5.0) < kEps;
    EXPECT_TRUE(on_vertical || on_horizontal)
        << "waypoint (" << pose.x << ", " << pose.y
        << ") left the L corridor (wall cut suspected)";
    // 直插点 (2.5, 2.5) 双条件均不满足，上面的断言会直接拦截。
  }
  EXPECT_GT(MinDistanceTo(path, 2.5, 2.5), 1.0);
}

TEST(SkeletonPlannerTest, LCorridor_PlanBetweenNodesFollowsPolyline) {
  SkeletonPlanner planner(MakeLCorridorCatalog());

  auto path = planner.PlanBetweenNodes("A", "B");
  ASSERT_FALSE(path.empty());
  EXPECT_NEAR(path.front().x, 0.0, kEps);
  EXPECT_NEAR(path.front().y, 0.0, kEps);
  EXPECT_NEAR(path.back().x, 5.0, kEps);
  EXPECT_NEAR(path.back().y, 5.0, kEps);
  for (const auto& pose : path) {
    EXPECT_TRUE(std::fabs(pose.x) < kEps || std::fabs(pose.y - 5.0) < kEps)
        << "waypoint (" << pose.x << ", " << pose.y << ") left the L corridor";
  }
}

// ---------------------------------------------------------------------------
// 用例 2：沿边 polyline 的弯曲折线贴合 —— 精准经过关键转弯拐点。
// ---------------------------------------------------------------------------
TEST(SkeletonPlannerTest, BentPolyline_PassesThroughCornerVertices) {
  catalog::StationCatalog catalog;
  catalog.stations.push_back(MakeStation("A", 0.0, 0.0));
  catalog.stations.push_back(MakeStation("B", 2.0, 4.0));
  catalog.edges.push_back(MakeEdge(
      "A", "B", 8.0,
      {{0.0, 0.0}, {0.0, 2.0}, {2.0, 2.0}, {2.0, 4.0}}));

  SkeletonPlanner planner(std::move(catalog));
  auto path = planner.PlanBetweenNodes("A", "B");
  ASSERT_FALSE(path.empty());

  EXPECT_NEAR(path.front().x, 0.0, kEps);
  EXPECT_NEAR(path.front().y, 0.0, kEps);
  EXPECT_NEAR(path.back().x, 2.0, kEps);
  EXPECT_NEAR(path.back().y, 4.0, kEps);

  // 0.1m 等距采样应精确命中 (0,2) 与 (2,2) 两个转弯拐点
  EXPECT_LT(MinDistanceTo(path, 0.0, 2.0), 1e-6);
  EXPECT_LT(MinDistanceTo(path, 2.0, 2.0), 1e-6);
}

// ---------------------------------------------------------------------------
// 用例 3：双向边反向遍历 —— 边定义为 from B to A，请求 A 到 B，
// polyline 必须被逆序展开。
// ---------------------------------------------------------------------------
TEST(SkeletonPlannerTest, BidirectionalEdge_ReverseTraversalReversesPolyline) {
  catalog::StationCatalog catalog;
  catalog.stations.push_back(MakeStation("A", 0.0, 0.0));
  catalog.stations.push_back(MakeStation("B", 5.0, 5.0));
  catalog.edges.push_back(MakeEdge(
      "B", "A", std::hypot(5.0, 5.0),
      {{5.0, 5.0}, {3.0, 3.0}, {0.0, 0.0}}, /*bidirectional=*/true));

  SkeletonPlanner planner(std::move(catalog));
  auto path = planner.PlanBetweenNodes("A", "B");
  ASSERT_FALSE(path.empty());

  // 正向展开 (B->A) 的话起点会是 (5,5)；逆序展开才应从 A 出发
  EXPECT_NEAR(path.front().x, 0.0, kEps);
  EXPECT_NEAR(path.front().y, 0.0, kEps);
  EXPECT_NEAR(path.back().x, 5.0, kEps);
  EXPECT_NEAR(path.back().y, 5.0, kEps);
  // 中途拐点 (3,3) 应被经过（等距采样步长 0.1m）
  EXPECT_LT(MinDistanceTo(path, 3.0, 3.0), 0.1);
}

// ---------------------------------------------------------------------------
// 用例 4：同一节点直接到达（from == to）。
// ---------------------------------------------------------------------------
TEST(SkeletonPlannerTest, SameNode_ReturnsSinglePose) {
  SkeletonPlanner planner(MakeLCorridorCatalog());

  auto path = planner.PlanBetweenNodes("A", "A");
  ASSERT_EQ(path.size(), 1u);
  EXPECT_NEAR(path.front().x, 0.0, kEps);
  EXPECT_NEAR(path.front().y, 0.0, kEps);
}

// ---------------------------------------------------------------------------
// 用例 5：无连通路径时返回空。
// ---------------------------------------------------------------------------
TEST(SkeletonPlannerTest, DisconnectedNodes_ReturnEmptyPath) {
  catalog::StationCatalog catalog;
  catalog.stations.push_back(MakeStation("A", 0.0, 0.0));
  catalog.stations.push_back(MakeStation("B", 3.0, 4.0));
  // 两条互不连通的边：A 只连 J1，B 只连 J2，A 与 B 之间无路径
  catalog.road_nodes.push_back(MakeRoadNode("J1", 1.0, 0.0));
  catalog.road_nodes.push_back(MakeRoadNode("J2", 4.0, 4.0));
  catalog.edges.push_back(MakeEdge("A", "J1", 1.0, {{0.0, 0.0}, {1.0, 0.0}}));
  catalog.edges.push_back(MakeEdge("B", "J2", 1.0, {{3.0, 4.0}, {4.0, 4.0}}));

  SkeletonPlanner planner(std::move(catalog));
  EXPECT_TRUE(planner.PlanBetweenNodes("A", "B").empty());
  EXPECT_TRUE(
      planner.Plan(Pose2D{0.0, 0.0, 0.0}, Pose2D{3.0, 4.0, 0.0}).empty());
}

// ---------------------------------------------------------------------------
// 用例 6：边缺少 polyline 时退化为两节点坐标间的直线。
// ---------------------------------------------------------------------------
TEST(SkeletonPlannerTest, EdgeWithoutPolyline_FallsBackToStraightLine) {
  catalog::StationCatalog catalog;
  catalog.stations.push_back(MakeStation("A", 0.0, 0.0));
  catalog.stations.push_back(MakeStation("B", 3.0, 4.0));
  catalog.edges.push_back(MakeEdge("A", "B", 5.0, /*polyline=*/{}));

  SkeletonPlanner planner(std::move(catalog));
  auto path = planner.PlanBetweenNodes("A", "B");
  ASSERT_FALSE(path.empty());
  EXPECT_NEAR(path.front().x, 0.0, kEps);
  EXPECT_NEAR(path.front().y, 0.0, kEps);
  EXPECT_NEAR(path.back().x, 3.0, kEps);
  EXPECT_NEAR(path.back().y, 4.0, kEps);
  // 共线校验：到直线 AB 的距离都应接近 0
  for (const auto& pose : path) {
    const double cross =
        std::fabs(4.0 * pose.x - 3.0 * pose.y);  // |cross((3,4),(x,y))| / 5
    EXPECT_LT(cross / 5.0, kEps);
  }
}

// ---------------------------------------------------------------------------
// 用例 7：Plan 首尾接驳直线段 —— start 不在节点上时拼接 start->N_start，
// goal 不在节点上时拼接 N_goal->goal，且首尾路点严格等于 start/goal。
// ---------------------------------------------------------------------------
TEST(SkeletonPlannerTest, Plan_OffNodeEndpoints_SpliceAccessSegments) {
  SkeletonPlanner planner(MakeLCorridorCatalog());

  const Pose2D start{-1.0, 0.0, 0.0};
  const Pose2D goal{6.0, 5.0, 1.5707};
  auto path = planner.Plan(start, goal);
  ASSERT_FALSE(path.empty());

  EXPECT_NEAR(path.front().x, start.x, kEps);
  EXPECT_NEAR(path.front().y, start.y, kEps);
  EXPECT_NEAR(path.back().x, goal.x, kEps);
  EXPECT_NEAR(path.back().y, goal.y, kEps);
  EXPECT_NEAR(path.back().theta, goal.theta, kEps);

  for (const auto& pose : path) {
    const bool on_access_y0 = std::fabs(pose.y) < kEps && pose.x >= -1.0 - kEps &&
                              pose.x <= 0.0 + kEps;
    const bool on_vertical = std::fabs(pose.x) < kEps && pose.y >= -kEps &&
                             pose.y <= 5.0 + kEps;
    const bool on_horizontal = std::fabs(pose.y - 5.0) < kEps &&
                               pose.x >= -kEps && pose.x <= 6.0 + kEps;
    EXPECT_TRUE(on_access_y0 || on_vertical || on_horizontal)
        << "waypoint (" << pose.x << ", " << pose.y
        << ") left the L corridor with access segments";
  }
}

// ---------------------------------------------------------------------------
// 用例 8：等距步长与航向角 —— 相邻路点间距不超过 step_size，
// 航向角与运动方向一致。
// ---------------------------------------------------------------------------
TEST(SkeletonPlannerTest, Resample_StepSizeBoundedAndHeadingConsistent) {
  SkeletonPlanner planner(MakeLCorridorCatalog());

  auto path = planner.Plan(Pose2D{0.0, 0.0, 0.0}, Pose2D{5.0, 5.0, 0.0});
  ASSERT_GE(path.size(), 100u);
  for (size_t i = 1; i < path.size(); ++i) {
    const double spacing =
        std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
    EXPECT_LE(spacing, 0.1 + kEps);
  }
  // 首点保持起点、末点保持终点及其航向
  EXPECT_NEAR(path.front().x, 0.0, kEps);
  EXPECT_NEAR(path.back().x, 5.0, kEps);
  EXPECT_NEAR(path.back().theta, 0.0, kEps);
}

}  // namespace
}  // namespace amr_dispatcher_core::planning
