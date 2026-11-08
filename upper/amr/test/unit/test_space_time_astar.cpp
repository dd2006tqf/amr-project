#include "amr_dispatcher_core/planning/space_time_astar.hpp"
#include "amr_dispatcher_core/catalog/station_catalog.hpp"

#include <gtest/gtest.h>

using amr_dispatcher_core::catalog::CatalogStationEdge;
using amr_dispatcher_core::catalog::Station;
using amr_dispatcher_core::catalog::StationCatalog;
using amr_dispatcher_core::planning::EdgeConstraint;
using amr_dispatcher_core::planning::PlanSpaceTimeAStar;
using amr_dispatcher_core::planning::SpaceTimeAStarRequest;
using amr_dispatcher_core::planning::VertexConstraint;

namespace {

StationCatalog BuildTestCatalog() {
  // A --- B --- C
  //       |
  //       D
  StationCatalog cat;
  cat.stations.push_back(Station{"A", "A", 0.0, 0.0, 0.0, {}});
  cat.stations.push_back(Station{"B", "B", 10.0, 0.0, 0.0, {}});
  cat.stations.push_back(Station{"C", "C", 20.0, 0.0, 0.0, {}});
  cat.stations.push_back(Station{"D", "D", 10.0, 10.0, 0.0, {}});

  // A <-> B (10m)
  cat.edges.push_back(CatalogStationEdge{"A", "B", 10.0, true, true, {}});
  // B <-> C (10m)
  cat.edges.push_back(CatalogStationEdge{"B", "C", 10.0, true, true, {}});
  // B <-> D (10m)
  cat.edges.push_back(CatalogStationEdge{"B", "D", 10.0, true, true, {}});

  return cat;
}

}  // namespace

TEST(SpaceTimeAStarTest, DirectUnconstrainedPath) {
  auto cat = BuildTestCatalog();
  SpaceTimeAStarRequest req;
  req.agent_id = "amr1";
  req.start_station = "A";
  req.goal_station = "C";
  req.travel_speed_mps = 1.0;
  req.time_step_duration_sec = 1.0;

  auto res = PlanSpaceTimeAStar(req, cat);
  EXPECT_TRUE(res.success);
  ASSERT_EQ(res.path.size(), 3);
  EXPECT_EQ(res.path[0].station_id, "A");
  EXPECT_EQ(res.path[0].time_step, 0);
  EXPECT_EQ(res.path[1].station_id, "B");
  EXPECT_EQ(res.path[1].time_step, 10);
  EXPECT_EQ(res.path[2].station_id, "C");
  EXPECT_EQ(res.path[2].time_step, 20);
}

TEST(SpaceTimeAStarTest, VertexConstraintForcesWaitInPlace) {
  auto cat = BuildTestCatalog();
  SpaceTimeAStarRequest req;
  req.agent_id = "amr1";
  req.start_station = "A";
  req.goal_station = "C";
  req.travel_speed_mps = 1.0;
  req.time_step_duration_sec = 1.0;

  // 在 t=10 时站点 B 被占用，阻断 amr1 准时入站
  req.vertex_constraints.push_back(VertexConstraint{"amr1", "B", 10});

  auto res = PlanSpaceTimeAStar(req, cat);
  EXPECT_TRUE(res.success);
  // 预期在 A 站等待至少 1 步 (t=1)，然后再出发在 t=11 到达 B
  ASSERT_GE(res.path.size(), 4);
  EXPECT_EQ(res.path[0].station_id, "A");
  EXPECT_EQ(res.path[0].time_step, 0);
  EXPECT_TRUE(res.path[1].is_wait);
  EXPECT_EQ(res.path[1].station_id, "A");
  EXPECT_EQ(res.path[1].time_step, 1);
  EXPECT_EQ(res.path[2].station_id, "B");
  EXPECT_EQ(res.path[2].time_step, 11);
  EXPECT_EQ(res.path[3].station_id, "C");
  EXPECT_EQ(res.path[3].time_step, 21);
}

TEST(SpaceTimeAStarTest, EdgeConstraintForcesWaitOrAlternative) {
  auto cat = BuildTestCatalog();
  SpaceTimeAStarRequest req;
  req.agent_id = "amr1";
  req.start_station = "A";
  req.goal_station = "C";
  req.travel_speed_mps = 1.0;
  req.time_step_duration_sec = 1.0;

  // 在 t=0 阻断边 A->B，必须等待
  req.edge_constraints.push_back(EdgeConstraint{"amr1", "A", "B", 0});

  auto res = PlanSpaceTimeAStar(req, cat);
  EXPECT_TRUE(res.success);
  ASSERT_GE(res.path.size(), 4);
  EXPECT_TRUE(res.path[1].is_wait);
  EXPECT_EQ(res.path[1].station_id, "A");
}
