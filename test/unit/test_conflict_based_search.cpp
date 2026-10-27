#include "amr_dispatcher_core/planning/conflict_based_search.hpp"
#include "amr_dispatcher_core/catalog/station_catalog.hpp"

#include <gtest/gtest.h>

using amr_dispatcher_core::catalog::CatalogStationEdge;
using amr_dispatcher_core::catalog::Station;
using amr_dispatcher_core::catalog::StationCatalog;
using amr_dispatcher_core::planning::CBSAgentSpec;
using amr_dispatcher_core::planning::CBSRequest;
using amr_dispatcher_core::planning::DetectFirstConflict;
using amr_dispatcher_core::planning::PlanConflictBasedSearch;

namespace {

StationCatalog BuildNarrowCorridorCatalog() {
  // A --- B --- C
  StationCatalog cat;
  cat.stations.push_back(Station{"A", "A", 0.0, 0.0, 0.0, {}});
  cat.stations.push_back(Station{"B", "B", 10.0, 0.0, 0.0, {}});
  cat.stations.push_back(Station{"C", "C", 20.0, 0.0, 0.0, {}});

  cat.edges.push_back(CatalogStationEdge{"A", "B", 10.0, true, true, {}});
  cat.edges.push_back(CatalogStationEdge{"B", "C", 10.0, true, true, {}});

  return cat;
}

StationCatalog BuildParallelCorridorCatalog() {
  //       B1
  //     /    \
  //   A        C
  //     \    /
  //       B2
  StationCatalog cat;
  cat.stations.push_back(Station{"A", "A", 0.0, 0.0, 0.0, {}});
  cat.stations.push_back(Station{"B1", "B1", 10.0, 5.0, 0.0, {}});
  cat.stations.push_back(Station{"B2", "B2", 10.0, -5.0, 0.0, {}});
  cat.stations.push_back(Station{"C", "C", 20.0, 0.0, 0.0, {}});

  cat.edges.push_back(CatalogStationEdge{"A", "B1", 10.0, true, true, {}});
  cat.edges.push_back(CatalogStationEdge{"B1", "C", 10.0, true, true, {}});
  cat.edges.push_back(CatalogStationEdge{"A", "B2", 10.0, true, true, {}});
  cat.edges.push_back(CatalogStationEdge{"B2", "C", 10.0, true, true, {}});
  return cat;
}

}  // namespace

TEST(ConflictBasedSearchTest, NonConflictingAgentsSolveDirectly) {
  auto cat = BuildNarrowCorridorCatalog();
  CBSRequest req;
  req.travel_speed_mps = 1.0;
  req.time_step_duration_sec = 1.0;

  // amr1 从 A 走到 B，amr2 从 B 走到 C (时空错峰)
  req.agents.push_back(CBSAgentSpec{"amr1", "A", "B", 0, 0.0});
  req.agents.push_back(CBSAgentSpec{"amr2", "B", "C", 20, 20.0});

  auto res = PlanConflictBasedSearch(req, cat);
  EXPECT_TRUE(res.success);
  EXPECT_EQ(res.solution.size(), 2);
  EXPECT_FALSE(DetectFirstConflict(res.solution).has_value());
}

TEST(ConflictBasedSearchTest, DualLaneBypassResolvesOppositeTraffic) {
  auto cat = BuildParallelCorridorCatalog();
  CBSRequest req;
  req.travel_speed_mps = 1.0;
  req.time_step_duration_sec = 1.0;

  // 两车对向：amr1 从 A 走向 C；amr2 从 C 走向 A
  req.agents.push_back(CBSAgentSpec{"amr1", "A", "C", 0, 0.0});
  req.agents.push_back(CBSAgentSpec{"amr2", "C", "A", 0, 0.0});

  auto res = PlanConflictBasedSearch(req, cat);
  EXPECT_TRUE(res.success);
  EXPECT_EQ(res.solution.size(), 2);

  // 关键断言：经过 CBS 协同后，绝对不存在任何顶点或对向碰撞冲突
  auto conflict = DetectFirstConflict(res.solution);
  EXPECT_FALSE(conflict.has_value());

  // 验证两车各自选择了不同的分流通道 (分别走 B1 与 B2)
  std::string p1_mid, p2_mid;
  for (const auto& pt : res.solution["amr1"]) {
    if (pt.station_id == "B1" || pt.station_id == "B2") p1_mid = pt.station_id;
  }
  for (const auto& pt : res.solution["amr2"]) {
    if (pt.station_id == "B1" || pt.station_id == "B2") p2_mid = pt.station_id;
  }
  EXPECT_FALSE(p1_mid.empty());
  EXPECT_FALSE(p2_mid.empty());
  EXPECT_NE(p1_mid, p2_mid);
}

TEST(ConflictBasedSearchTest, CrossIntersectionConflictResolvedByWait) {
  // 十字路口交叉穿行:
  //      Y1
  //       |
  // X1 - Mid - X2
  //       |
  //      Y2
  StationCatalog cat;
  cat.stations.push_back(Station{"X1", "X1", 0.0, 10.0, 0.0, {}});
  cat.stations.push_back(Station{"Mid", "Mid", 10.0, 10.0, 0.0, {}});
  cat.stations.push_back(Station{"X2", "X2", 20.0, 10.0, 0.0, {}});
  cat.stations.push_back(Station{"Y1", "Y1", 10.0, 0.0, 0.0, {}});
  cat.stations.push_back(Station{"Y2", "Y2", 10.0, 20.0, 0.0, {}});

  cat.edges.push_back(CatalogStationEdge{"X1", "Mid", 10.0, true, true, {}});
  cat.edges.push_back(CatalogStationEdge{"Mid", "X2", 10.0, true, true, {}});
  cat.edges.push_back(CatalogStationEdge{"Y1", "Mid", 10.0, true, true, {}});
  cat.edges.push_back(CatalogStationEdge{"Mid", "Y2", 10.0, true, true, {}});

  CBSRequest req;
  req.travel_speed_mps = 1.0;
  req.time_step_duration_sec = 1.0;

  // amrX: X1 -> X2 (经 Mid)
  // amrY: Y1 -> Y2 (经 Mid)
  // 两车同时出发，将在 t=10 在 Mid 发生十字路口顶点冲突
  req.agents.push_back(CBSAgentSpec{"amrX", "X1", "X2", 0, 0.0});
  req.agents.push_back(CBSAgentSpec{"amrY", "Y1", "Y2", 0, 0.0});

  auto res = PlanConflictBasedSearch(req, cat);
  EXPECT_TRUE(res.success);
  EXPECT_EQ(res.solution.size(), 2);
  EXPECT_FALSE(DetectFirstConflict(res.solution).has_value());

  // 必须有一台车在进路口前做了等待
  bool had_wait = false;
  for (const auto& [agent, path] : res.solution) {
    for (const auto& pt : path) {
      if (pt.is_wait) had_wait = true;
    }
  }
  EXPECT_TRUE(had_wait);
}
