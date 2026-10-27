#include "amr_dispatcher_core/planning/prioritized_planning.hpp"
#include "amr_dispatcher_core/catalog/station_catalog.hpp"

#include <gtest/gtest.h>

using amr_dispatcher_core::catalog::CatalogStationEdge;
using amr_dispatcher_core::catalog::Station;
using amr_dispatcher_core::catalog::StationCatalog;
using amr_dispatcher_core::planning::CBSAgentSpec;
using amr_dispatcher_core::planning::DetectFirstConflict;
using amr_dispatcher_core::planning::PlanPrioritizedPaths;
using amr_dispatcher_core::planning::PrioritizedRequest;

namespace {

StationCatalog BuildParallelBypassCatalog() {
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

StationCatalog BuildCrossIntersectionCatalog() {
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
  return cat;
}

}  // namespace

TEST(PrioritizedPlanningTest, OppositeTrafficResolvesByPrioritizedDetour) {
  auto cat = BuildParallelBypassCatalog();
  PrioritizedRequest req;
  req.travel_speed_mps = 1.0;
  req.time_step_duration_sec = 1.0;

  // amr1 从 A 走到 C；amr2 从 C 走到 A
  req.agents.push_back(CBSAgentSpec{"amr1", "A", "C", 0, 0.0});
  req.agents.push_back(CBSAgentSpec{"amr2", "C", "A", 0, 0.0});

  auto res = PlanPrioritizedPaths(req, cat);
  EXPECT_TRUE(res.success);
  EXPECT_TRUE(res.verified_conflict_free);
  EXPECT_EQ(res.solution.size(), 2);

  // 保证无任何两两冲突
  EXPECT_FALSE(DetectFirstConflict(res.solution).has_value());
}

TEST(PrioritizedPlanningTest, CrossIntersectionConflictResolvedByWaitInPlace) {
  auto cat = BuildCrossIntersectionCatalog();
  PrioritizedRequest req;
  req.travel_speed_mps = 1.0;
  req.time_step_duration_sec = 1.0;

  req.agents.push_back(CBSAgentSpec{"amrX", "X1", "X2", 0, 0.0});
  req.agents.push_back(CBSAgentSpec{"amrY", "Y1", "Y2", 0, 0.0});

  auto res = PlanPrioritizedPaths(req, cat);
  EXPECT_TRUE(res.success);
  EXPECT_TRUE(res.verified_conflict_free);
  EXPECT_EQ(res.solution.size(), 2);
  EXPECT_FALSE(DetectFirstConflict(res.solution).has_value());

  // 后走的智能体必须发生了等待以让行先走的高优先级车辆
  bool had_wait = false;
  for (const auto& [agent, path] : res.solution) {
    for (const auto& pt : path) {
      if (pt.is_wait) had_wait = true;
    }
  }
  EXPECT_TRUE(had_wait);
}

TEST(PrioritizedPlanningTest, GoalHoldPreventsSubsequentAgentsPassThrough) {
  // A --- B
  StationCatalog cat;
  cat.stations.push_back(Station{"A", "A", 0.0, 0.0, 0.0, {}});
  cat.stations.push_back(Station{"B", "B", 10.0, 0.0, 0.0, {}});
  cat.edges.push_back(CatalogStationEdge{"A", "B", 10.0, true, true, {}});

  PrioritizedRequest req;
  req.travel_speed_mps = 1.0;
  req.time_step_duration_sec = 1.0;
  req.options.hold_agents_at_goal = true;

  // amr1 到达 B 停驻；amr2 也试图去终点 B (当 B 只能容纳 1 车时)
  req.agents.push_back(CBSAgentSpec{"amr1", "A", "B", 0, 0.0});
  req.agents.push_back(CBSAgentSpec{"amr2", "A", "B", 5, 5.0});

  auto res = PlanPrioritizedPaths(req, cat);
  // 由于 B 被 amr1 停驻独占，amr2 无法在无冲突情况下进驻 B 站，应如实判定无法规划而不是穿透发生冲突
  EXPECT_FALSE(res.success);
}
