#include "amr_dispatcher_core/planning/mapf_solver.hpp"
#include "amr_dispatcher_core/catalog/station_catalog.hpp"

#include <gtest/gtest.h>

using amr_dispatcher_core::catalog::CatalogStationEdge;
using amr_dispatcher_core::catalog::Station;
using amr_dispatcher_core::catalog::StationCatalog;
using amr_dispatcher_core::planning::CBSAgentSpec;
using amr_dispatcher_core::planning::MAPFPolicy;
using amr_dispatcher_core::planning::MAPFSolverMode;
using amr_dispatcher_core::planning::MAPFUnifiedRequest;
using amr_dispatcher_core::planning::SolveMAPF;

namespace {

StationCatalog BuildParallelBypassCatalog() {
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

TEST(MAPFSolverTest, AutoModeSolvesWithCBSWhenSmallFleet) {
  auto cat = BuildParallelBypassCatalog();
  MAPFUnifiedRequest req;
  req.travel_speed_mps = 1.0;
  req.time_step_duration_sec = 1.0;
  req.policy.mode = MAPFSolverMode::kAuto;

  req.agents.push_back(CBSAgentSpec{"amr1", "A", "C", 0, 0.0});
  req.agents.push_back(CBSAgentSpec{"amr2", "C", "A", 0, 0.0});

  auto res = SolveMAPF(req, cat);
  EXPECT_TRUE(res.success);
  EXPECT_TRUE(res.verified_conflict_free);
  EXPECT_EQ(res.solver_used, "cbs");
}

TEST(MAPFSolverTest, FallbackToPrioritizedWhenCBSExceedsBudget) {
  auto cat = BuildParallelBypassCatalog();
  MAPFUnifiedRequest req;
  req.travel_speed_mps = 1.0;
  req.time_step_duration_sec = 1.0;
  req.policy.mode = MAPFSolverMode::kAuto;
  req.policy.fallback_to_prioritized = true;
  // 故意将 CBS 搜索预算压缩到极限 (max_ct_nodes=1)，强行触发 CBS 预算耗尽
  req.policy.cbs_max_ct_nodes = 1;

  req.agents.push_back(CBSAgentSpec{"amr1", "A", "C", 0, 0.0});
  req.agents.push_back(CBSAgentSpec{"amr2", "C", "A", 0, 0.0});

  auto res = SolveMAPF(req, cat);
  EXPECT_TRUE(res.success);
  EXPECT_TRUE(res.verified_conflict_free);
  // 断言成功触发了降级
  EXPECT_EQ(res.solver_used, "cbs->prioritized");
  EXPECT_FALSE(res.fallback_reason.empty());
}
