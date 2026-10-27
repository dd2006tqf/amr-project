#include "amr_dispatcher_core/workflow/cbs_traffic_coordinator.hpp"
#include "amr_dispatcher_core/catalog/station_catalog.hpp"

#include <gtest/gtest.h>

using amr_dispatcher_core::catalog::CatalogStationEdge;
using amr_dispatcher_core::catalog::Station;
using amr_dispatcher_core::catalog::StationCatalog;
using amr_dispatcher_core::workflow::CBSMissionRequest;
using amr_dispatcher_core::workflow::MotionTimingConfig;
using amr_dispatcher_core::workflow::PlanCBSMultiMissionCoordination;
using amr_dispatcher_core::workflow::TimeSpaceOccupancy;

namespace {

StationCatalog BuildDualCorridorCatalog() {
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

TEST(CBSTrafficCoordinatorTest, PlanCoordinatedMissionsOutputsWindows) {
  auto cat = BuildDualCorridorCatalog();
  TimeSpaceOccupancy occ;
  MotionTimingConfig cfg;
  cfg.cruise_speed_mps = 1.0;
  auto now = std::chrono::steady_clock::now();

  std::vector<CBSMissionRequest> reqs;
  reqs.push_back(CBSMissionRequest{"m1", "amr1", "A", "C", now});
  reqs.push_back(CBSMissionRequest{"m2", "amr2", "C", "A", now});

  auto res = PlanCBSMultiMissionCoordination(reqs, cat, occ, cfg, now);
  EXPECT_TRUE(res.all_solved);
  EXPECT_EQ(res.mission_plans.size(), 2);

  // m1 与 m2 均成功规划，并产出各自的无冲突 TimeWindows
  ASSERT_TRUE(res.mission_plans["m1"].success);
  ASSERT_TRUE(res.mission_plans["m2"].success);
  EXPECT_FALSE(res.mission_plans["m1"].time_windows.empty());
  EXPECT_FALSE(res.mission_plans["m2"].time_windows.empty());

  // 验证两任务路径不重合 (走不同的通道 B1 与 B2)
  const auto& p1 = res.mission_plans["m1"].path_nodes;
  const auto& p2 = res.mission_plans["m2"].path_nodes;
  EXPECT_NE(p1, p2);
}
