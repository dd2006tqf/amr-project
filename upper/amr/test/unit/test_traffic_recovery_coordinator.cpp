#include "amr_dispatcher_core/workflow/traffic_recovery_coordinator.hpp"

#include <gtest/gtest.h>

using amr_dispatcher_core::catalog::CatalogStationEdge;
using amr_dispatcher_core::catalog::Station;
using amr_dispatcher_core::catalog::StationCatalog;
using amr_dispatcher_core::workflow::DeadlockRecoveryCandidate;
using amr_dispatcher_core::workflow::DeadlockRecoveryDecision;
using amr_dispatcher_core::workflow::DeadlockRecoveryRequest;
using amr_dispatcher_core::workflow::PlanDeadlockRecovery;

namespace {

StationCatalog BuildBypassGridCatalog() {
  // A <-> B (狭窄瓶颈)
  // 另有经 C 的绕道：A <-> C <-> B
  StationCatalog catalog;
  catalog.stations = {
      Station{"A", "map", 0.0, 0.0, 0.0},
      Station{"B", "map", 10.0, 0.0, 0.0},
      Station{"C", "map", 5.0, 5.0, 0.0},
  };
  catalog.edges = {
      CatalogStationEdge{"A", "B", 10.0, true, true, {}},
      CatalogStationEdge{"A", "C", 7.0, true, true, {}},
      CatalogStationEdge{"C", "B", 7.0, true, true, {}},
  };
  return catalog;
}

}  // namespace

TEST(TrafficRecoveryCoordinatorTest, PicksLowerPriorityVehicleToReroute) {
  const auto catalog = BuildBypassGridCatalog();

  // m1 从 A 到 B（高优先级 priority=10）
  DeadlockRecoveryCandidate m1;
  m1.mission_id = "m1";
  m1.vehicle_id = "amr_01";
  m1.priority = 10;
  m1.current_safe_node = "A";
  m1.destination_node = "B";
  m1.full_original_path = {"A", "B"};
  m1.current_held_locks = {"route_node:A", "route_edge:A__B"};

  // m2 从 B 到 A（低优先级 priority=1）
  DeadlockRecoveryCandidate m2;
  m2.mission_id = "m2";
  m2.vehicle_id = "amr_02";
  m2.priority = 1;
  m2.current_safe_node = "B";
  m2.destination_node = "A";
  m2.full_original_path = {"B", "A"};
  m2.current_held_locks = {"route_node:B", "route_edge:A__B"};

  DeadlockRecoveryRequest req;
  req.catalog = &catalog;
  req.cycle_mission_ids = {"m1", "m2"};
  req.cycle_blocked_edges = {"route_edge:A__B"};
  req.candidates = {m1, m2};

  const auto decision = PlanDeadlockRecovery(req);
  EXPECT_EQ(decision.action, DeadlockRecoveryDecision::Action::kRerouteMission);
  // 必须选取低优先级 m2 执行绕行
  EXPECT_EQ(decision.selected_mission_id, "m2");
  EXPECT_EQ(decision.reroute_decision.new_route_path, (std::vector<std::string>{"B", "C", "A"}));
}

TEST(TrafficRecoveryCoordinatorTest, NoBypassFallsBackToSafeStop) {
  // 单一窄道，无任何绕行通道
  StationCatalog deadend_catalog;
  deadend_catalog.stations = {Station{"A", "map", 0.0, 0.0, 0.0}, Station{"B", "map", 10.0, 0.0, 0.0}};
  deadend_catalog.edges = {CatalogStationEdge{"A", "B", 10.0, true, true, {}}};

  DeadlockRecoveryCandidate m1{"m1", "amr_01", 10, "A", "B", {"route_node:A", "route_edge:A__B"}, {"A", "B"}};
  DeadlockRecoveryCandidate m2{"m2", "amr_02", 1, "B", "A", {"route_node:B", "route_edge:A__B"}, {"B", "A"}};

  DeadlockRecoveryRequest req;
  req.catalog = &deadend_catalog;
  req.cycle_mission_ids = {"m1", "m2"};
  req.cycle_blocked_edges = {"route_edge:A__B"};
  req.candidates = {m1, m2};

  const auto decision = PlanDeadlockRecovery(req);
  EXPECT_EQ(decision.action, DeadlockRecoveryDecision::Action::kSafeStopSuspend);
}
