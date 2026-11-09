#include "amr_dispatcher_core/workflow/in_flight_reroute_planning.hpp"
#include "amr_dispatcher_core/catalog/station_catalog.hpp"

#include <gtest/gtest.h>

using amr_dispatcher_core::catalog::CatalogStationEdge;
using amr_dispatcher_core::catalog::Station;
using amr_dispatcher_core::catalog::StationCatalog;
using amr_dispatcher_core::workflow::InFlightRerouteDecision;
using amr_dispatcher_core::workflow::InFlightRerouteRequest;
using amr_dispatcher_core::workflow::PlanInFlightReroute;

namespace {

StationCatalog BuildLoopCatalog() {
  StationCatalog catalog;
  catalog.stations = {
      Station{"A", "map", 0.0, 0.0, 0.0},
      Station{"B", "map", 5.0, 5.0, 0.0},
      Station{"C", "map", 5.0, -5.0, 0.0},
      Station{"D", "map", 10.0, 0.0, 0.0},
  };
  catalog.edges = {
      CatalogStationEdge{"A", "B", 7.0, true, true, {}},
      CatalogStationEdge{"B", "D", 7.0, true, true, {}},
      CatalogStationEdge{"A", "C", 7.0, true, true, {}},
      CatalogStationEdge{"C", "D", 7.0, true, true, {}},
  };
  return catalog;
}

}  // namespace

TEST(InFlightReroutePlanningTest, BlockedEdgeTriggersAlternativeBypass) {
  const auto catalog = BuildLoopCatalog();

  InFlightRerouteRequest req;
  req.catalog = &catalog;
  req.mission_id = "m1";
  req.vehicle_id = "amr_01";
  req.current_safe_node = "A";
  req.destination_node = "D";
  req.full_original_path = {"A", "B", "D"};
  // 原持锁：已占有 A、edge_AB、B、edge_BD、D
  req.current_held_locks = {
      "route_node:A",
      "route_edge:A__B",
      "route_node:B",
      "route_edge:B__D",
      "route_node:D",
  };
  // 边 A-B 遇到障碍被封锁
  req.blocked_edge_keys = {"route_edge:A__B"};

  const auto decision = PlanInFlightReroute(req);
  ASSERT_EQ(decision.outcome, InFlightRerouteDecision::Outcome::kRerouteReady);
  EXPECT_EQ(decision.new_route_path, (std::vector<std::string>{"A", "C", "D"}));

  // 新需申请的锁：C、edge_AC、edge_CD（A 和 D 已经持有）
  EXPECT_NE(std::find(decision.new_required_locks.begin(), decision.new_required_locks.end(), "route_node:C"),
            decision.new_required_locks.end());
  EXPECT_NE(std::find(decision.new_required_locks.begin(), decision.new_required_locks.end(), "route_edge:A__C"),
            decision.new_required_locks.end());
  EXPECT_NE(std::find(decision.new_required_locks.begin(), decision.new_required_locks.end(), "route_edge:C__D"),
            decision.new_required_locks.end());

  // 待释放的废弃锁：原未来的 edge_AB、B、edge_BD（注意：当前所在节点 A 绝对不能释放！）
  EXPECT_EQ(std::find(decision.obsolete_locks.begin(), decision.obsolete_locks.end(), "route_node:A"),
            decision.obsolete_locks.end());
  EXPECT_NE(std::find(decision.obsolete_locks.begin(), decision.obsolete_locks.end(), "route_edge:A__B"),
            decision.obsolete_locks.end());
  EXPECT_NE(std::find(decision.obsolete_locks.begin(), decision.obsolete_locks.end(), "route_node:B"),
            decision.obsolete_locks.end());
  EXPECT_NE(std::find(decision.obsolete_locks.begin(), decision.obsolete_locks.end(), "route_edge:B__D"),
            decision.obsolete_locks.end());
}

TEST(InFlightReroutePlanningTest, AllRoutesBlockedReturnsNoAlternative) {
  const auto catalog = BuildLoopCatalog();

  InFlightRerouteRequest req;
  req.catalog = &catalog;
  req.mission_id = "m1";
  req.vehicle_id = "amr_01";
  req.current_safe_node = "A";
  req.destination_node = "D";
  req.full_original_path = {"A", "B", "D"};
  req.current_held_locks = {"route_node:A", "route_edge:A__B", "route_node:B"};
  // 两条路全部封死
  req.blocked_edge_keys = {"route_edge:A__B", "route_edge:A__C"};

  const auto decision = PlanInFlightReroute(req);
  EXPECT_EQ(decision.outcome, InFlightRerouteDecision::Outcome::kNoAlternative);
  EXPECT_TRUE(decision.new_route_path.empty());
  EXPECT_TRUE(decision.new_required_locks.empty());
  EXPECT_TRUE(decision.obsolete_locks.empty());
}

TEST(InFlightReroutePlanningTest, AlreadyAtDestinationNoop) {
  const auto catalog = BuildLoopCatalog();

  InFlightRerouteRequest req;
  req.catalog = &catalog;
  req.mission_id = "m1";
  req.vehicle_id = "amr_01";
  req.current_safe_node = "D";
  req.destination_node = "D";
  req.full_original_path = {"A", "B", "D"};

  const auto decision = PlanInFlightReroute(req);
  EXPECT_EQ(decision.outcome, InFlightRerouteDecision::Outcome::kAlreadyAtGoal);
}
