#include "amr_dispatcher_core/dispatcher/station_topology.hpp"

#include <gtest/gtest.h>

using namespace amr_dispatcher_core::dispatcher;

TEST(StationTopologyTest, BasicDijkstraPathSearch) {
  StationTopologyGraph graph;
  graph.AddStation({"A", 0.0, 0.0, 0.0, "pickup"});
  graph.AddStation({"B", 10.0, 0.0, 0.0, "normal"});
  graph.AddStation({"C", 20.0, 0.0, 0.0, "dropoff"});

  graph.AddEdge({"A", "B", 10.0, true});
  graph.AddEdge({"B", "C", 10.0, true});

  auto path = graph.FindShortestPath("A", "C");
  ASSERT_TRUE(path.has_value());
  ASSERT_EQ(path->size(), 3u);
  EXPECT_EQ((*path)[0], "A");
  EXPECT_EQ((*path)[1], "B");
  EXPECT_EQ((*path)[2], "C");

  auto dist = graph.EstimatePathDistance("A", "C");
  ASSERT_TRUE(dist.has_value());
  EXPECT_DOUBLE_EQ(*dist, 20.0);
}

TEST(StationTopologyTest, ChoosesShorterAlternativeRoute) {
  StationTopologyGraph graph;
  graph.AddStation({"A", 0.0, 0.0});
  graph.AddStation({"B", 5.0, 10.0});
  graph.AddStation({"C", 10.0, 0.0});

  // A -> B -> C (距离 5 + 5 = 10)
  graph.AddEdge({"A", "B", 5.0, true});
  graph.AddEdge({"B", "C", 5.0, true});

  // A -> C 直达慢速边 (距离 15)
  graph.AddEdge({"A", "C", 15.0, true});

  auto path = graph.FindShortestPath("A", "C");
  ASSERT_TRUE(path.has_value());
  EXPECT_EQ(path->size(), 3u);
  EXPECT_EQ((*path)[1], "B");

  auto dist = graph.EstimatePathDistance("A", "C");
  EXPECT_DOUBLE_EQ(*dist, 10.0);
}

TEST(StationTopologyTest, DynamicReroutingBypassesDeadlockEdge) {
  StationTopologyGraph graph;
  graph.AddStation({"A", 0.0, 0.0});
  graph.AddStation({"B", 10.0, 0.0});
  graph.AddStation({"C", 20.0, 0.0});
  graph.AddStation({"Detour", 10.0, 10.0});

  // 主干道: A <-> B <-> C (距离 10 + 10 = 20)
  graph.AddEdge({"A", "B", 10.0, true});
  graph.AddEdge({"B", "C", 10.0, true});

  // 绕行支路: A <-> Detour <-> C (距离 12 + 12 = 24)
  graph.AddEdge({"A", "Detour", 12.0, true});
  graph.AddEdge({"Detour", "C", 12.0, true});

  // 1. 常规情况下走主干道: A -> B -> C
  auto normal_path = graph.FindShortestPath("A", "C");
  ASSERT_TRUE(normal_path.has_value());
  EXPECT_EQ((*normal_path)[1], "B");

  // 2. 模拟主干道 B__C 发生拥堵/死锁，动态封锁边 "route_edge:B__C"
  std::unordered_set<std::string> blocked_edges{"route_edge:B__C"};
  auto rerouted_path = graph.FindShortestPathAvoiding("A", "C", blocked_edges);

  // 3. 校验算法自动触发 Dynamic Re-routing，绕道 Detour 顺利到达 C！
  ASSERT_TRUE(rerouted_path.has_value());
  ASSERT_EQ(rerouted_path->size(), 3u);
  EXPECT_EQ((*rerouted_path)[0], "A");
  EXPECT_EQ((*rerouted_path)[1], "Detour");
  EXPECT_EQ((*rerouted_path)[2], "C");
}

TEST(StationTopologyTest, DisconnectedGraphReturnsNullopt) {
  StationTopologyGraph graph;
  graph.AddStation({"A", 0.0, 0.0});
  graph.AddStation({"B", 10.0, 0.0});
  EXPECT_FALSE(graph.FindShortestPath("A", "B").has_value());
}
