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

TEST(StationTopologyTest, DisconnectedGraphReturnsNullopt) {
  StationTopologyGraph graph;
  graph.AddStation({"A", 0.0, 0.0});
  graph.AddStation({"Isolated", 100.0, 100.0});

  EXPECT_FALSE(graph.FindShortestPath("A", "Isolated").has_value());
  EXPECT_FALSE(graph.EstimatePathDistance("A", "Isolated").has_value());
}
