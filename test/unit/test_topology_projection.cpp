#include "amr_dispatcher_core/dispatcher/topology_projection.hpp"

#include <gtest/gtest.h>

#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace amr_dispatcher_core::dispatcher {

TEST(TopologyProjectionTest, EmptyInputs_ProducesEmptyProjection) {
  const auto projection = BuildTopologyProjection({}, {}, {});

  EXPECT_TRUE(projection.active_edges.empty());
  EXPECT_TRUE(projection.locked_edges.empty());
  EXPECT_TRUE(projection.edge_owners.empty());
  EXPECT_TRUE(projection.deadlocked_nodes.empty());
  EXPECT_FALSE(projection.deadlock_detected);
}

TEST(TopologyProjectionTest, Snapshot_SeparatesActiveMissionEdgesFromManualBlocks) {
  const std::vector<std::pair<std::string, std::string>> snapshot = {
      {"route_edge:A__B", "m1"},
      {"route_node:C", "m1"},
      {"route_edge:C__D", "traffic_block:maintenance"},
  };
  const std::unordered_set<std::string> active = {"m1"};

  const auto projection = BuildTopologyProjection(snapshot, active, {});

  // locked_edges/edge_owners 全量透传（Snapshot 已排序）
  ASSERT_EQ(projection.locked_edges.size(), 3U);
  EXPECT_EQ(projection.locked_edges[0], "route_edge:A__B");
  EXPECT_EQ(projection.locked_edges[1], "route_node:C");
  EXPECT_EQ(projection.locked_edges[2], "route_edge:C__D");
  ASSERT_EQ(projection.edge_owners.size(), 3U);
  EXPECT_EQ(projection.edge_owners[0], "m1");
  EXPECT_EQ(projection.edge_owners[1], "m1");
  EXPECT_EQ(projection.edge_owners[2], "traffic_block:maintenance");

  // active_edges 只保留任务持有的 route_edge: 前缀项
  ASSERT_EQ(projection.active_edges.size(), 1U);
  EXPECT_EQ(projection.active_edges[0], "route_edge:A__B");
  EXPECT_FALSE(projection.deadlock_detected);
}

TEST(TopologyProjectionTest, CyclePath_ResolvesStationIdsFromEdgesAndNodes) {
  const std::vector<std::vector<std::string>> cycles = {
      {"m1", "route_edge:A__B", "m2", "route_node:C", "m1"}};

  const auto projection = BuildTopologyProjection({}, {}, cycles);

  EXPECT_TRUE(projection.deadlock_detected);
  // mission id 不得混入；站点 id 去重且保持出现顺序
  const std::vector<std::string> expected = {"A", "B", "C"};
  EXPECT_EQ(projection.deadlocked_nodes, expected);
}

TEST(TopologyProjectionTest, CyclePathWithMissionIdsOnly_YieldsNoStationNodes) {
  // 环里全是 mission id（及不带冒号的近似前缀）：不得解析出站点，
  // 否则 visualizer 拿未知 id 画到原点的假警戒柱
  const std::vector<std::vector<std::string>> cycles = {
      {"m1", "m2", "route_node_unknown", "m1"}};

  const auto projection = BuildTopologyProjection({}, {}, cycles);

  EXPECT_TRUE(projection.deadlock_detected);
  EXPECT_TRUE(projection.deadlocked_nodes.empty());
}

}  // namespace amr_dispatcher_core::dispatcher
