// 拓扑连通性分析的验收。
//
// 核心命题：封一条边可能在局部看不出问题，却把一片站点变成孤岛。
// 这里的用例就是"提交前必须能拒绝的四种图形"。
#include "amr_dispatcher_core/catalog/topology_connectivity.hpp"

#include <gtest/gtest.h>

namespace amr_dispatcher_core::catalog {
namespace {

StationCatalog MakeRing() {
  StationCatalog catalog;
  catalog.map_hash = "h";
  catalog.stations.push_back(Station{"s1", "map", 0.0, 0.0, 0.0});
  catalog.stations.push_back(Station{"s2", "map", 1.0, 0.0, 0.0});
  catalog.stations.push_back(Station{"s3", "map", 1.0, 1.0, 0.0});
  catalog.edges.push_back(CatalogStationEdge{"s1", "s2", 1.0, true, true});
  catalog.edges.push_back(CatalogStationEdge{"s2", "s3", 1.0, true, true});
  catalog.edges.push_back(CatalogStationEdge{"s3", "s1", 1.0, true, true});
  return catalog;
}

}  // namespace

TEST(TopologyConnectivityTest, ConnectedGraphIsReachable) {
  const auto report = AnalyzeTopologyConnectivity(MakeRing());
  EXPECT_TRUE(report.all_stations_reachable) << report.message;
  EXPECT_TRUE(report.unreachable_stations.empty());
}

// 真正的桥边被封 = 图裂成两半。这是现场最容易犯的错：以为只是关了一条通道。
// 注意不能用三角环构造：环里封一条边三个站点仍互通，那测不出任何东西。
TEST(TopologyConnectivityTest, BlockedBridgeSplitsComponents) {
  StationCatalog catalog;
  catalog.map_hash = "h";
  catalog.stations.push_back(Station{"s1", "map", 0.0, 0.0, 0.0});
  catalog.stations.push_back(Station{"s2", "map", 1.0, 0.0, 0.0});
  // leaf 只有 s2 这一条通路
  catalog.stations.push_back(Station{"leaf", "map", 2.0, 0.0, 0.0});
  catalog.edges.push_back(CatalogStationEdge{"s1", "s2", 1.0, true, true});
  catalog.edges.push_back(CatalogStationEdge{"s2", "leaf", 1.0, true, true});

  // 封桥前：全通
  EXPECT_TRUE(AnalyzeTopologyConnectivity(catalog).all_stations_reachable);

  catalog.edges[1].enabled = false;  // s2 - leaf 封掉（叠加层的形态）
  const auto report = AnalyzeTopologyConnectivity(catalog);
  EXPECT_FALSE(report.all_stations_reachable);
  ASSERT_EQ(report.unreachable_stations.size(), 1u);
  EXPECT_EQ(report.unreachable_stations.front(), "leaf");
  // 两个含站点的分量都要报告出来，运维才能看到"裂成了哪几块"
  EXPECT_GE(report.components.size(), 2u);
}

// 环上封一条边：三个站点仍可绕行，不应误判为断裂。
// （这条用例守住"别把可达性判得太严"—— 过严会让每一次合法封路都被拒绝。）
TEST(TopologyConnectivityTest, BlockingOneEdgeOfARingStaysConnected) {
  StationCatalog catalog = MakeRing();
  catalog.edges[1].enabled = false;  // s2 - s3
  const auto report = AnalyzeTopologyConnectivity(catalog);
  EXPECT_TRUE(report.all_stations_reachable) << report.message;
  EXPECT_TRUE(report.unreachable_stations.empty());
}

TEST(TopologyConnectivityTest, ReportsUnreachableStationNames) {
  StationCatalog catalog;
  catalog.stations.push_back(Station{"a", "map", 0.0, 0.0, 0.0});
  catalog.stations.push_back(Station{"b", "map", 1.0, 0.0, 0.0});
  catalog.stations.push_back(Station{"island", "map", 9.0, 9.0, 0.0});
  catalog.edges.push_back(CatalogStationEdge{"a", "b", 1.0, true, true});
  // island 没有任何边

  const auto report = AnalyzeTopologyConnectivity(catalog);
  EXPECT_FALSE(report.all_stations_reachable);
  ASSERT_EQ(report.unreachable_stations.size(), 1u);
  EXPECT_EQ(report.unreachable_stations.front(), "island");
  EXPECT_TRUE(report.message.find("island") != std::string::npos) << report.message;
}

// 单向环：每条边只能沿一个方向走，但整圈仍强连通 —— 不应误判为断裂。
TEST(TopologyConnectivityTest, OneWayRingStillReachable) {
  StationCatalog catalog;
  catalog.stations.push_back(Station{"s1", "map", 0.0, 0.0, 0.0});
  catalog.stations.push_back(Station{"s2", "map", 1.0, 0.0, 0.0});
  catalog.stations.push_back(Station{"s3", "map", 1.0, 1.0, 0.0});
  catalog.edges.push_back(CatalogStationEdge{"s1", "s2", 1.0, false, true});
  catalog.edges.push_back(CatalogStationEdge{"s2", "s3", 1.0, false, true});
  catalog.edges.push_back(CatalogStationEdge{"s3", "s1", 1.0, false, true});

  const auto report = AnalyzeTopologyConnectivity(catalog);
  EXPECT_TRUE(report.all_stations_reachable) << report.message;
}

// 单向链：能顺着走但不能回头，强连通判定必须判负（车开进去就出不来）。
TEST(TopologyConnectivityTest, OneWayChainIsNotStronglyConnected) {
  StationCatalog catalog;
  catalog.stations.push_back(Station{"s1", "map", 0.0, 0.0, 0.0});
  catalog.stations.push_back(Station{"s2", "map", 1.0, 0.0, 0.0});
  catalog.stations.push_back(Station{"s3", "map", 2.0, 0.0, 0.0});
  catalog.edges.push_back(CatalogStationEdge{"s1", "s2", 1.0, false, true});
  catalog.edges.push_back(CatalogStationEdge{"s2", "s3", 1.0, false, true});

  const auto report = AnalyzeTopologyConnectivity(catalog);
  EXPECT_FALSE(report.all_stations_reachable);
  EXPECT_FALSE(report.unreachable_stations.empty());
}

// 骨架提取常留短支路（路网节点死胡同）：不能因此拒绝整张图，只报警告。
TEST(TopologyConnectivityTest, DeadEndRoadNodeIsWarningOnly) {
  StationCatalog catalog = MakeRing();
  catalog.road_nodes.push_back(RoadNode{"stub", "map", 5.0, 5.0, 0.0});
  catalog.edges.push_back(CatalogStationEdge{"s1", "stub", 1.0, false, true});

  const auto report = AnalyzeTopologyConnectivity(catalog);
  EXPECT_TRUE(report.all_stations_reachable) << report.message;
  ASSERT_EQ(report.dead_end_road_nodes.size(), 1u);
  EXPECT_EQ(report.dead_end_road_nodes.front(), "stub");
  EXPECT_TRUE(report.message.find("dead-end") != std::string::npos);
}

TEST(TopologyConnectivityTest, EmptyCatalogIsNotReachable) {
  const auto report = AnalyzeTopologyConnectivity(StationCatalog{});
  EXPECT_FALSE(report.all_stations_reachable);
  EXPECT_EQ(report.message, "catalog has no stations");
}

// 混合图：站点与路网节点共用 id 空间，SCC 必须把它们放在同一张图里算。
TEST(TopologyConnectivityTest, RoadNodesParticipateInReachability) {
  StationCatalog catalog;
  catalog.stations.push_back(Station{"a", "map", 0.0, 0.0, 0.0});
  catalog.stations.push_back(Station{"b", "map", 4.0, 0.0, 0.0});
  catalog.road_nodes.push_back(RoadNode{"n1", "map", 2.0, 2.0, 0.0});
  // 唯一的通路要经过路网节点
  catalog.edges.push_back(CatalogStationEdge{"a", "n1", 2.0, true, true});
  catalog.edges.push_back(CatalogStationEdge{"n1", "b", 2.0, true, true});

  const auto report = AnalyzeTopologyConnectivity(catalog);
  EXPECT_TRUE(report.all_stations_reachable) << report.message;
}

// 大图压力：迭代式 Tarjan 不能爆栈（拓扑校验跑在服务回调里，栈溢出 = 节点崩溃）。
TEST(TopologyConnectivityTest, LargeChainDoesNotOverflowStack) {
  StationCatalog catalog;
  constexpr int kNodes = 20000;
  for (int i = 0; i < kNodes; ++i) {
    catalog.stations.push_back(
        Station{"s" + std::to_string(i), "map", static_cast<double>(i), 0.0, 0.0});
    if (i > 0) {
      catalog.edges.push_back(CatalogStationEdge{"s" + std::to_string(i - 1),
                                                 "s" + std::to_string(i), 1.0, true, true});
    }
  }
  const auto report = AnalyzeTopologyConnectivity(catalog);
  EXPECT_TRUE(report.all_stations_reachable) << report.message;
}

}  // namespace amr_dispatcher_core::catalog
