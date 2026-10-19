#include "amr_dispatcher_core/catalog/station_catalog.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cstdio>
#include <fstream>
#include <limits>
#include <thread>

namespace amr_dispatcher_core::workflow {
using namespace amr_dispatcher_core::catalog;
namespace {

TEST(StationCatalogTest, BuildStationListProjection_PreservesStationFields) {
  StationCatalog catalog;
  catalog.stations.push_back(Station{"receiving", "map", 1.0, 2.0, 0.25});
  catalog.stations.push_back(Station{"storage", "odom", -3.0, 4.5, -1.5});

  const auto projection = BuildStationListProjection(catalog);

  EXPECT_EQ(projection.station_ids, (std::vector<std::string>{"receiving", "storage"}));
  EXPECT_EQ(projection.frame_ids, (std::vector<std::string>{"map", "odom"}));
  EXPECT_EQ(projection.x, (std::vector<double>{1.0, -3.0}));
  EXPECT_EQ(projection.y, (std::vector<double>{2.0, 4.5}));
  EXPECT_EQ(projection.yaw, (std::vector<double>{0.25, -1.5}));
  EXPECT_EQ(projection.message, "loaded 2 station(s)");
}

TEST(StationCatalogTest, BuildStationRouteListProjection_PreservesRouteFields) {
  StationCatalog catalog;
  catalog.edges.push_back(CatalogStationEdge{"receiving", "storage", 5.5, true, true});
  catalog.edges.push_back(CatalogStationEdge{"storage", "dock", 2.0, false, false});

  const auto projection = BuildStationRouteListProjection(catalog);

  EXPECT_EQ(projection.from_station, (std::vector<std::string>{"receiving", "storage"}));
  EXPECT_EQ(projection.to_station, (std::vector<std::string>{"storage", "dock"}));
  EXPECT_EQ(projection.distance_m, (std::vector<double>{5.5, 2.0}));
  EXPECT_EQ(projection.bidirectional, (std::vector<bool>{true, false}));
  EXPECT_EQ(projection.enabled, (std::vector<bool>{true, false}));
  EXPECT_EQ(projection.message, "loaded 2 route edge(s)");
}

}  // namespace
}  // namespace amr_dispatcher_core::workflow

namespace {

std::string WriteCatalogTemp(const std::string& content) {
  std::string path = std::tmpnam(nullptr);
  std::ofstream out(path);
  out << content;
  return path;
}

constexpr const char* kTwoStations =
    "stations:\n"
    "  - id: s1\n    x: 1.0\n    y: 2.0\n"
    "  - id: s2\n    x: 3.0\n    y: 4.0\n";
constexpr const char* kThreeStations =
    "stations:\n"
    "  - id: s1\n    x: 1.0\n    y: 2.0\n"
    "  - id: s2\n    x: 3.0\n    y: 4.0\n"
    "  - id: s3\n    x: 5.0\n    y: 6.0\n";
constexpr const char* kDuplicateIds =
    "stations:\n"
    "  - id: s1\n    x: 1.0\n    y: 2.0\n"
    "  - id: s1\n    x: 3.0\n    y: 4.0\n";

}  // namespace

// 构造即完成初始加载（等价节点 on_configure 的一次性读取）。
TEST(StationCatalogReloaderTest, LoadsInitialCatalog) {
  auto p = WriteCatalogTemp(kTwoStations);
  amr_dispatcher_core::catalog::StationCatalogReloader reloader(p);
  EXPECT_TRUE(reloader.loaded());
  EXPECT_EQ(reloader.catalog().stations.size(), 2u);
  EXPECT_FALSE(reloader.Tick());  // mtime 未变，不触发重载
  std::remove(p.c_str());
}

// mtime 变化触发重载换新；schema 非法时保留旧目录不清空。
TEST(StationCatalogReloaderTest, ReloadsOnMtimeChangeAndKeepsOldOnInvalid) {
  auto p = WriteCatalogTemp(kTwoStations);
  amr_dispatcher_core::catalog::StationCatalogReloader reloader(p);
  ASSERT_TRUE(reloader.loaded());

  std::this_thread::sleep_for(std::chrono::milliseconds(20));  // 跨过文件系统 mtime 粒度
  {
    std::ofstream out(p, std::ios::trunc);
    out << kThreeStations;
  }
  EXPECT_TRUE(reloader.Tick());
  EXPECT_EQ(reloader.catalog().stations.size(), 3u);

  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  {
    std::ofstream out(p, std::ios::trunc);
    out << kDuplicateIds;
  }
  EXPECT_FALSE(reloader.Tick());                     // 校验失败，没有换新
  EXPECT_EQ(reloader.catalog().stations.size(), 3u); // 旧目录仍在
  EXPECT_FALSE(reloader.last_error().empty());
  std::remove(p.c_str());
}

// ---- Task 1：road_nodes / polyline / version 混合图扩展 ----

namespace amr_dispatcher_core::catalog {
namespace {

// 完整 schema：version 段 + 站点 + 通行节点 + flow/map 两种 polyline 写法
constexpr const char* kMixedGraphCatalog = R"(version:
  map_hash: "deadbeefcafe"
  graph_version: "v1_skeleton"

stations:
  - id: s1
    x: 0.0
    y: 0.0
    yaw: 0.0
  - id: s2
    x: 10.0
    y: 0.0
    yaw: 3.14159

road_nodes:
  - id: n1
    x: 5.0
    y: 0.0
    yaw: 1.5708
  - id: n2
    x: 5.0
    y: 5.0
    frame_id: map

edges:
  - from: s1
    to: n1
    distance_m: 5.0
    bidirectional: true
    enabled: true
    polyline:
      - [0.0, 0.0]
      - [5.0, 0.0]
  - from: n1
    to: n2
    distance_m: 5.0
    polyline:
      - x: 5.0
        y: 0.0
      - x: 5.0
        y: 5.0
  - from: n2
    to: s2
    distance_m: 7.071
    polyline:
      - [5.0, 5.0]
      - [10.0, 0.0]
)";

// road_node 与 station 撞 id：混合 id 空间不允许任何重复
constexpr const char* kCrossDuplicateIds =
    "stations:\n"
    "  - id: s1\n    x: 0.0\n    y: 0.0\n"
    "road_nodes:\n"
    "  - id: s1\n    x: 1.0\n    y: 1.0\n"
    "edges:\n"
    "  - from: s1\n    to: s1\n    distance_m: 1.0\n";

// M2：polyline 块之后仍跟正常 edge 字段（IsEdgeFieldKey 退出折线的路径）
constexpr const char* kEdgeFieldsAfterPolyline =
    "stations:\n"
    "  - id: s1\n    x: 0.0\n    y: 0.0\n"
    "  - id: s2\n    x: 1.5\n    y: 0.0\n"
    "edges:\n"
    "  - from: s1\n"
    "    to: s2\n"
    "    polyline:\n"
    "      - [0.0, 0.0]\n"
    "      - [1.5, 0.0]\n"
    "    distance_m: 3.0\n"
    "    bidirectional: true\n"
    "    enabled: true\n";

// I1：折线点损坏（- [broken]）必须让加载失败，而不是静默丢弃折线后报告成功
constexpr const char* kBrokenFlowPolylinePoint =
    "stations:\n"
    "  - id: s1\n    x: 0.0\n    y: 0.0\n"
    "  - id: s2\n    x: 1.0\n    y: 0.0\n"
    "edges:\n"
    "  - from: s1\n    to: s2\n    distance_m: 1.0\n"
    "    polyline:\n"
    "      - [broken]\n"
    "      - [1.0, 0.0]\n";

// I1：折线块内出现嵌套块序列（非 flow 点、非 map 点、非 edge 字段）→ 严格失败
constexpr const char* kNestedBlockSequencePolyline =
    "stations:\n"
    "  - id: s1\n    x: 0.0\n    y: 0.0\n"
    "  - id: s2\n    x: 1.0\n    y: 0.0\n"
    "edges:\n"
    "  - from: s1\n    to: s2\n    distance_m: 1.0\n"
    "    polyline:\n"
    "      -\n"
    "        - 1.5\n"
    "        - 2.0\n";

// I1：map 风格半截点（缺 y 分量）不静默丢弃 → 加载失败
constexpr const char* kIncompleteMapPolylinePoint =
    "stations:\n"
    "  - id: s1\n    x: 0.0\n    y: 0.0\n"
    "  - id: s2\n    x: 1.0\n    y: 0.0\n"
    "edges:\n"
    "  - from: s1\n    to: s2\n    distance_m: 1.0\n"
    "    polyline:\n"
    "      - x: 0.0\n"
    "        y: 0.0\n"
    "      - x: 1.0\n";

TEST(StationCatalogMixedGraphTest, LoadStationCatalog_ParsesVersionRoadNodesAndPolyline) {
  const auto path = WriteCatalogTemp(kMixedGraphCatalog);
  const auto catalog = LoadStationCatalog(path);
  std::remove(path.c_str());
  ASSERT_TRUE(catalog.has_value());

  EXPECT_EQ(catalog->map_hash, "deadbeefcafe");
  EXPECT_EQ(catalog->graph_version, "v1_skeleton");

  ASSERT_EQ(catalog->stations.size(), 2u);
  ASSERT_EQ(catalog->road_nodes.size(), 2u);
  EXPECT_EQ(catalog->road_nodes[0].id, "n1");
  EXPECT_DOUBLE_EQ(catalog->road_nodes[0].x, 5.0);
  EXPECT_DOUBLE_EQ(catalog->road_nodes[0].y, 0.0);
  EXPECT_DOUBLE_EQ(catalog->road_nodes[0].yaw, 1.5708);
  EXPECT_EQ(catalog->road_nodes[0].frame_id, "map");
  EXPECT_EQ(catalog->road_nodes[1].id, "n2");
  EXPECT_DOUBLE_EQ(catalog->road_nodes[1].x, 5.0);
  EXPECT_DOUBLE_EQ(catalog->road_nodes[1].y, 5.0);
  EXPECT_EQ(catalog->road_nodes[1].frame_id, "map");

  ASSERT_EQ(catalog->edges.size(), 3u);
  // flow 序列写法 - [x, y]
  ASSERT_EQ(catalog->edges[0].polyline.size(), 2u);
  EXPECT_DOUBLE_EQ(catalog->edges[0].polyline[0].first, 0.0);
  EXPECT_DOUBLE_EQ(catalog->edges[0].polyline[0].second, 0.0);
  EXPECT_DOUBLE_EQ(catalog->edges[0].polyline[1].first, 5.0);
  EXPECT_DOUBLE_EQ(catalog->edges[0].polyline[1].second, 0.0);
  // map 写法 - x: ... / y: ...
  ASSERT_EQ(catalog->edges[1].polyline.size(), 2u);
  EXPECT_DOUBLE_EQ(catalog->edges[1].polyline[0].first, 5.0);
  EXPECT_DOUBLE_EQ(catalog->edges[1].polyline[0].second, 0.0);
  EXPECT_DOUBLE_EQ(catalog->edges[1].polyline[1].first, 5.0);
  EXPECT_DOUBLE_EQ(catalog->edges[1].polyline[1].second, 5.0);
  // 后续边不受前一条边 polyline 块影响
  EXPECT_DOUBLE_EQ(catalog->edges[2].distance_m, 7.071);
  ASSERT_EQ(catalog->edges[2].polyline.size(), 2u);
  EXPECT_DOUBLE_EQ(catalog->edges[2].polyline[1].first, 10.0);
  EXPECT_DOUBLE_EQ(catalog->edges[2].polyline[1].second, 0.0);
}

TEST(StationCatalogMixedGraphTest, LoadStationCatalog_RejectsCrossSetDuplicateIds) {
  const auto path = WriteCatalogTemp(kCrossDuplicateIds);
  const auto catalog = LoadStationCatalog(path);
  std::remove(path.c_str());
  EXPECT_FALSE(catalog.has_value());
}

// M2：折线块之后的 distance_m/bidirectional/enabled 仍被正常解析
TEST(StationCatalogMixedGraphTest, LoadStationCatalog_ParsesEdgeFieldsAfterPolylineBlock) {
  const auto path = WriteCatalogTemp(kEdgeFieldsAfterPolyline);
  const auto catalog = LoadStationCatalog(path);
  std::remove(path.c_str());
  ASSERT_TRUE(catalog.has_value());
  ASSERT_EQ(catalog->edges.size(), 1u);
  EXPECT_EQ(catalog->edges[0].from, "s1");
  EXPECT_EQ(catalog->edges[0].to, "s2");
  EXPECT_DOUBLE_EQ(catalog->edges[0].distance_m, 3.0);
  EXPECT_TRUE(catalog->edges[0].bidirectional);
  EXPECT_TRUE(catalog->edges[0].enabled);
  ASSERT_EQ(catalog->edges[0].polyline.size(), 2u);
  EXPECT_DOUBLE_EQ(catalog->edges[0].polyline[0].first, 0.0);
  EXPECT_DOUBLE_EQ(catalog->edges[0].polyline[1].first, 1.5);
}

// I1：坏 flow 点（- [broken]）→ 加载严格失败
TEST(StationCatalogMixedGraphTest, LoadStationCatalog_RejectsBrokenFlowPolylinePoint) {
  const auto path = WriteCatalogTemp(kBrokenFlowPolylinePoint);
  const auto catalog = LoadStationCatalog(path);
  std::remove(path.c_str());
  EXPECT_FALSE(catalog.has_value());
}

// I1：折线块内嵌套块序列 → 加载严格失败，不再被当成新边静默吞掉折线
TEST(StationCatalogMixedGraphTest, LoadStationCatalog_RejectsNestedBlockSequencePolyline) {
  const auto path = WriteCatalogTemp(kNestedBlockSequencePolyline);
  const auto catalog = LoadStationCatalog(path);
  std::remove(path.c_str());
  EXPECT_FALSE(catalog.has_value());
}

// I1：map 风格半截点（缺 y）→ 加载严格失败
TEST(StationCatalogMixedGraphTest, LoadStationCatalog_RejectsIncompleteMapPolylinePoint) {
  const auto path = WriteCatalogTemp(kIncompleteMapPolylinePoint);
  const auto catalog = LoadStationCatalog(path);
  std::remove(path.c_str());
  EXPECT_FALSE(catalog.has_value());
}

TEST(StationCatalogMixedGraphTest, FindRoutePath_SpansStationsAndRoadNodes) {
  StationCatalog catalog;
  catalog.stations.push_back(Station{"s1", "map", 0.0, 0.0, 0.0});
  catalog.stations.push_back(Station{"s2", "map", 10.0, 0.0, 0.0});
  catalog.road_nodes.push_back(RoadNode{"n1", "map", 5.0, 0.0, 0.0});
  catalog.road_nodes.push_back(RoadNode{"n2", "map", 5.0, 5.0, 0.0});
  // 短路径途经 road_nodes，长直连边兜底确保必须真的走混合图
  catalog.edges.push_back(CatalogStationEdge{"s1", "n1", 2.0, true, true});
  catalog.edges.push_back(CatalogStationEdge{"n1", "n2", 2.0, true, true});
  catalog.edges.push_back(CatalogStationEdge{"n2", "s2", 2.0, true, true});
  catalog.edges.push_back(CatalogStationEdge{"s1", "s2", 100.0, true, true});

  std::string message;
  ASSERT_TRUE(ValidateStationCatalog(catalog, &message)) << message;

  const auto path = FindRoutePath(catalog, "s1", "s2");
  ASSERT_TRUE(path.has_value());
  EXPECT_EQ(*path, (std::vector<std::string>{"s1", "n1", "n2", "s2"}));

  const auto distance = EstimateRouteDistance(catalog, "s1", "s2");
  ASSERT_TRUE(distance.has_value());
  EXPECT_DOUBLE_EQ(*distance, 6.0);

  // 起终点本身可以是 road_node
  const auto node_path = FindRoutePath(catalog, "n1", "n2");
  ASSERT_TRUE(node_path.has_value());
  EXPECT_EQ(*node_path, (std::vector<std::string>{"n1", "n2"}));

  // 未知端点仍然拒绝
  EXPECT_FALSE(FindRoutePath(catalog, "s1", "missing").has_value());
}

TEST(StationCatalogMixedGraphTest, ValidateStationCatalog_AcceptsRoadNodeEndpoints) {
  StationCatalog catalog;
  catalog.stations.push_back(Station{"s1", "map", 0.0, 0.0, 0.0});
  catalog.road_nodes.push_back(RoadNode{"n1", "map", 5.0, 0.0, 0.0});
  catalog.edges.push_back(CatalogStationEdge{"s1", "n1", 5.0, true, true});
  catalog.edges.push_back(CatalogStationEdge{"n1", "s1", 5.0, true, true});

  std::string message;
  EXPECT_TRUE(ValidateStationCatalog(catalog, &message)) << message;

  // 未知端点仍拒绝
  catalog.edges.push_back(CatalogStationEdge{"n1", "ghost", 1.0, true, true});
  EXPECT_FALSE(ValidateStationCatalog(catalog, &message));
  EXPECT_NE(message.find("ghost"), std::string::npos);
}

TEST(StationCatalogMixedGraphTest, ValidateStationCatalog_RejectsDuplicateIdsAcrossSets) {
  StationCatalog catalog;
  catalog.stations.push_back(Station{"s1", "map", 0.0, 0.0, 0.0});
  catalog.road_nodes.push_back(RoadNode{"s1", "map", 1.0, 1.0, 0.0});  // 与站点撞 id
  std::string message;
  EXPECT_FALSE(ValidateStationCatalog(catalog, &message));
  EXPECT_NE(message.find("duplicate"), std::string::npos);

  // road_nodes 内部重复
  StationCatalog nodes_only;
  nodes_only.stations.push_back(Station{"s1", "map", 0.0, 0.0, 0.0});
  nodes_only.road_nodes.push_back(RoadNode{"n1", "map", 1.0, 1.0, 0.0});
  nodes_only.road_nodes.push_back(RoadNode{"n1", "map", 2.0, 2.0, 0.0});
  EXPECT_FALSE(ValidateStationCatalog(nodes_only, &message));
  EXPECT_NE(message.find("duplicate"), std::string::npos);
}

TEST(StationCatalogMixedGraphTest, ValidateStationCatalog_RejectsNonFinitePolylinePoint) {
  StationCatalog catalog;
  catalog.stations.push_back(Station{"s1", "map", 0.0, 0.0, 0.0});
  catalog.stations.push_back(Station{"s2", "map", 1.0, 0.0, 0.0});
  catalog.edges.push_back(CatalogStationEdge{"s1", "s2", 1.0, true, true});
  catalog.edges.back().polyline.push_back({0.0, 0.0});
  catalog.edges.back().polyline.push_back(
      {std::numeric_limits<double>::quiet_NaN(), 1.0});

  std::string message;
  EXPECT_FALSE(ValidateStationCatalog(catalog, &message));
  EXPECT_NE(message.find("polyline"), std::string::npos);
}

TEST(StationCatalogMixedGraphTest, ValidateStationCatalog_RejectsNonFiniteRoadNodePose) {
  StationCatalog catalog;
  catalog.stations.push_back(Station{"s1", "map", 0.0, 0.0, 0.0});
  catalog.road_nodes.push_back(
      RoadNode{"n1", "map", std::numeric_limits<double>::infinity(), 0.0, 0.0});
  std::string message;
  EXPECT_FALSE(ValidateStationCatalog(catalog, &message));
  EXPECT_NE(message.find("n1"), std::string::npos);
}

}  // namespace
}  // namespace amr_dispatcher_core::catalog
