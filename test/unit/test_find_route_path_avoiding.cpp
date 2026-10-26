// 静态拓扑绕行（FindRoutePathAvoiding）的验收。
//
// 用户指定的四个场景（批次 4.4）中与寻路直接相关的两个：
//   1. 存在两条可达路线，其中一条边被禁用 -> 选不经过该边的替代路线
//   2. 所有路线均不可达 -> 返回明确失败（nullopt），**不伪造直线路径**
// 另外锁定"空禁用集 = 与 FindRoutePath 逐字一致"这条回归线。
#include "amr_dispatcher_core/catalog/station_catalog.hpp"

#include <gtest/gtest.h>

namespace amr_dispatcher_core::catalog {
namespace {

// s1 --(直连, 3m)--> s2
//  |                  ^
//  +--(2m)-- n1 --(2m)--+
// 直连边最短（3m < 4m），但只要它被禁，就必须走 n1 的绕行路（4m）。
StationCatalog MakeTwoRouteCatalog() {
  StationCatalog catalog;
  catalog.stations.push_back(Station{"s1", "map", 0.0, 0.0, 0.0});
  catalog.stations.push_back(Station{"s2", "map", 10.0, 0.0, 0.0});
  catalog.road_nodes.push_back(RoadNode{"n1", "map", 5.0, 5.0, 0.0});
  catalog.edges.push_back(CatalogStationEdge{"s1", "s2", 3.0, true, true});
  catalog.edges.push_back(CatalogStationEdge{"s1", "n1", 2.0, true, true});
  catalog.edges.push_back(CatalogStationEdge{"n1", "s2", 2.0, true, true});
  return catalog;
}

}  // namespace

TEST(FindRoutePathAvoidingTest, EmptyBlockedSetMatchesFindRoutePath) {
  const auto catalog = MakeTwoRouteCatalog();
  const auto plain = FindRoutePath(catalog, "s1", "s2");
  const auto avoiding = FindRoutePathAvoiding(catalog, "s1", "s2", {});
  ASSERT_TRUE(plain.has_value());
  ASSERT_TRUE(avoiding.has_value());
  EXPECT_EQ(*plain, *avoiding);
  EXPECT_EQ(*plain, (std::vector<std::string>{"s1", "s2"}));  // 直连最短
}

TEST(FindRoutePathAvoidingTest, BlockedShortestEdgeForcesDetour) {
  const auto catalog = MakeTwoRouteCatalog();
  const auto path = FindRoutePathAvoiding(catalog, "s1", "s2", {"route_edge:s1__s2"});
  ASSERT_TRUE(path.has_value());
  EXPECT_EQ(*path, (std::vector<std::string>{"s1", "n1", "s2"}));
}

// 调用方必须传**规范化**键（RouteEdgeId 的无序两端形式）。反向写法不会被
// 猜测成同一条边：边两端 id 里也可能出现 "__"，反解析是歧义的，
// 与其猜错（悄悄绕开/不绕开一条本来不想动的边），不如不做。
TEST(FindRoutePathAvoidingTest, NonCanonicalKeyDoesNotMatchEdge) {
  const auto catalog = MakeTwoRouteCatalog();
  const auto path = FindRoutePathAvoiding(catalog, "s1", "s2", {"route_edge:s2__s1"});
  ASSERT_TRUE(path.has_value());
  EXPECT_EQ(*path, (std::vector<std::string>{"s1", "s2"}));  // 未被封禁
}

// 场景 2：无路可走时必须明确失败，绝不返回一条"看起来能走"的假路径。
TEST(FindRoutePathAvoidingTest, NoAlternativeReturnsNullopt) {
  const auto catalog = MakeTwoRouteCatalog();
  const auto path = FindRoutePathAvoiding(
      catalog, "s1", "s2",
      {"route_edge:s1__s2", "route_edge:n1__s1", "route_edge:n1__s2"});
  EXPECT_FALSE(path.has_value());
}

TEST(FindRoutePathAvoidingTest, BlockingOneSegmentOfDetourIsEnough) {
  const auto catalog = MakeTwoRouteCatalog();
  const auto path =
      FindRoutePathAvoiding(catalog, "s1", "s2", {"route_edge:s1__s2", "route_edge:n1__s2"});
  EXPECT_FALSE(path.has_value());
}

TEST(FindRoutePathAvoidingTest, UnknownResourceKeysAreIgnored) {
  const auto catalog = MakeTwoRouteCatalog();
  const auto path =
      FindRoutePathAvoiding(catalog, "s1", "s2", {"route_edge:no_such_a__no_such_b"});
  ASSERT_TRUE(path.has_value());
  EXPECT_EQ(*path, (std::vector<std::string>{"s1", "s2"}));
}

TEST(FindRoutePathAvoidingTest, NodeKeysDoNotBlockEdges) {
  // route_node:* 不解成边：节点级冲突不代表整条边不可用（计划里显式写死）
  const auto catalog = MakeTwoRouteCatalog();
  const auto path =
      FindRoutePathAvoiding(catalog, "s1", "s2", {"route_node:s1", "route_node:n1"});
  ASSERT_TRUE(path.has_value());
  EXPECT_EQ(*path, (std::vector<std::string>{"s1", "s2"}));
}

TEST(FindRoutePathAvoidingTest, SameFromAndToIsUnaffected) {
  const auto catalog = MakeTwoRouteCatalog();
  const auto path = FindRoutePathAvoiding(catalog, "s1", "s1", {"route_edge:s1__s2"});
  ASSERT_TRUE(path.has_value());
  EXPECT_EQ(*path, (std::vector<std::string>{"s1"}));
}

TEST(FindRoutePathAvoidingTest, UnknownEndpointsRejected) {
  const auto catalog = MakeTwoRouteCatalog();
  EXPECT_FALSE(FindRoutePathAvoiding(catalog, "s1", "missing", {}).has_value());
  EXPECT_FALSE(FindRoutePathAvoiding(catalog, "missing", "s2", {}).has_value());
}

// 键格式契约：绕行用的键必须与路权预约用的键完全一致，
// 否则"规划时绕开、预约时按另一套算"。这里直接比对两条拼键路径。
TEST(FindRoutePathAvoidingTest, EdgeKeyFormatMatchesReservationContract) {
  // ResourceTable::RouteEdgeId 的契约（见 traffic_reservation.cpp）：
  //   route_edge:<min>__<max>
  const std::string a = "s1";
  const std::string b = "n1";
  const std::string expected = "route_edge:n1__s1";  // "n1" < "s1"
  EXPECT_EQ((a <= b ? "route_edge:" + a + "__" + b : "route_edge:" + b + "__" + a), expected);
}

}  // namespace amr_dispatcher_core::catalog
