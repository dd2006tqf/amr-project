#include "amr_dispatcher_core/fleet/vda5050_order_builder.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <unordered_set>
#include <vector>

#include "amr_dispatcher_core/catalog/station_catalog.hpp"
#include "amr_dispatcher_core/dispatcher/traffic_reservation.hpp"

namespace fleet = amr_dispatcher_core::fleet;
namespace catalog_ns = amr_dispatcher_core::catalog;
using amr_dispatcher_core::dispatcher::ResourceTable;

namespace {

// L 形拓扑：station_A(0,0) --弯折线--> rn_1(10,0) --竖直--> station_B(10,10)
catalog_ns::StationCatalog MakeCatalog() {
  catalog_ns::StationCatalog catalog;
  catalog.stations.push_back({"station_A", "map", 0.0, 0.0, 0.0, "pickup"});
  catalog.stations.push_back({"station_B", "map", 10.0, 10.0, 1.5707963, "dropoff"});
  catalog.road_nodes.push_back({"rn_1", "map", 10.0, 0.0, 1.5707963});

  catalog_ns::CatalogStationEdge to_corner;
  to_corner.from = "station_A";
  to_corner.to = "rn_1";
  to_corner.distance_m = 10.0;
  to_corner.bidirectional = true;
  to_corner.enabled = true;
  to_corner.polyline = {{0.0, 0.0}, {4.0, 0.0}, {7.0, 0.5}, {10.0, 0.0}};  // 真实弯曲折线
  catalog.edges.push_back(to_corner);

  catalog_ns::CatalogStationEdge to_dest;
  to_dest.from = "rn_1";
  to_dest.to = "station_B";
  to_dest.distance_m = 10.0;
  to_dest.bidirectional = true;
  to_dest.enabled = true;
  to_dest.polyline = {{10.0, 0.0}, {10.0, 5.0}, {10.0, 10.0}};
  catalog.edges.push_back(to_dest);
  return catalog;
}

std::vector<std::string> MakePath() { return {"station_A", "rn_1", "station_B"}; }

std::unordered_set<std::string> LocksFor(const std::vector<std::string>& path) {
  const auto lock_ids = ResourceTable::BuildLockIds(path, /*include_intersection_locks=*/true);
  return std::unordered_set<std::string>(lock_ids.begin(), lock_ids.end());
}

}  // namespace

TEST(Vda5050OrderBuilderTest, FullLockedPath_MarksAllReleasedAndAttachesTerminalAction) {
  const auto catalog = MakeCatalog();
  const auto path = MakePath();
  const auto reserved = LocksFor(path);

  fleet::VehicleAction action;
  action.action_type = "drop";
  action.action_id = "act_drop_1";
  action.parameters = {{"station", "station_B"}};

  auto order = fleet::BuildVda5050OrderFromPath(
      "amr_01", "mission_42", /*order_update_id=*/5, path, catalog, reserved, {action});

  ASSERT_TRUE(order.has_value());
  EXPECT_EQ(order->vehicle_id, "amr_01");
  EXPECT_EQ(order->order_id, "mission_42");
  EXPECT_EQ(order->order_update_id, 5U);

  ASSERT_EQ(order->nodes.size(), 3U);
  ASSERT_EQ(order->edges.size(), 2U);

  // 全锁场景：Base 区间贯通整单
  for (const auto& node : order->nodes) {
    EXPECT_TRUE(node.released) << node.node_id;
  }
  for (const auto& edge : order->edges) {
    EXPECT_TRUE(edge.released) << edge.edge_id;
  }

  // VDA 5050 奇数 sequenceId 规范：0, 2, 4
  EXPECT_EQ(order->nodes[0].sequence_id, 0U);
  EXPECT_EQ(order->nodes[1].sequence_id, 2U);
  EXPECT_EQ(order->nodes[2].sequence_id, 4U);

  // 坐标从 stations / road_nodes 中提取
  EXPECT_DOUBLE_EQ(order->nodes[0].x, 0.0);
  EXPECT_DOUBLE_EQ(order->nodes[0].y, 0.0);
  EXPECT_DOUBLE_EQ(order->nodes[1].x, 10.0);
  EXPECT_DOUBLE_EQ(order->nodes[1].y, 0.0);
  EXPECT_DOUBLE_EQ(order->nodes[2].x, 10.0);
  EXPECT_DOUBLE_EQ(order->nodes[2].y, 10.0);
  EXPECT_DOUBLE_EQ(order->nodes[2].yaw, 1.5707963);

  // 终点动作挂载在最后一个节点，前序节点不挂动作
  EXPECT_TRUE(order->nodes[0].actions.empty());
  EXPECT_TRUE(order->nodes[1].actions.empty());
  ASSERT_EQ(order->nodes[2].actions.size(), 1U);
  EXPECT_EQ(order->nodes[2].actions[0].action_type, "drop");
  EXPECT_EQ(order->nodes[2].actions[0].action_id, "act_drop_1");

  // 边字段按 ResourceTable 命名规则生成
  EXPECT_EQ(order->edges[0].start_node_id, "station_A");
  EXPECT_EQ(order->edges[0].end_node_id, "rn_1");
  EXPECT_EQ(order->edges[0].edge_id, ResourceTable::RouteEdgeId("station_A", "rn_1"));
  EXPECT_EQ(order->edges[1].edge_id, ResourceTable::RouteEdgeId("rn_1", "station_B"));
  EXPECT_EQ(order->edges[0].max_speed, 1.0);
}

TEST(Vda5050OrderBuilderTest, PartialLock_MarksBaseThenHorizonSegment) {
  const auto catalog = MakeCatalog();
  const auto path = MakePath();

  // 三个节点锁全有，仅第一条边有锁（第二条边未获路权）
  auto reserved = LocksFor(path);
  reserved.erase(ResourceTable::RouteEdgeId("rn_1", "station_B"));

  auto order = fleet::BuildVda5050OrderFromPath(
      "amr_01", "mission_43", /*order_update_id=*/0, path, catalog, reserved);

  ASSERT_TRUE(order.has_value());
  ASSERT_EQ(order->nodes.size(), 3U);
  ASSERT_EQ(order->edges.size(), 2U);

  // 前两节点 + 第一条边属于 Base 区间
  EXPECT_TRUE(order->nodes[0].released);
  EXPECT_TRUE(order->edges[0].released);
  EXPECT_TRUE(order->nodes[1].released);

  // 第二条边无锁 -> Horizon；其后的节点即使持有节点锁也必须被拉平为 false
  EXPECT_FALSE(order->edges[1].released);
  EXPECT_FALSE(order->nodes[2].released);
}

TEST(Vda5050OrderBuilderTest, CurvedPolyline_PreservedInOrderAndReversedWhenTraversingBackward) {
  const auto catalog = MakeCatalog();

  // 正向：A -> rn_1 -> B
  const auto path = MakePath();
  const auto reserved = LocksFor(path);
  auto forward = fleet::BuildVda5050OrderFromPath(
      "amr_01", "mission_44", 0, path, catalog, reserved);

  ASSERT_TRUE(forward.has_value());
  ASSERT_EQ(forward->edges[0].polyline.size(), 4U);
  EXPECT_DOUBLE_EQ(forward->edges[0].polyline[0].x, 0.0);
  EXPECT_DOUBLE_EQ(forward->edges[0].polyline[0].y, 0.0);
  EXPECT_DOUBLE_EQ(forward->edges[0].polyline[1].x, 4.0);
  EXPECT_DOUBLE_EQ(forward->edges[0].polyline[1].y, 0.0);
  EXPECT_DOUBLE_EQ(forward->edges[0].polyline[2].x, 7.0);
  EXPECT_DOUBLE_EQ(forward->edges[0].polyline[2].y, 0.5);
  EXPECT_DOUBLE_EQ(forward->edges[0].polyline[3].x, 10.0);
  EXPECT_DOUBLE_EQ(forward->edges[0].polyline[3].y, 0.0);

  // yaw 取所在行驶方向：首段水平 -> 0，次段 atan2(0.5, 3)
  EXPECT_NEAR(forward->edges[0].polyline[0].yaw, 0.0, 1e-9);
  EXPECT_NEAR(forward->edges[0].polyline[1].yaw, std::atan2(0.5, 3.0), 1e-9);

  ASSERT_EQ(forward->edges[1].polyline.size(), 3U);
  EXPECT_NEAR(forward->edges[1].polyline[0].yaw, M_PI / 2.0, 1e-9);

  // 反向：B -> rn_1 -> A，双向边折线需按行驶方向反转
  const std::vector<std::string> reverse_path = {"station_B", "rn_1", "station_A"};
  const auto reverse_locks = LocksFor(reverse_path);
  auto reverse = fleet::BuildVda5050OrderFromPath(
      "amr_01", "mission_45", 0, reverse_path, catalog, reverse_locks);

  ASSERT_TRUE(reverse.has_value());
  ASSERT_EQ(reverse->edges[0].polyline.size(), 3U);
  EXPECT_DOUBLE_EQ(reverse->edges[0].polyline[0].x, 10.0);
  EXPECT_DOUBLE_EQ(reverse->edges[0].polyline[0].y, 10.0);
  EXPECT_DOUBLE_EQ(reverse->edges[0].polyline[2].x, 10.0);
  EXPECT_DOUBLE_EQ(reverse->edges[0].polyline[2].y, 0.0);
  EXPECT_NEAR(reverse->edges[0].polyline[0].yaw, -M_PI / 2.0, 1e-9);

  ASSERT_EQ(reverse->edges[1].polyline.size(), 4U);
  EXPECT_DOUBLE_EQ(reverse->edges[1].polyline[0].x, 10.0);
  EXPECT_DOUBLE_EQ(reverse->edges[1].polyline[0].y, 0.0);
  EXPECT_DOUBLE_EQ(reverse->edges[1].polyline[3].x, 0.0);
  EXPECT_DOUBLE_EQ(reverse->edges[1].polyline[3].y, 0.0);
}

TEST(Vda5050OrderBuilderTest, MissingCatalogEdge_FallsBackToStraightLine) {
  const auto catalog = MakeCatalog();
  // A 与 B 之间无直连边：触发两端节点直线插值
  const std::vector<std::string> path = {"station_A", "station_B"};
  const auto reserved = LocksFor(path);

  auto order = fleet::BuildVda5050OrderFromPath(
      "amr_01", "mission_46", 0, path, catalog, reserved);

  ASSERT_TRUE(order.has_value());
  ASSERT_EQ(order->edges.size(), 1U);
  ASSERT_EQ(order->edges[0].polyline.size(), 2U);
  EXPECT_DOUBLE_EQ(order->edges[0].polyline[0].x, 0.0);
  EXPECT_DOUBLE_EQ(order->edges[0].polyline[0].y, 0.0);
  EXPECT_DOUBLE_EQ(order->edges[0].polyline[1].x, 10.0);
  EXPECT_DOUBLE_EQ(order->edges[0].polyline[1].y, 10.0);
  EXPECT_NEAR(order->edges[0].polyline[0].yaw, M_PI / 4.0, 1e-9);
  EXPECT_TRUE(order->edges[0].released);
}

TEST(Vda5050OrderBuilderTest, CustomConfig_OverridesDefaultMaxSpeed) {
  const auto catalog = MakeCatalog();
  const auto path = MakePath();
  const auto reserved = LocksFor(path);

  fleet::Vda5050OrderBuilderConfig config;
  config.default_max_speed_mps = 2.5;

  auto order = fleet::BuildVda5050OrderFromPath(
      "amr_01", "mission_47", 0, path, catalog, reserved, {}, config);

  ASSERT_TRUE(order.has_value());
  ASSERT_EQ(order->edges.size(), 2U);
  EXPECT_EQ(order->edges[0].max_speed, 2.5);
  EXPECT_EQ(order->edges[1].max_speed, 2.5);
}

TEST(Vda5050OrderBuilderTest, InvalidInput_EmptyPathOrUnknownNode_ReturnsNullopt) {
  const auto catalog = MakeCatalog();
  const auto reserved = LocksFor(MakePath());

  // 空路径
  EXPECT_FALSE(fleet::BuildVda5050OrderFromPath(
      "amr_01", "mission_48", 0, {}, catalog, reserved).has_value());

  // 起点未知
  EXPECT_FALSE(fleet::BuildVda5050OrderFromPath(
      "amr_01", "mission_48", 0, {"ghost_station", "rn_1"}, catalog, reserved).has_value());

  // 中途未知
  EXPECT_FALSE(fleet::BuildVda5050OrderFromPath(
      "amr_01", "mission_48", 0, {"station_A", "ghost_node", "station_B"}, catalog, reserved)
                   .has_value());
}
