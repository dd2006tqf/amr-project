#include "amr_dispatcher_core/fleet/vehicle_adapter_interface.hpp"

#include <gtest/gtest.h>

#include <string>
#include <vector>

namespace fleet = amr_dispatcher_core::fleet;

namespace {

fleet::VehicleStateReport MakeReport(const std::string& vehicle_id = "amr_01") {
  fleet::VehicleStateReport report;
  report.vehicle_id = vehicle_id;
  report.order_id = "";
  report.last_node_sequence_id = 3;
  report.last_node_id = "node_3";
  report.x = 1.5;
  report.y = -2.25;
  report.yaw = 1.57;
  report.battery_percent = 87.5;
  report.is_charging = false;
  report.estop_active = false;
  report.online = true;
  report.operating_mode = "AUTOMATIC";
  report.active_errors = {};
  return report;
}

fleet::VehicleOrder MakeOrder(const std::string& vehicle_id = "amr_01") {
  fleet::VehicleOrder order;
  order.order_id = "order_1001";
  order.order_update_id = 7;
  order.vehicle_id = vehicle_id;

  fleet::VehicleNode node_a;
  node_a.node_id = "node_a";
  node_a.sequence_id = 1;
  node_a.released = true;
  node_a.x = 0.0;
  node_a.y = 0.0;
  node_a.yaw = 0.0;
  node_a.actions.push_back(fleet::VehicleAction{"drop", "act_1", {{"station", "dock"}}});
  order.nodes.push_back(node_a);

  fleet::VehicleNode node_b;
  node_b.node_id = "node_b";
  node_b.sequence_id = 2;
  node_b.released = false;
  node_b.x = 4.0;
  node_b.y = 0.0;
  node_b.yaw = 1.57;
  order.nodes.push_back(node_b);

  fleet::VehicleEdge edge;
  edge.edge_id = "edge_a_b";
  edge.start_node_id = "node_a";
  edge.end_node_id = "node_b";
  edge.released = true;
  edge.max_speed = 0.8;
  edge.polyline = {{0.0, 0.0, 0.0}, {2.0, 0.1, 0.05}, {4.0, 0.0, 1.57}};
  edge.actions.push_back(fleet::VehicleAction{"beep", "act_2", {{"reason", "turn"}}});
  order.edges.push_back(edge);

  return order;
}

}  // namespace

TEST(VehicleAdapterInterfaceTest, DispatchOrder_UpdatesVehicleOrderIdAndStoresOrder) {
  fleet::MockVehicleAdapter adapter;
  adapter.SetVehicleState(MakeReport());
  const auto order = MakeOrder();

  EXPECT_TRUE(adapter.DispatchOrder("amr_01", order));

  const auto state = adapter.GetVehicleState("amr_01");
  ASSERT_TRUE(state.has_value());
  EXPECT_EQ(state->order_id, "order_1001");

  const auto stored = adapter.GetDispatchedOrder("amr_01");
  ASSERT_TRUE(stored.has_value());
  EXPECT_EQ(stored->order_id, order.order_id);
  EXPECT_EQ(stored->order_update_id, order.order_update_id);
  EXPECT_EQ(stored->vehicle_id, order.vehicle_id);
  EXPECT_EQ(stored->nodes.size(), order.nodes.size());
  EXPECT_EQ(stored->edges.size(), order.edges.size());
}

TEST(VehicleAdapterInterfaceTest, GetVehicleState_ReturnsInjectedReport) {
  fleet::MockVehicleAdapter adapter;
  adapter.SetVehicleState(MakeReport());

  const auto state = adapter.GetVehicleState("amr_01");
  ASSERT_TRUE(state.has_value());
  EXPECT_EQ(state->vehicle_id, "amr_01");
  EXPECT_EQ(state->last_node_sequence_id, 3u);
  EXPECT_EQ(state->last_node_id, "node_3");
  EXPECT_DOUBLE_EQ(state->x, 1.5);
  EXPECT_DOUBLE_EQ(state->y, -2.25);
  EXPECT_DOUBLE_EQ(state->yaw, 1.57);
  EXPECT_DOUBLE_EQ(state->battery_percent, 87.5);
  EXPECT_FALSE(state->is_charging);
  EXPECT_FALSE(state->estop_active);
  EXPECT_TRUE(state->online);
  EXPECT_EQ(state->operating_mode, "AUTOMATIC");
  EXPECT_TRUE(state->active_errors.empty());
}

TEST(VehicleAdapterInterfaceTest, GetVehicleState_UnknownVehicle_ReturnsNullopt) {
  fleet::MockVehicleAdapter adapter;
  adapter.SetVehicleState(MakeReport());

  EXPECT_FALSE(adapter.GetVehicleState("ghost_vehicle").has_value());
}

TEST(VehicleAdapterInterfaceTest, CancelOrder_ClearsVehicleOrderIdAndForgetsOrder) {
  fleet::MockVehicleAdapter adapter;
  adapter.SetVehicleState(MakeReport());
  const auto order = MakeOrder();
  ASSERT_TRUE(adapter.DispatchOrder("amr_01", order));

  EXPECT_TRUE(adapter.CancelOrder("amr_01", "order_1001"));

  const auto state = adapter.GetVehicleState("amr_01");
  ASSERT_TRUE(state.has_value());
  EXPECT_TRUE(state->order_id.empty());
  EXPECT_FALSE(adapter.GetDispatchedOrder("amr_01").has_value());
}

TEST(VehicleAdapterInterfaceTest, CancelOrder_UnknownOrMismatchedOrder_ReturnsFalse) {
  fleet::MockVehicleAdapter adapter;
  adapter.SetVehicleState(MakeReport());
  const auto order = MakeOrder();
  ASSERT_TRUE(adapter.DispatchOrder("amr_01", order));

  EXPECT_FALSE(adapter.CancelOrder("amr_01", "order_not_dispatched"));
  EXPECT_FALSE(adapter.CancelOrder("ghost_vehicle", "order_1001"));

  // 订单仍应保持原样，未被误删。
  EXPECT_TRUE(adapter.GetDispatchedOrder("amr_01").has_value());
  EXPECT_EQ(adapter.GetVehicleState("amr_01")->order_id, "order_1001");
}

TEST(VehicleAdapterInterfaceTest, PauseAndResumeVehicle_UpdatesOperatingMode) {
  fleet::MockVehicleAdapter adapter;
  adapter.SetVehicleState(MakeReport());

  EXPECT_TRUE(adapter.PauseVehicle("amr_01"));
  EXPECT_EQ(adapter.GetVehicleState("amr_01")->operating_mode, "PAUSED");

  EXPECT_TRUE(adapter.ResumeVehicle("amr_01"));
  EXPECT_EQ(adapter.GetVehicleState("amr_01")->operating_mode, "AUTOMATIC");

  EXPECT_FALSE(adapter.PauseVehicle("ghost_vehicle"));
  EXPECT_FALSE(adapter.ResumeVehicle("ghost_vehicle"));
}

TEST(VehicleAdapterInterfaceTest, DispatchOrder_PreservesMultiNodePolylineOrderIntact) {
  fleet::MockVehicleAdapter adapter;
  adapter.SetVehicleState(MakeReport());
  const auto order = MakeOrder();
  ASSERT_TRUE(adapter.DispatchOrder("amr_01", order));

  const auto stored = adapter.GetDispatchedOrder("amr_01");
  ASSERT_TRUE(stored.has_value());

  ASSERT_EQ(stored->nodes.size(), 2u);
  EXPECT_EQ(stored->nodes[0].node_id, "node_a");
  EXPECT_EQ(stored->nodes[0].sequence_id, 1u);
  EXPECT_TRUE(stored->nodes[0].released);
  ASSERT_EQ(stored->nodes[0].actions.size(), 1u);
  EXPECT_EQ(stored->nodes[0].actions[0].action_type, "drop");
  EXPECT_EQ(stored->nodes[0].actions[0].action_id, "act_1");
  EXPECT_EQ(stored->nodes[0].actions[0].parameters.at("station"), "dock");
  EXPECT_EQ(stored->nodes[1].node_id, "node_b");
  EXPECT_EQ(stored->nodes[1].sequence_id, 2u);
  EXPECT_FALSE(stored->nodes[1].released);

  ASSERT_EQ(stored->edges.size(), 1u);
  const auto& edge = stored->edges[0];
  EXPECT_EQ(edge.edge_id, "edge_a_b");
  EXPECT_EQ(edge.start_node_id, "node_a");
  EXPECT_EQ(edge.end_node_id, "node_b");
  EXPECT_TRUE(edge.released);
  EXPECT_DOUBLE_EQ(edge.max_speed, 0.8);
  ASSERT_EQ(edge.polyline.size(), 3u);
  EXPECT_DOUBLE_EQ(edge.polyline[1].x, 2.0);
  EXPECT_DOUBLE_EQ(edge.polyline[1].y, 0.1);
  EXPECT_DOUBLE_EQ(edge.polyline[1].yaw, 0.05);
  EXPECT_DOUBLE_EQ(edge.polyline[2].x, 4.0);
  EXPECT_DOUBLE_EQ(edge.polyline[2].yaw, 1.57);
  ASSERT_EQ(edge.actions.size(), 1u);
  EXPECT_EQ(edge.actions[0].action_type, "beep");
  EXPECT_EQ(edge.actions[0].parameters.at("reason"), "turn");
}

TEST(VehicleAdapterInterfaceTest, DispatchOrder_UnknownVehicle_StillStoresOrder) {
  fleet::MockVehicleAdapter adapter;
  const auto order = MakeOrder("ghost_vehicle");

  EXPECT_TRUE(adapter.DispatchOrder("ghost_vehicle", order));
  EXPECT_FALSE(adapter.GetVehicleState("ghost_vehicle").has_value());
  EXPECT_TRUE(adapter.GetDispatchedOrder("ghost_vehicle").has_value());
}
