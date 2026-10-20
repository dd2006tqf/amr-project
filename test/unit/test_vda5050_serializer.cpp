#include "amr_dispatcher_core/fleet/vda5050_serializer.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

namespace fleet = amr_dispatcher_core::fleet;

namespace {

fleet::VehicleOrder MakeTestOrder() {
  fleet::VehicleOrder order;
  order.order_id = "order_test_001";
  order.order_update_id = 1;
  order.vehicle_id = "robot_01";

  fleet::VehicleNode node1;
  node1.node_id = "node_1";
  node1.sequence_id = 1;
  node1.released = true;
  node1.x = 10.0;
  node1.y = 20.0;
  node1.yaw = 1.57;
  fleet::VehicleAction act1;
  act1.action_type = "lift";
  act1.action_id = "act_lift_1";
  act1.parameters = {{"height", "0.5"}};
  node1.actions.push_back(act1);
  order.nodes.push_back(node1);

  fleet::VehicleNode node2;
  node2.node_id = "node_2";
  node2.sequence_id = 3;
  node2.released = false;
  node2.x = 15.0;
  node2.y = 25.0;
  node2.yaw = 0.0;
  order.nodes.push_back(node2);

  fleet::VehicleEdge edge;
  edge.edge_id = "edge_1_2";
  edge.start_node_id = "node_1";
  edge.end_node_id = "node_2";
  edge.released = true;
  edge.max_speed = 1.2;
  edge.polyline = {{10.0, 20.0, 1.57}, {12.5, 22.5, 0.78}, {15.0, 25.0, 0.0}};
  fleet::VehicleAction act2;
  act2.action_type = "beep";
  act2.action_id = "act_beep_1";
  act2.parameters = {{"duration", "2"}};
  edge.actions.push_back(act2);
  order.edges.push_back(edge);

  return order;
}

}  // namespace

TEST(Vda5050SerializerTest, SerializeOrder_V2_ContainsRequiredFields) {
  const auto order = MakeTestOrder();
  const std::string json = fleet::SerializeVda5050Order(order, "v2");

  EXPECT_NE(json.find("\"orderId\":\"order_test_001\""), std::string::npos);
  EXPECT_NE(json.find("\"orderUpdateId\":1"), std::string::npos);
  EXPECT_NE(json.find("\"serialNumber\":\"robot_01\""), std::string::npos);
  EXPECT_NE(json.find("\"version\":\"2.0.0\""), std::string::npos);
  EXPECT_NE(json.find("\"nodes\":["), std::string::npos);
  EXPECT_NE(json.find("\"nodeId\":\"node_1\""), std::string::npos);
  EXPECT_NE(json.find("\"sequenceId\":1"), std::string::npos);
  EXPECT_NE(json.find("\"released\":true"), std::string::npos);
  EXPECT_NE(json.find("\"released\":false"), std::string::npos);
  EXPECT_NE(json.find("\"nodePosition\":{"), std::string::npos);
  EXPECT_NE(json.find("\"x\":10"), std::string::npos);
  EXPECT_NE(json.find("\"y\":20"), std::string::npos);
  EXPECT_NE(json.find("\"theta\":1.57"), std::string::npos);
  EXPECT_NE(json.find("\"mapId\":\"map\""), std::string::npos);
  EXPECT_NE(json.find("\"edges\":["), std::string::npos);
  EXPECT_NE(json.find("\"edgeId\":\"edge_1_2\""), std::string::npos);
  EXPECT_NE(json.find("\"startNodeId\":\"node_1\""), std::string::npos);
  EXPECT_NE(json.find("\"endNodeId\":\"node_2\""), std::string::npos);
  EXPECT_NE(json.find("\"maxSpeed\":1.2"), std::string::npos);
  EXPECT_NE(json.find("\"trajectory\":{"), std::string::npos);
  EXPECT_NE(json.find("\"controlPoints\":["), std::string::npos);
  EXPECT_NE(json.find("\"actionType\":\"lift\""), std::string::npos);
  EXPECT_NE(json.find("\"actionId\":\"act_lift_1\""), std::string::npos);
}

TEST(Vda5050SerializerTest, SerializeOrder_V3_SetsVersion3) {
  const auto order = MakeTestOrder();
  const std::string json = fleet::SerializeVda5050Order(order, "v3");

  EXPECT_NE(json.find("\"version\":\"3.0.0\""), std::string::npos);
}

TEST(Vda5050SerializerTest, SerializeInstantActions_ContainsActions) {
  std::vector<fleet::VehicleAction> actions;
  fleet::VehicleAction act1;
  act1.action_type = "cancelOrder";
  act1.action_id = "act_cancel_1";
  act1.parameters = {{"orderId", "order_123"}};

  fleet::VehicleAction act2;
  act2.action_type = "pause";
  act2.action_id = "act_pause_2";

  actions.push_back(act1);
  actions.push_back(act2);

  const std::string json = fleet::SerializeVda5050InstantActions("robot_99", actions, "v2");

  EXPECT_NE(json.find("\"serialNumber\":\"robot_99\""), std::string::npos);
  EXPECT_NE(json.find("\"actionType\":\"cancelOrder\""), std::string::npos);
  EXPECT_NE(json.find("\"actionId\":\"act_cancel_1\""), std::string::npos);
  EXPECT_NE(json.find("\"actionType\":\"pause\""), std::string::npos);
  EXPECT_NE(json.find("\"blockingType\":\"HARD\""), std::string::npos);
  EXPECT_NE(json.find("\"key\":\"orderId\""), std::string::npos);
  EXPECT_NE(json.find("\"value\":\"order_123\""), std::string::npos);
}

TEST(Vda5050SerializerTest, DeserializeState_ValidPayload_ParsesAllFields) {
  const std::string valid_json = R"({
    "headerId": 42,
    "timestamp": "2026-10-20T12:00:00.000Z",
    "version": "2.0.0",
    "manufacturer": "RobotManufacturer",
    "serialNumber": "robot_01",
    "orderId": "order_test_001",
    "orderUpdateId": 1,
    "lastNodeId": "node_1",
    "lastNodeSequenceId": 1,
    "driving": true,
    "agvPosition": {
      "x": 10.5,
      "y": 20.25,
      "theta": 1.5708,
      "mapId": "map",
      "positionInitialized": true
    },
    "batteryState": {
      "batteryCharge": 85.5,
      "charging": false
    },
    "safetyState": {
      "eStop": "none",
      "fieldViolation": false
    },
    "operatingMode": "AUTOMATIC",
    "errors": [
      {
        "errorType": "warning_sensor",
        "errorDescription": "lidar lens dirty",
        "errorLevel": "warning"
      }
    ]
  })";

  const auto report = fleet::DeserializeVda5050State(valid_json);
  ASSERT_TRUE(report.has_value());
  EXPECT_EQ(report->vehicle_id, "robot_01");
  EXPECT_EQ(report->order_id, "order_test_001");
  EXPECT_EQ(report->last_node_id, "node_1");
  EXPECT_EQ(report->last_node_sequence_id, 1u);
  EXPECT_DOUBLE_EQ(report->x, 10.5);
  EXPECT_DOUBLE_EQ(report->y, 20.25);
  EXPECT_DOUBLE_EQ(report->yaw, 1.5708);
  EXPECT_DOUBLE_EQ(report->battery_percent, 85.5);
  EXPECT_FALSE(report->is_charging);
  EXPECT_FALSE(report->estop_active);
  EXPECT_TRUE(report->online);
  EXPECT_EQ(report->operating_mode, "AUTOMATIC");
  ASSERT_EQ(report->active_errors.size(), 1u);
  EXPECT_EQ(report->active_errors[0], "warning_sensor: lidar lens dirty");
}

TEST(Vda5050SerializerTest, DeserializeState_EstopActive_SetsEstop) {
  const std::string estop_json = R"({
    "headerId": 1,
    "serialNumber": "amr_e",
    "orderId": "ord_1",
    "lastNodeId": "n1",
    "lastNodeSequenceId": 2,
    "agvPosition": {"x": 1.0, "y": 2.0, "theta": 0.0},
    "batteryState": {"batteryCharge": 50.0, "charging": true},
    "safetyState": {"eStop": "manual", "fieldViolation": false},
    "operatingMode": "MANUAL",
    "errors": []
  })";

  const auto report = fleet::DeserializeVda5050State(estop_json);
  ASSERT_TRUE(report.has_value());
  EXPECT_TRUE(report->estop_active);
  EXPECT_TRUE(report->is_charging);
  EXPECT_EQ(report->operating_mode, "MANUAL");
}

TEST(Vda5050SerializerTest, DeserializeState_MalformedOrMissingFields_ReturnsNullopt) {
  // Empty string
  EXPECT_FALSE(fleet::DeserializeVda5050State("").has_value());

  // Not a JSON object
  EXPECT_FALSE(fleet::DeserializeVda5050State("[1, 2, 3]").has_value());

  // Unclosed JSON
  EXPECT_FALSE(fleet::DeserializeVda5050State("{\"serialNumber\": \"amr_01\"").has_value());

  // Missing serialNumber
  const std::string missing_serial = R"({
    "orderId": "ord_1",
    "agvPosition": {"x": 1.0, "y": 2.0, "theta": 0.0}
  })";
  EXPECT_FALSE(fleet::DeserializeVda5050State(missing_serial).has_value());

  // Missing agvPosition
  const std::string missing_pos = R"({
    "serialNumber": "amr_01",
    "orderId": "ord_1"
  })";
  EXPECT_FALSE(fleet::DeserializeVda5050State(missing_pos).has_value());

  // NaN or Inf in position
  const std::string nan_pos = R"({
    "serialNumber": "amr_01",
    "agvPosition": {"x": "not_a_number", "y": 2.0, "theta": 0.0}
  })";
  EXPECT_FALSE(fleet::DeserializeVda5050State(nan_pos).has_value());
}

TEST(Vda5050SerializerTest, DeserializeConnection_ValidOnlineAndBroken) {
  const std::string online_json = R"({
    "headerId": 1,
    "timestamp": "2026-10-20T12:00:00.000Z",
    "version": "2.0.0",
    "manufacturer": "RobotManufacturer",
    "serialNumber": "robot_01",
    "connectionState": "ONLINE"
  })";

  const auto state1 = fleet::DeserializeVda5050Connection(online_json);
  ASSERT_TRUE(state1.has_value());
  EXPECT_EQ(*state1, "ONLINE");

  const std::string broken_json = R"({
    "serialNumber": "robot_01",
    "connectionState": "CONNECTIONBROKEN"
  })";

  const auto state2 = fleet::DeserializeVda5050Connection(broken_json);
  ASSERT_TRUE(state2.has_value());
  EXPECT_EQ(*state2, "CONNECTIONBROKEN");

  const std::string offline_json = R"({
    "connectionState": "OFFLINE"
  })";
  const auto state3 = fleet::DeserializeVda5050Connection(offline_json);
  ASSERT_TRUE(state3.has_value());
  EXPECT_EQ(*state3, "OFFLINE");
}

TEST(Vda5050SerializerTest, DeserializeConnection_MissingOrInvalid_ReturnsNullopt) {
  EXPECT_FALSE(fleet::DeserializeVda5050Connection("").has_value());
  EXPECT_FALSE(fleet::DeserializeVda5050Connection("{}").has_value());
  EXPECT_FALSE(fleet::DeserializeVda5050Connection("{\"otherField\": 123}").has_value());
  EXPECT_FALSE(fleet::DeserializeVda5050Connection("not json").has_value());
}
