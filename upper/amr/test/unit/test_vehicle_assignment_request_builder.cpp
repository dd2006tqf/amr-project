#include "amr_dispatcher_core/fleet/vehicle_assignment_request_builder.hpp"

#include <gtest/gtest.h>

#include <string>
#include <vector>

using namespace amr_dispatcher_core::fleet;
using amr_dispatcher_core::catalog::CatalogStationEdge;
using amr_dispatcher_core::catalog::Station;
using amr_dispatcher_core::catalog::StationCatalog;

namespace {

StationCatalog MakeStations() {
  StationCatalog catalog;
  catalog.stations.push_back(Station{"dock", "map", 0.0, 0.0, 0.0});
  catalog.stations.push_back(Station{"receiving", "map", 1.0, 0.0, 0.0});
  catalog.stations.push_back(Station{"storage_a", "map", 2.0, 0.0, 0.0});
  catalog.edges.push_back(CatalogStationEdge{"dock", "receiving", 3.0, true, true});
  catalog.edges.push_back(CatalogStationEdge{"receiving", "storage_a", 1.0, true, true});
  return catalog;
}

VehicleRuntimeState MakeVehicle(const std::string& id, double battery = 100.0) {
  VehicleRuntimeState v;
  v.vehicle_id = id;
  v.battery_percent = battery;
  v.position_valid = true;
  return v;
}

VehicleAssignmentRequestInputs MakeInputs(std::vector<VehicleRuntimeState> vehicles) {
  VehicleAssignmentRequestInputs inputs;
  inputs.vehicles = std::move(vehicles);
  inputs.pickup_station_id = "storage_a";
  inputs.dropoff_station_id = "dock";
  inputs.required_capability = "transport";
  inputs.state_max_age_ms = 5000;
  return inputs;
}

}  // namespace

TEST(VehicleAssignmentRequestBuilderTest, CopiesTaskFields) {
  auto inputs = MakeInputs({MakeVehicle("v1")});
  inputs.requested_vehicle_id = "v1";
  inputs.min_battery_percent = 25.0;
  const auto request = BuildVehicleAssignmentRequest(inputs);
  EXPECT_EQ(request.requested_vehicle_id, "v1");
  EXPECT_EQ(request.pickup_station_id, "storage_a");
  EXPECT_EQ(request.dropoff_station_id, "dock");
  EXPECT_EQ(request.required_capability, "transport");
  EXPECT_DOUBLE_EQ(request.min_battery_percent, 25.0);
  EXPECT_EQ(request.state_max_age_ms, 5000);
}

// 核心属性：同分时的裁决顺序必须由 id 决定，而不是容器的哈希遍历序。
TEST(VehicleAssignmentRequestBuilderTest, VehiclesAreSortedByIdRegardlessOfInputOrder) {
  const auto request = BuildVehicleAssignmentRequest(
      MakeInputs({MakeVehicle("zeta"), MakeVehicle("alpha"), MakeVehicle("mid")}));
  ASSERT_EQ(request.vehicles.size(), 3u);
  EXPECT_EQ(request.vehicles[0].vehicle_id, "alpha");
  EXPECT_EQ(request.vehicles[1].vehicle_id, "mid");
  EXPECT_EQ(request.vehicles[2].vehicle_id, "zeta");
}

TEST(VehicleAssignmentRequestBuilderTest, StableOrderIsPermutationOfInput) {
  const std::vector<VehicleRuntimeState> vehicles{MakeVehicle("b"), MakeVehicle("a"),
                                                  MakeVehicle("c")};
  const auto order = StableVehicleOrder(vehicles);
  ASSERT_EQ(order.size(), 3u);
  std::vector<std::string> ids;
  for (const auto idx : order) {
    ASSERT_LT(idx, vehicles.size());
    ids.push_back(vehicles[idx].vehicle_id);
  }
  EXPECT_EQ(ids, (std::vector<std::string>{"a", "b", "c"}));
}

TEST(VehicleAssignmentRequestBuilderTest, NoCatalogMeansNoDistanceEstimates) {
  const auto request = BuildVehicleAssignmentRequest(MakeInputs({MakeVehicle("v1")}));
  EXPECT_TRUE(request.distance_to_pickup_m.empty());
}

TEST(VehicleAssignmentRequestBuilderTest, CatalogNotLoadedMeansNoDistanceEstimates) {
  const StationCatalog catalog = MakeStations();
  auto inputs = MakeInputs({MakeVehicle("v1")});
  inputs.catalog = &catalog;
  inputs.catalog_loaded = false;  // 有目录但未加载：仍不算成本
  const auto request = BuildVehicleAssignmentRequest(inputs);
  EXPECT_TRUE(request.distance_to_pickup_m.empty());
}

TEST(VehicleAssignmentRequestBuilderTest, EmptyPickupStationMeansNoDistanceEstimates) {
  const StationCatalog catalog = MakeStations();
  auto inputs = MakeInputs({MakeVehicle("v1")});
  inputs.pickup_station_id.clear();
  inputs.catalog = &catalog;
  inputs.catalog_loaded = true;
  const auto request = BuildVehicleAssignmentRequest(inputs);
  EXPECT_TRUE(request.distance_to_pickup_m.empty());
}

TEST(VehicleAssignmentRequestBuilderTest, DistanceUsesRouteNetworkCost) {
  // dock -> receiving (3.0) -> storage_a (1.0) = 4.0m 路网成本；
  // 欧氏距离只有 2.0m —— 断言用的是路网值（评审问题 3）
  const StationCatalog catalog = MakeStations();
  auto inputs = MakeInputs({MakeVehicle("v1")});
  inputs.catalog = &catalog;
  inputs.catalog_loaded = true;
  auto v = MakeVehicle("v1");
  v.current_station_id = "dock";
  inputs.vehicles = {v};
  const auto request = BuildVehicleAssignmentRequest(inputs);
  ASSERT_EQ(request.distance_to_pickup_m.count("v1"), 1u);
  EXPECT_DOUBLE_EQ(request.distance_to_pickup_m.at("v1"), 4.0);
}

TEST(VehicleAssignmentRequestBuilderTest, UnreachableVehicleHasNoDistanceEntry) {
  // 目标站点不在目录里：EstimateVehicleApproachCost 返回 nullopt ->
  // 不写距离项（核心据此把该车判为"距离不可算"，绝不盲派）
  const StationCatalog catalog = MakeStations();
  auto inputs = MakeInputs({MakeVehicle("v1")});
  inputs.pickup_station_id = "nonexistent";
  inputs.catalog = &catalog;
  inputs.catalog_loaded = true;
  auto v = MakeVehicle("v1");
  v.current_station_id = "dock";
  inputs.vehicles = {v};
  const auto request = BuildVehicleAssignmentRequest(inputs);
  EXPECT_TRUE(request.distance_to_pickup_m.empty());
}

TEST(VehicleAssignmentRequestBuilderTest, EmptyFleetProducesEmptyRequest) {
  const auto request = BuildVehicleAssignmentRequest(MakeInputs({}));
  EXPECT_TRUE(request.vehicles.empty());
  EXPECT_TRUE(request.distance_to_pickup_m.empty());
}

TEST(VehicleAssignmentRequestBuilderTest, PreservesDynamicStateFields) {
  auto v = MakeVehicle("v1", 42.0);
  v.online = false;
  v.busy = true;
  v.position_valid = true;
  v.position_x = 1.5;
  v.position_y = -2.5;
  v.current_station_id = "dock";
  v.quarantined = true;
  const auto request = BuildVehicleAssignmentRequest(MakeInputs({v}));
  ASSERT_EQ(request.vehicles.size(), 1u);
  const auto& out = request.vehicles[0];
  EXPECT_FALSE(out.online);
  EXPECT_TRUE(out.busy);
  EXPECT_TRUE(out.quarantined);
  EXPECT_DOUBLE_EQ(out.battery_percent, 42.0);
  EXPECT_DOUBLE_EQ(out.position_x, 1.5);
  EXPECT_DOUBLE_EQ(out.position_y, -2.5);
  EXPECT_EQ(out.current_station_id, "dock");
}
