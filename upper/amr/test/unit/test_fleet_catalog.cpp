// 车队目录（fleet.yaml）与选车路网成本的单测。
//
// 覆盖点：
//   1. fleet.yaml 解析：robots[]、backend 字段（local_sim / vda5050）
//   2. 目录校验：重复 id、非法 backend、非法电量/载重
//   3. EstimateVehicleApproachCost 的三条规则（评审问题 3：路网成本，不用欧氏距离）：
//      规则 1 车挂在已知站点 -> 该站点到目标的路网成本
//      规则 2 车在站点之间   -> 最近站点直线接驳 + 路网成本
//      规则 3 目标不存在     -> nullopt（距离不可算 -> 该车不合格，绝不盲派）

#include "amr_dispatcher_core/fleet/fleet_catalog.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>

using namespace amr_dispatcher_core::fleet;
using amr_dispatcher_core::catalog::CatalogStationEdge;
using amr_dispatcher_core::catalog::Station;
using amr_dispatcher_core::catalog::StationCatalog;

namespace {

class FleetCatalogTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dir_ = std::filesystem::temp_directory_path() /
           ("fleet_catalog_test_" +
            std::to_string(reinterpret_cast<std::uintptr_t>(this)));
    std::filesystem::create_directories(dir_);
  }
  void TearDown() override {
    std::error_code ec;
    std::filesystem::remove_all(dir_, ec);
  }

  std::filesystem::path WriteFile(const std::string& name, const std::string& content) {
    const auto path = dir_ / name;
    std::ofstream out(path);
    out << content;
    out.close();
    return path;
  }

  std::filesystem::path dir_;
};

// 三站点直线拓扑：dock --(3m)-- receiving --(1m)-- storage_a
StationCatalog MakeStations() {
  StationCatalog catalog;
  catalog.stations.push_back(Station{"dock", "map", 0.0, 0.0, 0.0});
  catalog.stations.push_back(Station{"receiving", "map", 1.0, 0.0, 0.0});
  catalog.stations.push_back(Station{"storage_a", "map", 2.0, 0.0, 0.0});
  catalog.edges.push_back(CatalogStationEdge{"dock", "receiving", 3.0, true, true});
  catalog.edges.push_back(CatalogStationEdge{"receiving", "storage_a", 1.0, true, true});
  return catalog;
}

}  // namespace

// ---- fleet.yaml 解析 ----

TEST_F(FleetCatalogTest, LoadFleetCatalog_ParsesBackendAndFields) {
  const auto path = WriteFile("fleet.yaml", R"(robots:
  - id: amr_01
    enabled: true
    state: available
    battery_voltage: 24.5
    max_payload_kg: 40.0
    current_station_id: dock
    capabilities: transport,station_transport
    backend: local_sim
  - id: amr_02
    enabled: true
    state: available
    battery_voltage: 23.0
    max_payload_kg: 30.0
    capabilities: transport
    backend: vda5050
)");

  const auto catalog = LoadFleetCatalog(path);

  ASSERT_TRUE(catalog.has_value());
  ASSERT_EQ(catalog->robots.size(), 2u);
  const auto* local = FindFleetRobot(*catalog, "amr_01");
  ASSERT_NE(local, nullptr);
  EXPECT_EQ(local->backend, "local_sim");
  EXPECT_EQ(local->current_station_id, "dock");
  EXPECT_TRUE(RobotHasCapability(*local, "station_transport"));
  const auto* vda = FindFleetRobot(*catalog, "amr_02");
  ASSERT_NE(vda, nullptr);
  EXPECT_EQ(vda->backend, "vda5050");
}

TEST_F(FleetCatalogTest, LoadFleetCatalog_MissingBackendDefaultsToLocalSim) {
  const auto path = WriteFile("fleet.yaml", R"(robots:
  - id: amr_01
    enabled: true
    state: available
    battery_voltage: 24.0
    max_payload_kg: 10.0
)");

  const auto catalog = LoadFleetCatalog(path);

  ASSERT_TRUE(catalog.has_value());
  ASSERT_EQ(catalog->robots.size(), 1u);
  EXPECT_EQ(catalog->robots.front().backend, "local_sim");
}

TEST_F(FleetCatalogTest, LoadFleetCatalog_UnknownBackendIsRejected) {
  const auto path = WriteFile("fleet.yaml", R"(robots:
  - id: amr_01
    enabled: true
    state: available
    battery_voltage: 24.0
    max_payload_kg: 10.0
    backend: telepresence
)");

  // 未知后端**不猜、不静默回落** -> 整份目录判负
  EXPECT_FALSE(LoadFleetCatalog(path).has_value());
}

TEST_F(FleetCatalogTest, LoadFleetCatalog_DuplicateIdIsRejected) {
  const auto path = WriteFile("fleet.yaml", R"(robots:
  - id: amr_01
    battery_voltage: 24.0
    max_payload_kg: 10.0
  - id: amr_01
    battery_voltage: 23.0
    max_payload_kg: 10.0
)");

  EXPECT_FALSE(LoadFleetCatalog(path).has_value());
}

TEST_F(FleetCatalogTest, LoadFleetCatalog_MissingFileReturnsNullopt) {
  EXPECT_FALSE(LoadFleetCatalog(dir_ / "does_not_exist.yaml").has_value());
}

TEST_F(FleetCatalogTest, ValidateFleetCatalog_RejectsInvalidBatteryAndPayload) {
  FleetCatalog catalog;
  catalog.robots.push_back(FleetRobot{"amr_01", true, "available", -1.0, 10.0, "dock", {}});
  std::string message;
  EXPECT_FALSE(ValidateFleetCatalog(catalog, &message));
  EXPECT_FALSE(message.empty());

  catalog.robots.front().battery_voltage = 24.0;
  catalog.robots.front().max_payload_kg = -5.0;
  EXPECT_FALSE(ValidateFleetCatalog(catalog, &message));
}

// ---- 选车路网成本（EstimateVehicleApproachCost）----

TEST_F(FleetCatalogTest, ApproachCost_VehicleOnKnownStation_UsesRouteDistance) {
  const auto catalog = MakeStations();

  // dock -> receiving -> storage_a：路网成本 3 + 1 = 4（不是欧氏距离 2）
  const auto cost = EstimateVehicleApproachCost(catalog, 0.0, 0.0, "dock", "storage_a");

  ASSERT_TRUE(cost.has_value());
  EXPECT_NEAR(*cost, 4.0, 1e-6);
}

TEST_F(FleetCatalogTest, ApproachCost_VehicleBetweenStations_AddsStraightLineShuttle) {
  const auto catalog = MakeStations();

  // 车挂在 dock（坐标 0,0），但把坐标写成 (0.5, 0.0)：此时按规则 1 走
  // 站点->目标的纯路网成本；车辆坐标不参与（车在已知站点上）
  const auto on_station = EstimateVehicleApproachCost(catalog, 0.5, 0.0, "dock", "storage_a");
  ASSERT_TRUE(on_station.has_value());
  EXPECT_NEAR(*on_station, 4.0, 1e-6);

  // 车不在已知站点上（vehicle_station_id 为空）：取最近站点 + 直线接驳
  // 坐标 (1.0, 1.0) 距 receiving(1,0) 为 1.0 m，receiving -> storage_a 为 1.0 m
  const auto off_station = EstimateVehicleApproachCost(catalog, 1.0, 1.0, "", "storage_a");
  ASSERT_TRUE(off_station.has_value());
  EXPECT_NEAR(*off_station, 2.0, 1e-6);
}

TEST_F(FleetCatalogTest, ApproachCost_UnknownTargetReturnsNullopt) {
  const auto catalog = MakeStations();

  // 目标站点不存在 -> 距离不可算 -> nullopt（该车在自动选车时不合格）
  EXPECT_FALSE(EstimateVehicleApproachCost(catalog, 0.0, 0.0, "dock", "nowhere").has_value());
  // 目标 id 为空同理
  EXPECT_FALSE(EstimateVehicleApproachCost(catalog, 0.0, 0.0, "dock", "").has_value());
}

TEST_F(FleetCatalogTest, ApproachCost_UnknownVehicleStationReturnsNullopt) {
  const auto catalog = MakeStations();

  // 车挂在一个目录里根本不存在的站点上 -> 路网成本算不出来 -> nullopt。
  // 这正是"距离不可算即不合格"的落点：绝不退化成"随便找个近的站点"。
  EXPECT_FALSE(
      EstimateVehicleApproachCost(catalog, 1.0, 1.0, "ghost_station", "storage_a").has_value());
}

TEST_F(FleetCatalogTest, ApproachCost_UnreachableTargetReturnsNullopt) {
  StationCatalog catalog;
  catalog.stations.push_back(Station{"isolated", "map", 0.0, 0.0, 0.0});
  catalog.stations.push_back(Station{"other", "map", 5.0, 0.0, 0.0});
  // 无边 -> 路网不通
  EXPECT_FALSE(EstimateVehicleApproachCost(catalog, 0.0, 0.0, "isolated", "other").has_value());
}
