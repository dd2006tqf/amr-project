#include "amr_dispatcher_core/workflow/dispatch_endpoint_resolution.hpp"

#include <gtest/gtest.h>

using amr_dispatcher_core::catalog::Station;
using amr_dispatcher_core::catalog::StationCatalog;
using amr_dispatcher_core::workflow::DispatchEndpointResolution;
using amr_dispatcher_core::workflow::DispatchEndpointRequest;
using amr_dispatcher_core::workflow::ResolveDispatchEndpoints;

namespace {

StationCatalog MakeCatalog() {
  StationCatalog catalog;
  catalog.stations.push_back(Station{"pickup_A", "map", 1.0, 2.0, 0.5});
  catalog.stations.push_back(Station{"dropoff_B", "map", 7.0, 8.0, 1.5});
  return catalog;
}

DispatchEndpointRequest MakeLoadedRequest(const StationCatalog* catalog) {
  DispatchEndpointRequest request;
  request.pickup_station = "pickup_A";
  request.dropoff_station = "dropoff_B";
  request.catalog_loaded = true;
  request.catalog = catalog;
  return request;
}

}  // namespace

TEST(DispatchEndpointResolutionTest, ResolvesBothStationsFromCatalog) {
  const StationCatalog catalog = MakeCatalog();
  const auto r = ResolveDispatchEndpoints(MakeLoadedRequest(&catalog));
  ASSERT_EQ(r.kind, DispatchEndpointResolution::Kind::kResolved);
  EXPECT_DOUBLE_EQ(r.start.x, 1.0);
  EXPECT_DOUBLE_EQ(r.start.y, 2.0);
  EXPECT_DOUBLE_EQ(r.start.yaw, 0.5);
  EXPECT_DOUBLE_EQ(r.goal.x, 7.0);
  EXPECT_DOUBLE_EQ(r.goal.y, 8.0);
  EXPECT_DOUBLE_EQ(r.goal.yaw, 1.5);
  EXPECT_FALSE(r.start_from_odom);
  EXPECT_TRUE(r.reason.empty());
}

// 没有 pickup = "车辆已在现场接单"：起点取车辆当前位姿。
TEST(DispatchEndpointResolutionTest, MissingPickupUsesOdomPose) {
  const StationCatalog catalog = MakeCatalog();
  auto request = MakeLoadedRequest(&catalog);
  request.pickup_station.clear();
  request.has_odom = true;
  request.current_pose = {3.5, -4.5, 0.25};
  const auto r = ResolveDispatchEndpoints(request);
  ASSERT_EQ(r.kind, DispatchEndpointResolution::Kind::kResolved);
  EXPECT_DOUBLE_EQ(r.start.x, 3.5);
  EXPECT_DOUBLE_EQ(r.start.y, -4.5);
  EXPECT_DOUBLE_EQ(r.start.yaw, 0.25);
  EXPECT_TRUE(r.start_from_odom);
}

// 无 pickup 且无里程计：起点保持零位（与旧实现一致，不额外报错）。
TEST(DispatchEndpointResolutionTest, MissingPickupWithoutOdomKeepsOrigin) {
  const StationCatalog catalog = MakeCatalog();
  auto request = MakeLoadedRequest(&catalog);
  request.pickup_station.clear();
  request.has_odom = false;
  const auto r = ResolveDispatchEndpoints(request);
  ASSERT_EQ(r.kind, DispatchEndpointResolution::Kind::kResolved);
  EXPECT_DOUBLE_EQ(r.start.x, 0.0);
  EXPECT_FALSE(r.start_from_odom);
}

// pickup 在目录里找不到但里程计可用：仍以里程计为起点（旧实现同）。
TEST(DispatchEndpointResolutionTest, UnknownPickupFallsBackToOdomWhenAvailable) {
  const StationCatalog catalog = MakeCatalog();
  auto request = MakeLoadedRequest(&catalog);
  request.pickup_station = "pickup_missing";
  request.has_odom = true;
  request.current_pose = {9.0, 9.0, 0.0};
  const auto r = ResolveDispatchEndpoints(request);
  ASSERT_EQ(r.kind, DispatchEndpointResolution::Kind::kResolved);
  EXPECT_TRUE(r.start_from_odom);
  EXPECT_DOUBLE_EQ(r.start.x, 9.0);
}

TEST(DispatchEndpointResolutionTest, EmptyDropoffFails) {
  const StationCatalog catalog = MakeCatalog();
  auto request = MakeLoadedRequest(&catalog);
  request.dropoff_station.clear();
  const auto r = ResolveDispatchEndpoints(request);
  EXPECT_EQ(r.kind, DispatchEndpointResolution::Kind::kUnknownDropoff);
  EXPECT_EQ(r.reason, "unknown dropoff station: ");
}

// 文案逐字：e2e 与事件消费者依赖 `unknown dropoff station: <id>`。
TEST(DispatchEndpointResolutionTest, UnknownDropoffReportsStationId) {
  const StationCatalog catalog = MakeCatalog();
  auto request = MakeLoadedRequest(&catalog);
  request.dropoff_station = "dropoff_missing";
  const auto r = ResolveDispatchEndpoints(request);
  EXPECT_EQ(r.kind, DispatchEndpointResolution::Kind::kUnknownDropoff);
  EXPECT_EQ(r.reason, "unknown dropoff station: dropoff_missing");
}

// 目录未加载：即使 dropoff 名字看着正常也必须判负，绝不盲派。
TEST(DispatchEndpointResolutionTest, CatalogNotLoadedFails) {
  const StationCatalog catalog = MakeCatalog();
  auto request = MakeLoadedRequest(&catalog);
  request.catalog_loaded = false;
  const auto r = ResolveDispatchEndpoints(request);
  EXPECT_EQ(r.kind, DispatchEndpointResolution::Kind::kUnknownDropoff);
}

TEST(DispatchEndpointResolutionTest, NullCatalogTreatedAsNotLoaded) {
  auto request = MakeLoadedRequest(nullptr);
  const auto r = ResolveDispatchEndpoints(request);
  EXPECT_EQ(r.kind, DispatchEndpointResolution::Kind::kUnknownDropoff);
}

TEST(DispatchEndpointResolutionTest, DefaultRequestFails) {
  const DispatchEndpointRequest request;
  EXPECT_EQ(ResolveDispatchEndpoints(request).kind,
            DispatchEndpointResolution::Kind::kUnknownDropoff);
}
