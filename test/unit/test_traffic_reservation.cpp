#include "amr_dispatcher_core/dispatcher/traffic_reservation.hpp"

#include <gtest/gtest.h>

using namespace amr_dispatcher_core::dispatcher;
using namespace std::chrono_literals;

TEST(ResourceTableTest, BuildLockIdsRouteEdges) {
  const auto locks = ResourceTable::BuildLockIds({"A", "B", "C"}, false);
  ASSERT_EQ(locks.size(), 2u);
  EXPECT_EQ(locks[0], "route_edge:A__B");
  EXPECT_EQ(locks[1], "route_edge:B__C");
}

TEST(ResourceTableTest, BuildLockIdsWithNodes) {
  const auto locks = ResourceTable::BuildLockIds({"A", "B"}, true);
  ASSERT_EQ(locks.size(), 3u);
  EXPECT_EQ(locks[0], "route_node:A");
  EXPECT_EQ(locks[1], "route_node:B");
  EXPECT_EQ(locks[2], "route_edge:A__B");
}

TEST(ResourceTableTest, ReserveSuccess) {
  ResourceTable t;
  const auto r = t.Reserve("m1", {"route_edge:A__B", "route_edge:B__C"});
  EXPECT_TRUE(r.success);
  EXPECT_EQ(r.newly_reserved.size(), 2u);
  EXPECT_TRUE(t.Locked("route_edge:A__B"));
  EXPECT_EQ(t.Owner("route_edge:A__B"), "m1");
}

TEST(ResourceTableTest, ReserveAtomicRollback) {
  ResourceTable t;
  ASSERT_TRUE(t.Reserve("m1", {"route_edge:B__C"}).success);
  const auto r = t.Reserve("m2", {"route_edge:A__B", "route_edge:B__C"});
  EXPECT_FALSE(r.success);
  EXPECT_FALSE(t.Locked("route_edge:A__B"));  // atomic rollback
  EXPECT_EQ(t.Owner("route_edge:B__C"), "m1");
}

TEST(ResourceTableTest, TimeSpaceReservationNoConflict) {
  ResourceTable t;
  const auto t0 = std::chrono::steady_clock::now();

  // Car 1 在 [0s, 10s] 预约路段 A__B
  std::string msg;
  EXPECT_TRUE(t.ReserveTimeSpace("car_1", "route_edge:A__B", t0, t0 + 10s, &msg));

  // Car 2 在 [15s, 25s] 错峰预约同一路段 A__B -> 应当成功！
  EXPECT_TRUE(t.ReserveTimeSpace("car_2", "route_edge:A__B", t0 + 15s, t0 + 25s, &msg));
}

TEST(ResourceTableTest, TimeSpaceReservationOverlapConflict) {
  ResourceTable t;
  const auto t0 = std::chrono::steady_clock::now();

  // Car 1 在 [0s, 10s] 预约路段 A__B
  std::string msg;
  EXPECT_TRUE(t.ReserveTimeSpace("car_1", "route_edge:A__B", t0, t0 + 10s, &msg));

  // Car 2 试图在 [5s, 15s] 预约路段 A__B -> 发生时空重叠，应当被拦截！
  EXPECT_FALSE(t.ReserveTimeSpace("car_2", "route_edge:A__B", t0 + 5s, t0 + 15s, &msg));
  EXPECT_NE(msg.find("time-space contention"), std::string::npos);
}

TEST(ResourceTableTest, BlockAndUnblock) {
  ResourceTable t;
  std::string msg;
  EXPECT_TRUE(t.Block("route_edge:A__B", "maintenance", &msg));
  EXPECT_TRUE(t.Locked("route_edge:A__B"));
  EXPECT_FALSE(t.Reserve("m1", {"route_edge:A__B"}).success);

  EXPECT_TRUE(t.Unblock("route_edge:A__B", &msg));
  EXPECT_FALSE(t.Locked("route_edge:A__B"));
  EXPECT_TRUE(t.Reserve("m1", {"route_edge:A__B"}).success);
}

TEST(ResourceTableTest, BlockFailsIfAlreadyHeld) {
  ResourceTable t;
  ASSERT_TRUE(t.Reserve("m1", {"route_edge:A__B"}).success);
  std::string msg;
  EXPECT_FALSE(t.Block("route_edge:A__B", "construction", &msg));
  EXPECT_FALSE(msg.empty());
}

TEST(ResourceTableTest, UnblockFailsOnMissionReservation) {
  ResourceTable t;
  ASSERT_TRUE(t.Reserve("m1", {"route_edge:A__B"}).success);
  std::string msg;
  EXPECT_FALSE(t.Unblock("route_edge:A__B", &msg));
  EXPECT_FALSE(msg.empty());
}

TEST(ResourceTableTest, ReleaseRemovesAllLocksForMission) {
  ResourceTable t;
  ASSERT_TRUE(t.Reserve("m1", {"route_edge:A__B", "route_node:B", "route_edge:C__D"}).success);
  const auto r = t.Release("m1");
  EXPECT_TRUE(r.released);
  EXPECT_TRUE(t.Snapshot().empty());
}

TEST(ResourceTableTest, ParseRouteEdgeRoundTrip) {
  const auto parsed = ResourceTable::ParseRouteEdge("route_edge:A__B");
  ASSERT_TRUE(parsed.has_value());
  EXPECT_EQ(parsed->first, "A");
  EXPECT_EQ(parsed->second, "B");
  EXPECT_FALSE(ResourceTable::ParseRouteEdge("route_node:A").has_value());
}

TEST(ResourceTableTest, SnapshotIsSorted) {
  ResourceTable t;
  ASSERT_TRUE(t.Reserve("m1", {"route_edge:C__D", "route_edge:A__B"}).success);
  const auto snap = t.Snapshot();
  ASSERT_EQ(snap.size(), 2u);
  EXPECT_LT(snap[0].first, snap[1].first);
}
