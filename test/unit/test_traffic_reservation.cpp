#include "amr_dispatcher_core/dispatcher/traffic_reservation.hpp"

#include <gtest/gtest.h>

using namespace amr_dispatcher_core::dispatcher;

TEST(ResourceTableTest, BuildLockIdsRouteEdges) {
  const auto ids = ResourceTable::BuildLockIds({"A", "B", "C"}, false);
  EXPECT_EQ(ids.size(), 2u);
  // 字典序规范化：A__B / B__C
  EXPECT_EQ(ids[0], "route_edge:A__B");
  EXPECT_EQ(ids[1], "route_edge:B__C");
}

TEST(ResourceTableTest, BuildLockIdsWithNodes) {
  const auto ids = ResourceTable::BuildLockIds({"A", "B", "C"}, true);
  EXPECT_EQ(ids.size(), 5u);
}

TEST(ResourceTableTest, ReserveSuccess) {
  ResourceTable t;
  const auto r = t.Reserve("m1", {"route_edge:A__B", "route_node:B"});
  EXPECT_TRUE(r.success);
  EXPECT_EQ(r.newly_reserved.size(), 2u);
}

TEST(ResourceTableTest, ReserveAtomicRollback) {
  ResourceTable t;
  ASSERT_TRUE(t.Reserve("m1", {"route_edge:A__B"}).success);
  const auto r = t.Reserve("m2", {"route_edge:X__Y", "route_edge:A__B"});
  EXPECT_FALSE(r.success);
  // 失败方不应污染
  EXPECT_FALSE(t.Locked("route_edge:X__Y"));
  EXPECT_TRUE(t.Locked("route_edge:A__B"));
  EXPECT_EQ(t.Owner("route_edge:A__B").value(), "m1");
}

TEST(ResourceTableTest, BlockAndUnblock) {
  ResourceTable t;
  std::string msg;
  ASSERT_TRUE(t.Block("route_edge:A__B", "construction", &msg));
  EXPECT_TRUE(t.IsTrafficBlockOwner(t.Owner("route_edge:A__B").value()));
  ASSERT_TRUE(t.Unblock("route_edge:A__B", &msg));
  EXPECT_FALSE(t.Locked("route_edge:A__B"));
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
