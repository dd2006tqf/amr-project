#include "amr_dispatcher_core/dispatcher/traffic_reservation.hpp"

#include <gtest/gtest.h>

#include <algorithm>

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

// ---- 批量时空预约（全有或全无）----

TEST(ResourceTableTest, TimeSpaceBatchReservation_AllOrNothing) {
  ResourceTable t;
  const auto t0 = std::chrono::steady_clock::now();
  std::string msg;
  // 先占住一个资源
  ASSERT_TRUE(t.ReserveTimeSpace("m0", "route_edge:B__C", t0, t0 + 10s, &msg));

  // 批量预约：其中一个与 m0 冲突 -> 整批失败，且**不留下半套预约**
  const std::vector<std::pair<std::string, TimeInterval>> batch{
      {"route_edge:A__B", TimeInterval{t0, t0 + 10s}},
      {"route_edge:B__C", TimeInterval{t0, t0 + 10s}},
  };
  EXPECT_FALSE(t.ReserveTimeSpaceBatch("m1", batch, &msg));
  // m1 在 A__B 上不应有残留预约（能预约成功即证明没有残留）
  EXPECT_TRUE(t.ReserveTimeSpace("m1", "route_edge:A__B", t0, t0 + 10s, &msg));
}

TEST(ResourceTableTest, TimeSpaceBatchReservation_SucceedsWhenNoConflict) {
  ResourceTable t;
  const auto t0 = std::chrono::steady_clock::now();
  std::string msg;
  const std::vector<std::pair<std::string, TimeInterval>> batch{
      {"route_edge:A__B", TimeInterval{t0, t0 + 10s}},
      {"route_edge:B__C", TimeInterval{t0 + 12s, t0 + 22s}},
  };
  EXPECT_TRUE(t.ReserveTimeSpaceBatch("m1", batch, &msg));
  // 两份都已落表
  EXPECT_TRUE(t.ReserveTimeSpace("m2", "route_edge:A__B", t0, t0 + 10s, &msg) == false);
  EXPECT_TRUE(t.ReserveTimeSpace("m2", "route_edge:B__C", t0 + 12s, t0 + 22s, &msg) == false);
}

TEST(ResourceTableTest, TimeSpaceBatchReservation_RejectsStaticLock) {
  ResourceTable t;
  const auto t0 = std::chrono::steady_clock::now();
  ASSERT_TRUE(t.Reserve("m1", {"route_edge:A__B"}).success);
  std::string msg;
  const std::vector<std::pair<std::string, TimeInterval>> batch{
      {"route_edge:A__B", TimeInterval{t0, t0 + 10s}},
  };
  EXPECT_FALSE(t.ReserveTimeSpaceBatch("m2", batch, &msg));
}

TEST(ResourceTableTest, TimeSpaceReleaseResources_OnlyOwnEntries) {
  ResourceTable t;
  const auto t0 = std::chrono::steady_clock::now();
  std::string msg;
  ASSERT_TRUE(t.ReserveTimeSpace("m1", "route_edge:A__B", t0, t0 + 10s, &msg));
  ASSERT_TRUE(t.ReserveTimeSpace("m2", "route_edge:A__B", t0 + 20s, t0 + 30s, &msg));

  const auto released = t.ReleaseTimeSpaceResources("m1", {"route_edge:A__B"});
  ASSERT_EQ(released.size(), 1u);
  // m1 的区间已释放，m2 的仍在 -> 同一时段可再被预约
  EXPECT_TRUE(t.ReserveTimeSpace("m3", "route_edge:A__B", t0, t0 + 10s, &msg));
}

TEST(ResourceTableTest, TimeSpaceNoConflict) {
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

// A7(b)：同任务重复预约已持有锁 —— 成功但不计入 newly_reserved
TEST(ResourceTableTest, ReserveOwnExistingLockNotReportedNewlyReserved) {
  ResourceTable t;
  ASSERT_TRUE(t.Reserve("m1", {"route_edge:A__B"}).success);

  const auto r = t.Reserve("m1", {"route_edge:A__B", "route_edge:B__C"});

  EXPECT_TRUE(r.success);
  ASSERT_EQ(r.newly_reserved.size(), 1u);
  EXPECT_EQ(r.newly_reserved[0], "route_edge:B__C");
  EXPECT_EQ(t.Owner("route_edge:B__C"), "m1");
}

// A7(c)：id 列表含重复 —— 去重后只计入一次
TEST(ResourceTableTest, ReserveDuplicateIdsCountedOnce) {
  ResourceTable t;

  const auto r = t.Reserve("m1", {"route_edge:A__B", "route_edge:A__B", "route_node:B"});

  EXPECT_TRUE(r.success);
  ASSERT_EQ(r.newly_reserved.size(), 2u);
  EXPECT_EQ(std::count(r.newly_reserved.begin(), r.newly_reserved.end(), "route_edge:A__B"), 1);
  EXPECT_EQ(t.size(), 2u);
}

// A7(a)：过期时空预约（end < now）在下次 Reserve/Release 入口被剪除，表大小不随时间累积
TEST(ResourceTableTest, ExpiredTimeSpacePrunedOnNextReserveOrRelease) {
  ResourceTable t;
  const auto now = std::chrono::steady_clock::now();
  std::string msg;
  ASSERT_TRUE(t.ReserveTimeSpace("m1", "route_edge:A__B", now - 10s, now - 5s, &msg));
  ASSERT_TRUE(t.ReserveTimeSpace("m1", "route_edge:C__D", now - 8s, now - 3s, &msg));
  // M3 语义：ReserveTimeSpace 入口即剪过期——第二次插入时第一条（已过期）已被剪除
  EXPECT_EQ(t.TimeSpaceReservationCount(), 1u);

  // 静态 Reserve 入口触发 prune：全部过期条目清零
  ASSERT_TRUE(t.Reserve("m2", {"route_node:B"}).success);
  EXPECT_EQ(t.TimeSpaceReservationCount(), 0u);

  // Release 入口同样 prune：过期条目清除、未过期条目保留
  ASSERT_TRUE(t.ReserveTimeSpace("m1", "route_edge:A__B", now - 10s, now - 5s, &msg));
  ASSERT_TRUE(t.ReserveTimeSpace("m1", "route_edge:E__F", now + 5s, now + 15s, &msg));
  // M3 语义：插入未来时段时入口剪掉此前的过期条目 → 只剩未过期的 E__F
  EXPECT_EQ(t.TimeSpaceReservationCount(), 1u);
  t.Release("m2");
  EXPECT_EQ(t.TimeSpaceReservationCount(), 1u);
}

// M3：ReserveTimeSpace 入口同样剪除过期条目 —— 过期预约不得误拒时间重叠的回溯候选
TEST(ResourceTableTest, ReserveTimeSpacePrunesExpiredAtEntry) {
  ResourceTable t;
  const auto now = std::chrono::steady_clock::now();
  std::string msg;
  // m1 的过期条目（时段已结束）
  ASSERT_TRUE(t.ReserveTimeSpace("m1", "route_edge:A__B", now - 10s, now - 5s, &msg));
  // m2 的回溯候选与过期条目时间上重叠：入口 prune 后应成功（否则说明过期条目仍在参与冲突判断）
  EXPECT_TRUE(t.ReserveTimeSpace("m2", "route_edge:A__B", now - 7s, now - 2s, &msg))
      << "msg=" << msg;
}

// ---- B2：延期扩窗（ExtendTimeSpaceReservation）----

TEST(ResourceTableTest, ExtendTimeSpaceReservation_ExtendsOwnWindow) {
  ResourceTable t;
  const auto t0 = std::chrono::steady_clock::now();
  std::string msg;
  ASSERT_TRUE(t.ReserveTimeSpace("m1", "route_edge:A__B", t0, t0 + 10s, &msg));

  EXPECT_TRUE(t.ExtendTimeSpaceReservation("m1", "route_edge:A__B", t0 + 30s, &msg))
      << "msg=" << msg;
  EXPECT_EQ(t.TimeSpaceReservationCount(), 1u);
}

TEST(ResourceTableTest, ExtendTimeSpaceReservation_ShorterEndIsIdempotentNoOp) {
  ResourceTable t;
  const auto t0 = std::chrono::steady_clock::now();
  std::string msg;
  ASSERT_TRUE(t.ReserveTimeSpace("m1", "route_edge:A__B", t0, t0 + 30s, &msg));

  // 缩短到已有结束时间之前：无需延长，返回成功但不改表
  EXPECT_TRUE(t.ExtendTimeSpaceReservation("m1", "route_edge:A__B", t0 + 20s, &msg));
  EXPECT_NE(msg.find("no extension"), std::string::npos);
}

TEST(ResourceTableTest, ExtendTimeSpaceReservation_ConflictsWithOtherMission) {
  ResourceTable t;
  const auto t0 = std::chrono::steady_clock::now();
  std::string msg;
  ASSERT_TRUE(t.ReserveTimeSpace("m1", "route_edge:A__B", t0, t0 + 10s, &msg));
  ASSERT_TRUE(t.ReserveTimeSpace("m2", "route_edge:A__B", t0 + 20s, t0 + 40s, &msg));

  // m1 想延到 30s：会压到 m2 的窗口 -> 拒绝且不改动
  EXPECT_FALSE(t.ExtendTimeSpaceReservation("m1", "route_edge:A__B", t0 + 30s, &msg));
  EXPECT_NE(msg.find("m2"), std::string::npos);
  // m1 的窗口未被改动：再试一次延长到 12s（仍不冲突）应成功
  EXPECT_TRUE(t.ExtendTimeSpaceReservation("m1", "route_edge:A__B", t0 + 12s, &msg));
}

TEST(ResourceTableTest, ExtendTimeSpaceReservation_UnknownResourceOrMissionFails) {
  ResourceTable t;
  const auto t0 = std::chrono::steady_clock::now();
  std::string msg;
  ASSERT_TRUE(t.ReserveTimeSpace("m1", "route_edge:A__B", t0, t0 + 10s, &msg));

  EXPECT_FALSE(t.ExtendTimeSpaceReservation("m1", "route_edge:ZZ__YY", t0 + 30s, &msg));
  EXPECT_FALSE(t.ExtendTimeSpaceReservation("m_other", "route_edge:A__B", t0 + 30s, &msg));
}

// ---- time_space 模式：读单持有的资源（与 Snapshot() 的静态锁互补）----

TEST(ResourceTableTest, TimeSpaceResourcesOf_ListsOnlyOwnResources) {
  ResourceTable t;
  const auto t0 = std::chrono::steady_clock::now();
  std::string msg;
  ASSERT_TRUE(t.ReserveTimeSpace("m1", "route_edge:A__B", t0, t0 + 10s, &msg));
  ASSERT_TRUE(t.ReserveTimeSpace("m1", "route_node:B", t0, t0 + 10s, &msg));
  ASSERT_TRUE(t.ReserveTimeSpace("m2", "route_edge:C__D", t0, t0 + 10s, &msg));

  const auto mine = t.TimeSpaceResourcesOf("m1");
  ASSERT_EQ(mine.size(), 2u);
  EXPECT_EQ(mine[0], "route_edge:A__B");  // 按资源 id 排序
  EXPECT_EQ(mine[1], "route_node:B");
  // m2 看不到 m1 的资源
  EXPECT_EQ(t.TimeSpaceResourcesOf("m2").size(), 1u);
  EXPECT_TRUE(t.TimeSpaceResourcesOf("m_unknown").empty());
}

TEST(ResourceTableTest, TimeSpaceResourcesOf_EmptyWhenOnlyStaticLocks) {
  ResourceTable t;
  ASSERT_TRUE(t.Reserve("m1", {"route_edge:A__B"}).success);
  // 静态锁不在时空预约表里：time_space 模式读单持有资源必须读预约表
  EXPECT_TRUE(t.TimeSpaceResourcesOf("m1").empty());
  // 静态锁仍可从 Snapshot() 读到（两套视图互补）
  ASSERT_EQ(t.Snapshot().size(), 1u);
}
