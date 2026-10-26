#include "amr_dispatcher_core/dispatcher/mission_queue.hpp"

#include <gtest/gtest.h>

using namespace amr_dispatcher_core::dispatcher;

namespace {

Mission MakeMission(std::string id, int priority, std::uint64_t seq) {
  Mission m;
  m.id = std::move(id);
  m.order_id = "order_" + m.id;
  m.pickup_station = "A";
  m.dropoff_station = "B";
  m.priority = priority;
  m.sequence = seq;
  return m;
}

}  // namespace

TEST(MissionQueueTest, PushAndPopHighPriorityFirst) {
  MissionQueue q;
  ASSERT_TRUE(q.Push(MakeMission("a", 1, 1)).accepted);
  ASSERT_TRUE(q.Push(MakeMission("b", 5, 2)).accepted);
  ASSERT_TRUE(q.Push(MakeMission("c", 3, 3)).accepted);

  const auto r1 = q.PopNext();
  ASSERT_TRUE(r1.success);
  EXPECT_EQ(r1.mission->id, "b");
  const auto r2 = q.PopNext();
  ASSERT_TRUE(r2.success);
  EXPECT_EQ(r2.mission->id, "c");
  const auto r3 = q.PopNext();
  ASSERT_TRUE(r3.success);
  EXPECT_EQ(r3.mission->id, "a");
  const auto r4 = q.PopNext();
  EXPECT_TRUE(r4.queue_empty);
}

TEST(MissionQueueTest, RejectsDuplicateAndFull) {
  MissionQueue q{{.max_size = 2, .comparator = "priority_fifo"}};
  EXPECT_TRUE(q.Push(MakeMission("a", 0, 1)).accepted);
  EXPECT_FALSE(q.Push(MakeMission("a", 0, 2)).accepted);
  EXPECT_TRUE(q.Push(MakeMission("b", 0, 3)).accepted);
  EXPECT_FALSE(q.Push(MakeMission("c", 0, 4)).accepted);
}

TEST(MissionQueueTest, RejectEmptyId) {
  MissionQueue q;
  EXPECT_FALSE(q.Push(MakeMission("", 0, 1)).accepted);
}

TEST(MissionQueueTest, DeadlineComparator) {
  MissionQueue q{{.max_size = 16, .comparator = "earliest_deadline"}};
  Mission a = MakeMission("a", 0, 1);
  a.deadline_unix_ms = 2000;
  Mission b = MakeMission("b", 0, 2);
  b.deadline_unix_ms = 1000;
  Mission c = MakeMission("c", 0, 3);
  c.deadline_unix_ms = 3000;
  ASSERT_TRUE(q.Push(a).accepted);
  ASSERT_TRUE(q.Push(b).accepted);
  ASSERT_TRUE(q.Push(c).accepted);
  EXPECT_EQ(q.PopNext().mission->id, "b");
  EXPECT_EQ(q.PopNext().mission->id, "a");
  EXPECT_EQ(q.PopNext().mission->id, "c");
}

TEST(MissionQueueTest, PauseAndResumeOrder) {
  MissionQueue q;
  ASSERT_TRUE(q.Push(MakeMission("a", 0, 1)).accepted);
  q.PauseOrder("order_a");
  const auto blocked = q.PopNext();
  EXPECT_FALSE(blocked.success);
  EXPECT_TRUE(blocked.all_paused);
  q.ResumeOrder("order_a");
  ASSERT_TRUE(q.PopNext().success);
}

TEST(MissionQueueTest, CancelOrderAndMission) {
  MissionQueue q;
  ASSERT_TRUE(q.Push(MakeMission("a", 0, 1)).accepted);
  ASSERT_TRUE(q.Push(MakeMission("b", 0, 2)).accepted);
  EXPECT_TRUE(q.Cancel("a"));
  EXPECT_FALSE(q.Cancel("a"));
  EXPECT_EQ(q.size(), 1u);
  EXPECT_EQ(q.CancelOrder("order_b"), 1u);
  EXPECT_TRUE(q.empty());
}

TEST(MissionQueueTest, Reprioritize) {
  MissionQueue q;
  ASSERT_TRUE(q.Push(MakeMission("a", 1, 1)).accepted);
  ASSERT_TRUE(q.Push(MakeMission("b", 2, 2)).accepted);
  EXPECT_TRUE(q.Reprioritize("a", 10));
  EXPECT_EQ(q.Peek()->id, "a");
  EXPECT_FALSE(q.Reprioritize("nonexistent", 0));
}

TEST(MissionQueueTest, RejectUnknownComparatorFallsBack) {
  MissionQueue q{{.comparator = "nonsense"}};
  EXPECT_EQ(q.comparator_name(), "priority_fifo");
}

// ---- A6：队首阻塞规避（PeekN / PopById）----

TEST(MissionQueueTest, PeekN_ReturnsFirstNInDispatchOrder) {
  MissionQueue q;
  ASSERT_TRUE(q.Push(MakeMission("a", 1, 1)).accepted);
  ASSERT_TRUE(q.Push(MakeMission("b", 5, 2)).accepted);
  ASSERT_TRUE(q.Push(MakeMission("c", 3, 3)).accepted);

  const auto head = q.PeekN(2);
  ASSERT_EQ(head.size(), 2u);
  EXPECT_EQ(head[0].id, "b");  // 优先级最高者排在队首
  EXPECT_EQ(head[1].id, "c");
  // Peek 不改变队列
  EXPECT_EQ(q.size(), 3u);
}

TEST(MissionQueueTest, PeekN_SkipsPausedOrders) {
  MissionQueue q;
  ASSERT_TRUE(q.Push(MakeMission("a", 9, 1)).accepted);
  ASSERT_TRUE(q.Push(MakeMission("b", 8, 2)).accepted);
  q.PauseOrder("order_a");

  const auto head = q.PeekN(4);
  ASSERT_EQ(head.size(), 1u);
  EXPECT_EQ(head[0].id, "b");
}

TEST(MissionQueueTest, PeekN_ZeroOrEmptyReturnsNothing) {
  MissionQueue q;
  EXPECT_TRUE(q.PeekN(0).empty());
  EXPECT_TRUE(q.PeekN(8).empty());
  ASSERT_TRUE(q.Push(MakeMission("a", 1, 1)).accepted);
  EXPECT_TRUE(q.PeekN(0).empty());
  EXPECT_EQ(q.size(), 1u);
}

TEST(MissionQueueTest, PopById_DispatchesNonHeadMission) {
  MissionQueue q;
  ASSERT_TRUE(q.Push(MakeMission("a", 9, 1)).accepted);
  ASSERT_TRUE(q.Push(MakeMission("b", 8, 2)).accepted);

  // 队首 "a" 暂无合格车辆 -> 直接出后面的 "b"（队首阻塞规避）
  const auto popped = q.PopById("b");
  ASSERT_TRUE(popped.success);
  ASSERT_TRUE(popped.mission.has_value());
  EXPECT_EQ(popped.mission->id, "b");
  EXPECT_EQ(popped.mission->state, MissionState::kActive);
  EXPECT_EQ(q.size(), 1u);
  EXPECT_TRUE(q.Contains("a"));
}

TEST(MissionQueueTest, PopById_UnknownMissionFailsWithoutChangingQueue) {
  MissionQueue q;
  ASSERT_TRUE(q.Push(MakeMission("a", 1, 1)).accepted);

  const auto popped = q.PopById("nope");
  EXPECT_FALSE(popped.success);
  EXPECT_FALSE(popped.mission.has_value());
  EXPECT_FALSE(popped.message.empty());
  EXPECT_EQ(q.size(), 1u);
}

TEST(MissionQueueTest, PopById_PausedOrderFailsWithoutChangingQueue) {
  MissionQueue q;
  ASSERT_TRUE(q.Push(MakeMission("a", 1, 1)).accepted);
  q.PauseOrder("order_a");

  const auto popped = q.PopById("a");
  EXPECT_FALSE(popped.success);
  EXPECT_TRUE(popped.all_paused);
  EXPECT_EQ(q.size(), 1u);
  // 恢复后同一 id 可正常出队
  q.ResumeOrder("order_a");
  EXPECT_TRUE(q.PopById("a").success);
  EXPECT_TRUE(q.empty());
}

TEST(MissionQueueTest, PeekNThenPopById_SkipsUndispatchableHead) {
  // 端到端复现队首阻塞规避：前 N 个候选里挑第一个真正可派的任务
  MissionQueue q;
  ASSERT_TRUE(q.Push(MakeMission("head", 10, 1)).accepted);   // 无合格车 -> 跳过
  ASSERT_TRUE(q.Push(MakeMission("mid", 9, 2)).accepted);     // 也无合格车 -> 跳过
  ASSERT_TRUE(q.Push(MakeMission("tail", 8, 3)).accepted);    // 有合格车 -> 派它

  const auto candidates = q.PeekN(8);
  ASSERT_EQ(candidates.size(), 3u);
  EXPECT_EQ(candidates[0].id, "head");

  const auto popped = q.PopById("tail");
  ASSERT_TRUE(popped.success);
  EXPECT_EQ(q.size(), 2u);
  EXPECT_TRUE(q.Contains("head"));
  EXPECT_TRUE(q.Contains("mid"));
}

// ---------------------------------------------------------------------------
// 落盘快照与恢复（批次 2：状态持久化）
// ---------------------------------------------------------------------------

TEST(MissionQueueTest, SnapshotForPersistence_KeepsOriginalState) {
  // Snapshot() 会把 state 重算成 kPaused/kPending（面向话题展示），
  // 落盘必须用 SnapshotForPersistence()，否则往返后 state 被改写。
  MissionQueue q;
  Mission m = MakeMission("a", 1, 1);
  m.state = MissionState::kActive;  // 故意放一个非 kPending 状态
  ASSERT_TRUE(q.Push(m).accepted);  // Push 会归一化为 kPending
  const auto persisted = q.SnapshotForPersistence();
  ASSERT_EQ(persisted.size(), 1u);
  EXPECT_EQ(persisted[0].state, MissionState::kPending);
  EXPECT_EQ(persisted[0].sequence, 1u);
  EXPECT_EQ(persisted[0].priority, 1);
}

TEST(MissionQueueTest, Restore_RebuildsQueueInGivenOrder) {
  MissionQueue q(MissionQueue::Config{128, "priority_fifo"});
  std::vector<Mission> missions{MakeMission("first", 9, 5), MakeMission("second", 1, 6)};
  EXPECT_EQ(q.Restore(missions, {}), 2u);
  ASSERT_EQ(q.size(), 2u);
  // priority_fifo：高优先级先出
  auto pop = q.PopNext();
  ASSERT_TRUE(pop.success);
  EXPECT_EQ(pop.mission->id, "first");
}

TEST(MissionQueueTest, Restore_ReappliesPausedOrderMarkers) {
  MissionQueue q;
  std::vector<Mission> missions{MakeMission("a", 1, 1), MakeMission("b", 1, 2)};
  ASSERT_EQ(q.Restore(missions, {"order_a"}), 2u);
  EXPECT_TRUE(q.IsOrderPaused("order_a"));
  // 暂停单不出队，但仍在队列里（不丢单）
  const auto pop = q.PopNext();
  ASSERT_TRUE(pop.success);
  EXPECT_EQ(pop.mission->id, "b");
  EXPECT_TRUE(q.Contains("a"));
}

TEST(MissionQueueTest, Restore_SkipsDuplicateAndEmptyIds) {
  MissionQueue q;
  std::vector<Mission> missions{MakeMission("a", 1, 1), MakeMission("a", 1, 2),
                                MakeMission("", 1, 3), MakeMission("b", 1, 4)};
  EXPECT_EQ(q.Restore(missions, {}), 2u);
  EXPECT_TRUE(q.Contains("a"));
  EXPECT_TRUE(q.Contains("b"));
}

TEST(MissionQueueTest, Restore_RespectsCapacity) {
  MissionQueue q(MissionQueue::Config{2, "priority_fifo"});
  std::vector<Mission> missions{MakeMission("a", 1, 1), MakeMission("b", 1, 2),
                                MakeMission("c", 1, 3)};
  EXPECT_EQ(q.Restore(missions, {}), 2u);
  EXPECT_EQ(q.size(), 2u);
  EXPECT_FALSE(q.Contains("c"));
}

TEST(MissionQueueTest, Restore_ReplacesExistingContents) {
  MissionQueue q;
  ASSERT_TRUE(q.Push(MakeMission("stale", 1, 1)).accepted);
  std::vector<Mission> missions{MakeMission("fresh", 1, 2)};
  ASSERT_EQ(q.Restore(missions, {}), 1u);
  EXPECT_FALSE(q.Contains("stale"));
  EXPECT_TRUE(q.Contains("fresh"));
}

TEST(MissionQueueTest, SnapshotRestoreRoundTrip_PreservesDispatchOrder) {
  MissionQueue original;
  ASSERT_TRUE(original.Push(MakeMission("p_low", 1, 1)).accepted);
  ASSERT_TRUE(original.Push(MakeMission("p_high", 9, 2)).accepted);
  ASSERT_TRUE(original.Push(MakeMission("p_mid", 5, 3)).accepted);
  original.PauseOrder("order_p_mid");

  MissionQueue restored;
  ASSERT_EQ(restored.Restore(original.SnapshotForPersistence(), {"order_p_mid"}), 3u);

  const auto before = original.PeekN(8);
  const auto after = restored.PeekN(8);
  ASSERT_EQ(before.size(), after.size());
  for (std::size_t i = 0; i < before.size(); ++i) {
    EXPECT_EQ(before[i].id, after[i].id) << "dispatch order diverged at " << i;
  }
  // 暂停标记仍在：p_mid 不在可派候选里
  for (const auto & m : after) {
    EXPECT_NE(m.id, "p_mid");
  }
}

TEST(MissionQueueTest, RestoreAppend_AddsPausedBacklogAfterMarkersRestored) {
  // 场景：暂停订单下的任务先入队会被容量挤掉，所以要等暂停标记生效后再补回
  MissionQueue q(MissionQueue::Config{2, "priority_fifo"});
  // 暂停标记挂的是订单号：MakeMission 会把 order_id 设为 "order_<id>"
  ASSERT_EQ(q.Restore({MakeMission("live", 5, 1)}, {"order_paused_a"}), 1u);
  EXPECT_TRUE(q.IsOrderPaused("order_paused_a"));

  EXPECT_EQ(q.RestoreAppend({MakeMission("paused_a", 9, 2)}), 1u);
  EXPECT_EQ(q.size(), 2u);
  EXPECT_TRUE(q.Contains("paused_a"));
  // 暂停标记仍有效：补回的任务不出队
  const auto pop = q.PopNext();
  ASSERT_TRUE(pop.success);
  EXPECT_EQ(pop.mission->id, "live");
}

TEST(MissionQueueTest, RestoreAppend_SkipsDuplicatesAndRespectsCapacity) {
  MissionQueue q(MissionQueue::Config{2, "priority_fifo"});
  ASSERT_EQ(q.Restore({MakeMission("a", 1, 1)}, {}), 1u);
  EXPECT_EQ(q.RestoreAppend({MakeMission("a", 1, 2), MakeMission("b", 1, 3),
                             MakeMission("c", 1, 4)}),
            1u);
  EXPECT_EQ(q.size(), 2u);
  EXPECT_TRUE(q.Contains("b"));
  EXPECT_FALSE(q.Contains("c"));
}
