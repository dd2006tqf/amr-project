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
