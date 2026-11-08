#include "amr_dispatcher_core/dispatcher/deadlock_snapshot_assembly.hpp"

#include <gtest/gtest.h>

using namespace amr_dispatcher_core::dispatcher;
using Clock = std::chrono::steady_clock;

namespace {

Mission MakeMission(const std::string& id) {
  Mission m;
  m.id = id;
  m.order_id = "ord_" + id;
  m.state = MissionState::kActive;
  return m;
}

DeadlockSnapshotSources MakeSources() {
  DeadlockSnapshotSources sources;
  sources.active_missions["m1"] = MakeMission("m1");
  sources.active_missions["m2"] = MakeMission("m2");
  sources.last_progress["m1"] = Clock::now();
  sources.reservation_acquired_at["m1"] = Clock::now();
  sources.reserved_resources = {{"route_edge:A__B", "m1"}, {"route_node:C", "m1"}};
  sources.pending_wait["m2"] = {"route_edge:A__B"};
  sources.queued_mission_ids = {"m2"};
  return sources;
}

}  // namespace

TEST(DeadlockSnapshotAssemblyTest, AssemblesActiveMissionsAndGraph) {
  const auto assembly = AssembleDeadlockSnapshot(MakeSources());
  const auto& snap = assembly.snapshot;
  EXPECT_EQ(snap.active_missions.size(), 2u);
  EXPECT_EQ(snap.allocation_graph.held_by.at("route_edge:A__B"), "m1");
  EXPECT_EQ(snap.allocation_graph.held_by.at("route_node:C"), "m1");
  EXPECT_EQ(snap.reserved_resource_ids.count("route_edge:A__B"), 1u);
  ASSERT_EQ(snap.allocation_graph.waiting_for.count("m2"), 1u);
  EXPECT_EQ(snap.allocation_graph.waiting_for.at("m2").size(), 1u);
  EXPECT_EQ(snap.reservation_acquired_at.count("m1"), 1u);
  EXPECT_TRUE(assembly.stale_wait_mission_ids.empty());
}

// 关键回归：幽灵等待者（已不在队列里的任务）不得进 WFG，否则会凭空造出一个环。
TEST(DeadlockSnapshotAssemblyTest, StaleWaiterIsExcludedAndReported) {
  auto sources = MakeSources();
  sources.pending_wait["ghost"] = {"route_node:C"};  // 已不在队列
  const auto assembly = AssembleDeadlockSnapshot(sources);
  EXPECT_EQ(assembly.snapshot.allocation_graph.waiting_for.count("ghost"), 0u);
  ASSERT_EQ(assembly.stale_wait_mission_ids.size(), 1u);
  EXPECT_EQ(assembly.stale_wait_mission_ids[0], "ghost");
}

// 纯函数契约：组包不得修改输入（原实现是就地 erase，读操作会改状态）
TEST(DeadlockSnapshotAssemblyTest, DoesNotMutateInputs) {
  auto sources = MakeSources();
  sources.pending_wait["ghost"] = {"route_node:C"};
  const auto before_pending = sources.pending_wait;
  const auto before_active = sources.active_missions.size();
  (void)AssembleDeadlockSnapshot(sources);
  EXPECT_EQ(sources.pending_wait.size(), before_pending.size());
  EXPECT_EQ(sources.pending_wait.at("ghost").size(), 1u);
  EXPECT_EQ(sources.active_missions.size(), before_active);
}

TEST(DeadlockSnapshotAssemblyTest, StaleIdsAreSortedForDeterministicOutput) {
  auto sources = MakeSources();
  sources.pending_wait["zz_ghost"] = {"r1"};
  sources.pending_wait["aa_ghost"] = {"r2"};
  sources.pending_wait["mm_ghost"] = {"r3"};
  const auto assembly = AssembleDeadlockSnapshot(sources);
  ASSERT_EQ(assembly.stale_wait_mission_ids.size(), 3u);
  EXPECT_EQ(assembly.stale_wait_mission_ids[0], "aa_ghost");
  EXPECT_EQ(assembly.stale_wait_mission_ids[1], "mm_ghost");
  EXPECT_EQ(assembly.stale_wait_mission_ids[2], "zz_ghost");
}

TEST(DeadlockSnapshotAssemblyTest, MissingProgressUsesDefaultTimePoint) {
  auto sources = MakeSources();
  sources.active_missions["m3"] = MakeMission("m3");  // 没有 last_progress 记录
  const auto assembly = AssembleDeadlockSnapshot(sources);
  ASSERT_EQ(assembly.snapshot.last_progress.count("m3"), 1u);
  EXPECT_EQ(assembly.snapshot.last_progress.at("m3"), Clock::time_point{});
}

TEST(DeadlockSnapshotAssemblyTest, FilterDisabledKeepsAllWaiters) {
  // 队列尚未建立（原实现 queue_ == nullptr 分支）时不做幽灵判定
  auto sources = MakeSources();
  sources.filter_stale_waiters = false;
  sources.pending_wait["ghost"] = {"route_node:C"};
  sources.queued_mission_ids.clear();
  const auto assembly = AssembleDeadlockSnapshot(sources);
  EXPECT_TRUE(assembly.stale_wait_mission_ids.empty());
  ASSERT_EQ(assembly.snapshot.allocation_graph.waiting_for.count("ghost"), 1u);
  ASSERT_EQ(assembly.snapshot.allocation_graph.waiting_for.count("m2"), 1u);
}

TEST(DeadlockSnapshotAssemblyTest, EmptySourcesProduceEmptySnapshot) {
  const DeadlockSnapshotSources sources;
  const auto assembly = AssembleDeadlockSnapshot(sources);
  EXPECT_TRUE(assembly.snapshot.active_missions.empty());
  EXPECT_TRUE(assembly.snapshot.allocation_graph.held_by.empty());
  EXPECT_TRUE(assembly.snapshot.allocation_graph.waiting_for.empty());
  EXPECT_TRUE(assembly.snapshot.reserved_resource_ids.empty());
  EXPECT_TRUE(assembly.stale_wait_mission_ids.empty());
}

TEST(DeadlockSnapshotAssemblyTest, DetectorSeesTheCycleEndToEnd) {
  // 组包与检测器串起来：m1 持 A 等 C，m2 持 C 等 A -> kMutualBlock
  DeadlockSnapshotSources sources;
  sources.active_missions["m1"] = MakeMission("m1");
  sources.active_missions["m2"] = MakeMission("m2");
  sources.reserved_resources = {{"route_edge:A", "m1"}, {"route_node:C", "m2"}};
  sources.pending_wait["m1"] = {"route_node:C"};
  sources.pending_wait["m2"] = {"route_edge:A"};
  sources.queued_mission_ids = {"m1", "m2"};
  const auto assembly = AssembleDeadlockSnapshot(sources);

  DeadlockDetector detector(DeadlockDetectorConfig{16, std::chrono::seconds(10),
                                                   std::chrono::seconds(30)});
  const auto findings = detector.Detect(assembly.snapshot);
  bool saw_mutual_block = false;
  for (const auto& finding : findings) {
    if (finding.kind == DeadlockKind::kMutualBlock) {
      saw_mutual_block = true;
    }
  }
  EXPECT_TRUE(saw_mutual_block);
}
