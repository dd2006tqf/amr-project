#include "amr_dispatcher_core/dispatcher/dispatch_state_snapshot.hpp"

#include <gtest/gtest.h>
#include <unistd.h>

#include <filesystem>
#include <fstream>
#include <string>

namespace fs = std::filesystem;
using namespace amr_dispatcher_core::dispatcher;

namespace {

fs::path TempPath(const std::string& name) {
  static std::uintptr_t counter = 0;
  return fs::temp_directory_path() /
         ("amr_dispatch_snapshot_" + std::to_string(::getpid()) + "_" +
          std::to_string(++counter) + "_" + name);
}

Mission MakeMission(const std::string& id, std::uint64_t sequence, int priority) {
  Mission m;
  m.id = id;
  m.order_id = "ord_" + id;
  m.fleet_id = "fleet_a";
  m.type = "transport";
  m.pickup_station = "station_pickup_A";
  m.dropoff_station = "station_dropoff_B";
  m.priority = priority;
  m.sequence = sequence;
  m.deadline_unix_ms = 1700000000000LL;
  m.expected_seconds = 42;
  m.state = MissionState::kPending;
  m.result = MissionResult::kNone;
  m.attempt = 2;
  m.reservation_id = "res_1";
  return m;
}

}  // namespace

TEST(DispatchStateSnapshotTest, SerializeRoundTripPreservesAllFields) {
  DispatchStateSnapshot snapshot;
  snapshot.map_hash = "fnv1a64_deadbeef";
  snapshot.next_sequence = 7;
  snapshot.queued_missions.push_back(MakeMission("m_1", 1, 5));
  snapshot.queued_missions.push_back(MakeMission("m_2", 2, 9));
  snapshot.paused_order_ids.push_back("ord_m_1");
  Mission active = MakeMission("m_3", 3, 1);
  active.state = MissionState::kActive;
  snapshot.active_missions.push_back(active);
  snapshot.quarantined_vehicle_ids.push_back("amr_01");

  std::string message;
  const auto parsed = DeserializeDispatchState(SerializeDispatchState(snapshot), &message);
  ASSERT_TRUE(parsed.has_value()) << message;
  EXPECT_EQ(parsed->map_hash, "fnv1a64_deadbeef");
  EXPECT_EQ(parsed->next_sequence, 7u);
  ASSERT_EQ(parsed->queued_missions.size(), 2u);
  EXPECT_EQ(parsed->queued_missions[0].id, "m_1");
  EXPECT_EQ(parsed->queued_missions[1].priority, 9);
  EXPECT_EQ(parsed->queued_missions[0].deadline_unix_ms, 1700000000000LL);
  EXPECT_EQ(parsed->queued_missions[0].expected_seconds, 42);
  EXPECT_EQ(parsed->queued_missions[0].fleet_id, "fleet_a");
  EXPECT_EQ(parsed->queued_missions[0].assigned_vehicle_id,
            snapshot.queued_missions[0].assigned_vehicle_id);
  EXPECT_EQ(parsed->queued_missions[0].reservation_id, "res_1");
  EXPECT_EQ(parsed->queued_missions[0].attempt, 2);
  ASSERT_EQ(parsed->paused_order_ids.size(), 1u);
  EXPECT_EQ(parsed->paused_order_ids[0], "ord_m_1");
  ASSERT_EQ(parsed->active_missions.size(), 1u);
  EXPECT_EQ(parsed->active_missions[0].state, MissionState::kActive);
  ASSERT_EQ(parsed->quarantined_vehicle_ids.size(), 1u);
  EXPECT_EQ(parsed->quarantined_vehicle_ids[0], "amr_01");
}

TEST(DispatchStateSnapshotTest, EmptySnapshotRoundTrip) {
  DispatchStateSnapshot snapshot;
  snapshot.map_hash = "fnv1a64_0";
  std::string message;
  const auto parsed = DeserializeDispatchState(SerializeDispatchState(snapshot), &message);
  ASSERT_TRUE(parsed.has_value()) << message;
  EXPECT_TRUE(parsed->queued_missions.empty());
  EXPECT_TRUE(parsed->paused_order_ids.empty());
  EXPECT_TRUE(parsed->active_missions.empty());
  EXPECT_TRUE(parsed->quarantined_vehicle_ids.empty());
}

TEST(DispatchStateSnapshotTest, RejectsGarbageText) {
  std::string message;
  EXPECT_FALSE(DeserializeDispatchState("", &message).has_value());
  EXPECT_FALSE(message.empty());
  EXPECT_FALSE(DeserializeDispatchState("this: is: not: valid: yaml", &message).has_value());
  EXPECT_FALSE(DeserializeDispatchState("- just\n- a\n- list\n", &message).has_value());
}

TEST(DispatchStateSnapshotTest, RejectsUnsupportedFormatVersion) {
  const std::string text =
      "format_version: 99\nmap_hash: x\nnext_sequence: 1\n";
  std::string message;
  EXPECT_FALSE(DeserializeDispatchState(text, &message).has_value());
  EXPECT_NE(message.find("format_version"), std::string::npos) << message;
}

TEST(DispatchStateSnapshotTest, RejectsUnknownMissionState) {
  const std::string text =
      "format_version: 1\nmap_hash: x\nnext_sequence: 2\n"
      "queued_missions:\n  - id: m_1\n    state: \"EXPLODED\"\n";
  std::string message;
  EXPECT_FALSE(DeserializeDispatchState(text, &message).has_value());
  EXPECT_NE(message.find("EXPLODED"), std::string::npos) << message;
}

TEST(DispatchStateSnapshotTest, RejectsMissionWithoutId) {
  const std::string text =
      "format_version: 1\nmap_hash: x\nnext_sequence: 2\n"
      "queued_missions:\n  - priority: 3\n";
  std::string message;
  EXPECT_FALSE(DeserializeDispatchState(text, &message).has_value());
  EXPECT_NE(message.find("id"), std::string::npos) << message;
}

// 关键安全属性：游标不得回退到已有 sequence 之下，否则恢复后新任务会与旧任务撞号。
TEST(DispatchStateSnapshotTest, NextSequenceNeverRegressesBelowSeenSequences) {
  const std::string text =
      "format_version: 1\nmap_hash: x\nnext_sequence: 2\n"
      "queued_missions:\n  - id: m_1\n    sequence: 50\n"
      "active_missions:\n  - id: m_2\n    sequence: 77\n";
  std::string message;
  const auto parsed = DeserializeDispatchState(text, &message);
  ASSERT_TRUE(parsed.has_value()) << message;
  EXPECT_EQ(parsed->next_sequence, 78u);
}

TEST(DispatchStateSnapshotTest, AtomicWriteThenLoad) {
  const fs::path path = TempPath("snapshot.yaml");
  DispatchStateSnapshot snapshot;
  snapshot.map_hash = "fnv1a64_abc";
  snapshot.next_sequence = 3;
  snapshot.queued_missions.push_back(MakeMission("m_1", 1, 4));

  std::string message;
  ASSERT_TRUE(WriteDispatchStateAtomic(path, snapshot, &message)) << message;
  // 原子写不留 tmp
  EXPECT_FALSE(fs::exists(path.string() + ".tmp"));

  const auto loaded = LoadDispatchState(path, &message);
  ASSERT_TRUE(loaded.has_value()) << message;
  EXPECT_EQ(loaded->queued_missions.size(), 1u);
  EXPECT_EQ(loaded->queued_missions[0].id, "m_1");
  fs::remove(path);
}

TEST(DispatchStateSnapshotTest, LoadMissingFileReportsNotFound) {
  std::string message;
  EXPECT_FALSE(LoadDispatchState(TempPath("does_not_exist.yaml"), &message).has_value());
  EXPECT_EQ(message, "not_found");
}

TEST(DispatchStateSnapshotTest, AtomicWriteReplacesPreviousSnapshot) {
  const fs::path path = TempPath("replace.yaml");
  std::string message;
  DispatchStateSnapshot first;
  first.map_hash = "v1";
  first.next_sequence = 2;
  ASSERT_TRUE(WriteDispatchStateAtomic(path, first, &message)) << message;

  DispatchStateSnapshot second;
  second.map_hash = "v2";
  second.next_sequence = 9;
  ASSERT_TRUE(WriteDispatchStateAtomic(path, second, &message)) << message;

  const auto loaded = LoadDispatchState(path, &message);
  ASSERT_TRUE(loaded.has_value()) << message;
  EXPECT_EQ(loaded->map_hash, "v2");
  EXPECT_EQ(loaded->next_sequence, 9u);
  fs::remove(path);
}

// 回读校验：磁盘上若已是坏内容（模拟半截写），Load 必须明确失败而不是返回半套状态。
TEST(DispatchStateSnapshotTest, LoadRejectsCorruptedFile) {
  const fs::path path = TempPath("corrupt.yaml");
  {
    std::ofstream out(path);
    out << "format_version: 1\nqueued_missions:\n  - id: [unclosed\n";
  }
  std::string message;
  EXPECT_FALSE(LoadDispatchState(path, &message).has_value());
  EXPECT_FALSE(message.empty());
  fs::remove(path);
}

TEST(DispatchStateSnapshotTest, WriteCreatesParentDirectories) {
  const fs::path dir = TempPath("nested_dir");
  const fs::path path = dir / "sub" / "snapshot.yaml";
  DispatchStateSnapshot snapshot;
  snapshot.map_hash = "x";
  std::string message;
  ASSERT_TRUE(WriteDispatchStateAtomic(path, snapshot, &message)) << message;
  EXPECT_TRUE(fs::exists(path));
  fs::remove_all(dir);
}
