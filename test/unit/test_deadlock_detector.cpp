#include "amr_dispatcher_core/dispatcher/deadlock_detector.hpp"

#include <gtest/gtest.h>

using namespace amr_dispatcher_core::dispatcher;
using namespace std::chrono_literals;

namespace {

Mission Active(std::string id) {
  Mission m;
  m.id = std::move(id);
  m.state = MissionState::kActive;
  return m;
}

}  // namespace

TEST(DeadlockDetectorTest, EmptySnapshotNoFindings) {
  DeadlockDetector d;
  EXPECT_TRUE(d.Detect({}).empty());
}

TEST(DeadlockDetectorTest, TrueWaitForGraphCycleDetection) {
  DeadlockDetector d;
  DeadlockSnapshot snap;

  // 场景：两台车发生经典交叉死锁：
  // Robot A 持有 Edge 1，申请等待 Edge 2
  // Robot B 持有 Edge 2，申请等待 Edge 1
  snap.allocation_graph.held_by["edge_1"] = "robot_A";
  snap.allocation_graph.held_by["edge_2"] = "robot_B";

  snap.allocation_graph.waiting_for["robot_A"] = {"edge_2"};
  snap.allocation_graph.waiting_for["robot_B"] = {"edge_1"};

  snap.active_missions = {Active("robot_A"), Active("robot_B")};

  const auto findings = d.Detect(snap);
  ASSERT_FALSE(findings.empty());
  EXPECT_EQ(findings[0].kind, DeadlockKind::kMutualBlock);
  EXPECT_EQ(findings[0].mission_ids.size(), 2u);
  EXPECT_NE(findings[0].description.find("Cycle detected"), std::string::npos);
}

TEST(DeadlockDetectorTest, MultiNodeCircularWaitCycle) {
  DeadlockDetector d;
  ResourceAllocationModel model;

  // 3车环形死锁：A等B持有的R2，B等C持有的R3，C等A持有的R1
  model.held_by["R1"] = "car_A";
  model.held_by["R2"] = "car_B";
  model.held_by["R3"] = "car_C";

  model.waiting_for["car_A"] = {"R2"};
  model.waiting_for["car_B"] = {"R3"};
  model.waiting_for["car_C"] = {"R1"};

  const auto cycles = d.DetectCycles(model);
  ASSERT_EQ(cycles.size(), 1u);
  EXPECT_EQ(cycles[0].size(), 3u);
}

TEST(DeadlockDetectorTest, NoCycleWhenWaitGraphIsDAG) {
  DeadlockDetector d;
  ResourceAllocationModel model;

  // 有向无环图 (DAG)：A 等 B，B 等 C，C 不等待任何资源 -> 无死锁
  model.held_by["R1"] = "car_B";
  model.held_by["R2"] = "car_C";

  model.waiting_for["car_A"] = {"R1"};
  model.waiting_for["car_B"] = {"R2"};

  const auto cycles = d.DetectCycles(model);
  EXPECT_TRUE(cycles.empty());
}

TEST(DeadlockDetectorTest, TooManyActiveMissions) {
  DeadlockDetector d({.max_active_missions = 2});
  DeadlockSnapshot snap;
  snap.active_missions = {Active("a"), Active("b"), Active("c")};
  const auto findings = d.Detect(snap);
  ASSERT_FALSE(findings.empty());
  EXPECT_EQ(findings[0].kind, DeadlockKind::kMutualBlock);
  EXPECT_EQ(findings[0].mission_ids.size(), 3u);
}

TEST(DeadlockDetectorTest, IdleActiveDetected) {
  DeadlockDetector d({.max_active_missions = 100,
                     .active_idle_threshold = 100ms});
  DeadlockSnapshot snap;
  snap.active_missions = {Active("a")};
  snap.last_progress["a"] = std::chrono::steady_clock::now() - 500ms;
  const auto findings = d.Detect(snap);
  ASSERT_FALSE(findings.empty());
  EXPECT_EQ(findings[0].kind, DeadlockKind::kIdleActive);
  EXPECT_EQ(findings[0].mission_ids.front(), "a");
}

TEST(DeadlockDetectorTest, LongReservationHoldDetected) {
  DeadlockDetector d({.max_active_missions = 100,
                     .reservation_hold_threshold = 100ms});
  DeadlockSnapshot snap;
  Mission a = Active("a");
  a.reservation_id = "edge_1_2";
  snap.active_missions = {a};
  snap.reservation_acquired_at["edge_1_2"] = std::chrono::steady_clock::now() - 500ms;
  snap.reserved_resource_ids.insert("edge_1_2");
  const auto findings = d.Detect(snap);
  ASSERT_FALSE(findings.empty());
  EXPECT_EQ(findings[0].kind, DeadlockKind::kLongReservationHold);
}
