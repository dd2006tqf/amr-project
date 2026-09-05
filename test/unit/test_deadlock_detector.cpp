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
