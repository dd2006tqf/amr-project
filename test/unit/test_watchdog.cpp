#include "amr_dispatcher_core/safety/watchdog.hpp"

#include <gtest/gtest.h>
#include <thread>

using namespace amr_dispatcher_core::safety;
using namespace std::chrono_literals;

TEST(WatchdogTest, HealthyNodesProduceNoEvents) {
  SafetyWatchdog w({.heartbeat_timeout = 100ms, .soft_timeout = 500ms});
  w.FeedHeartbeat("a");
  w.FeedHeartbeat("b");
  EXPECT_TRUE(w.Inspect({"a", "b"}).empty());
}

TEST(WatchdogTest, HeartbeatTimeoutEmitsWarn) {
  SafetyWatchdog w({.heartbeat_timeout = 50ms, .soft_timeout = 500ms});
  w.FeedHeartbeat("a");
  std::this_thread::sleep_for(80ms);
  auto evs = w.Inspect({"a"});
  ASSERT_EQ(evs.size(), 1u);
  EXPECT_EQ(evs[0].severity, SafetySeverity::kWarn);
}

TEST(WatchdogTest, SoftTimeoutEmitsCritical) {
  SafetyWatchdog w({.heartbeat_timeout = 50ms, .soft_timeout = 80ms});
  w.FeedHeartbeat("a");
  std::this_thread::sleep_for(120ms);
  auto evs = w.Inspect({"a"});
  ASSERT_EQ(evs.size(), 1u);
  EXPECT_EQ(evs[0].severity, SafetySeverity::kCritical);
}

TEST(WatchdogTest, UnseenNodeIsCritical) {
  SafetyWatchdog w({});
  auto evs = w.Inspect({"never_seen"});
  ASSERT_EQ(evs.size(), 1u);
  EXPECT_EQ(evs[0].severity, SafetySeverity::kCritical);
}

TEST(WatchdogTest, ForgetRemovesTracking) {
  SafetyWatchdog w({});
  w.FeedHeartbeat("a");
  w.Forget("a");
  auto evs = w.Inspect({"a"});
  ASSERT_EQ(evs.size(), 1u);
  EXPECT_EQ(evs[0].severity, SafetySeverity::kCritical);  // 未见过 = critical
}
