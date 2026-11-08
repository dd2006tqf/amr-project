#include "amr_dispatcher_core/chassis/link_recovery.hpp"

#include <gtest/gtest.h>

using namespace amr_dispatcher_core::chassis;
using namespace std::chrono_literals;

TEST(LinkRecoveryTest, IdleToWaitingOnFirstTick) {
  LinkRecoveryPolicy p({.initial_backoff = 100ms, .max_backoff = 1000ms,
                        .backoff_multiplier = 2.0, .jitter_ratio = 0.0,
                        .max_attempts = 3, .open_timeout = 10ms,
                        .open_attempt = [] { return true; }});
  EXPECT_EQ(p.state(), LinkRecoveryPolicy::State::kIdle);
  const auto wait = p.Tick();
  EXPECT_EQ(p.state(), LinkRecoveryPolicy::State::kWaiting);
  EXPECT_GE(wait.count(), 0);
}

TEST(LinkRecoveryTest, ExponentialBackoffSequence) {
  LinkRecoveryPolicy p({.initial_backoff = 100ms, .max_backoff = 1000ms,
                        .backoff_multiplier = 2.0, .jitter_ratio = 0.0,
                        .max_attempts = 10});
  p.Tick();  // idle -> waiting (backoff = initial = 100ms)
  EXPECT_EQ(p.current_backoff(), 100ms);
  // 模拟反复失败：每次 waiting -> attempting -> waiting with DecideBackoff(attempts)
  for (int i = 0; i < 3; ++i) {
    p.Tick();  // waiting -> attempting
    p.Tick();  // attempting -> waiting with new backoff = initial * 2^(attempts-1)
  }
  // attempts 增长 1->2->3；backoff: 200, 400, ...
  EXPECT_EQ(p.current_backoff(), 400ms);
}

TEST(LinkRecoveryTest, BackoffCappedAtMax) {
  LinkRecoveryPolicy p({.initial_backoff = 100ms, .max_backoff = 250ms,
                        .backoff_multiplier = 10.0, .jitter_ratio = 0.0});
  p.Tick();
  for (int i = 0; i < 4; ++i) {
    p.Tick();
    p.Tick();
  }
  EXPECT_LE(p.current_backoff().count(), 250);
}

TEST(LinkRecoveryTest, AttemptSuccessGoesToConnected) {
  LinkRecoveryPolicy p({.initial_backoff = 50ms, .max_backoff = 1000ms,
                        .backoff_multiplier = 2.0, .jitter_ratio = 0.0,
                        .max_attempts = 5,
                        .open_attempt = [] { return true; }});
  p.Tick();
  p.Tick();
  EXPECT_EQ(p.state(), LinkRecoveryPolicy::State::kAttempting);
  p.Tick();
  EXPECT_EQ(p.state(), LinkRecoveryPolicy::State::kConnected);
  EXPECT_EQ(p.attempts(), 1u);
}

TEST(LinkRecoveryTest, MaxAttemptsExhausts) {
  int n = 0;
  LinkRecoveryPolicy p({.initial_backoff = 10ms, .max_backoff = 100ms,
                        .backoff_multiplier = 2.0, .jitter_ratio = 0.0,
                        .max_attempts = 2,
                        .open_attempt = [&n] { ++n; return false; }});
  // 4 ticks: idle, waiting, attempting -> fail, waiting, attempting -> fail -> exhausted
  for (int i = 0; i < 5; ++i) p.Tick();
  EXPECT_EQ(p.state(), LinkRecoveryPolicy::State::kExhausted);
  EXPECT_EQ(p.attempts(), 2u);
}

TEST(LinkRecoveryTest, NotifyFailedAfterConnectedResumes) {
  LinkRecoveryPolicy p({.initial_backoff = 50ms, .open_attempt = [] { return true; }});
  p.Tick();  // idle
  p.Tick();  // waiting
  p.Tick();  // attempting -> success -> connected
  p.NotifyFailed();
  EXPECT_EQ(p.state(), LinkRecoveryPolicy::State::kWaiting);
  p.NotifyConnected();
  EXPECT_EQ(p.state(), LinkRecoveryPolicy::State::kConnected);
}

TEST(LinkRecoveryTest, DecideBackoffIsPureFunction) {
  LinkRecoveryConfig cfg{.initial_backoff = 100ms, .max_backoff = 1000ms,
                        .backoff_multiplier = 2.0, .jitter_ratio = 0.0};
  EXPECT_EQ(LinkRecoveryPolicy::DecideBackoff(cfg, 1).count(), 100);
  EXPECT_EQ(LinkRecoveryPolicy::DecideBackoff(cfg, 2).count(), 200);
  EXPECT_EQ(LinkRecoveryPolicy::DecideBackoff(cfg, 3).count(), 400);
  EXPECT_EQ(LinkRecoveryPolicy::DecideBackoff(cfg, 10).count(), 1000);  // capped
}

TEST(LinkRecoveryTest, DecideBackoffWithJitterRange) {
  LinkRecoveryConfig cfg{.initial_backoff = 100ms, .max_backoff = 1000ms,
                        .backoff_multiplier = 1.0, .jitter_ratio = 0.5};
  // jitter=-1 → 100 * (1 - 0.5) = 50
  EXPECT_EQ(LinkRecoveryPolicy::DecideBackoff(cfg, 1, [] { return -1.0; }).count(), 50);
  // jitter=+1 → 150
  EXPECT_EQ(LinkRecoveryPolicy::DecideBackoff(cfg, 1, [] { return 1.0; }).count(), 150);
}
