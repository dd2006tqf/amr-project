#include "amr_dispatcher_core/chassis/link_quality.hpp"

#include <gtest/gtest.h>

using namespace amr_dispatcher_core::chassis;

TEST(LinkQualityTest, EmptyIsUnknown) {
  LinkQualityMonitor q;
  EXPECT_EQ(q.CurrentTier(), LinkHealthTier::kUnknown);
  EXPECT_FALSE(q.IsHealthy());
}

TEST(LinkQualityTest, AllSuccessBecomesHealthy) {
  LinkQualityMonitor q({.window_size = 8, .success_threshold = 4, .healthy_loss_rate = 0.02,
                        .degraded_loss_rate = 0.15});
  for (int i = 0; i < 12; ++i) {
    q.RecordSuccess(5.0);
  }
  EXPECT_TRUE(q.IsHealthy());
  const auto s = q.Snapshot();
  EXPECT_EQ(s.frames_success, 12u);
  EXPECT_EQ(s.frames_lost, 0u);
  EXPECT_NEAR(s.loss_rate, 0.0, 1e-9);
}

TEST(LinkQualityTest, CrcErrorCountsAsLost) {
  LinkQualityMonitor q({.window_size = 8, .success_threshold = 4, .healthy_loss_rate = 0.02,
                        .degraded_loss_rate = 0.15});
  for (int i = 0; i < 10; ++i) {
    q.RecordSuccess(1.0);
  }
  q.RecordCrcError();
  q.RecordCrcError();
  q.RecordCrcError();
  q.RecordLoss();
  const auto s = q.Snapshot();
  EXPECT_EQ(s.frames_crc_error, 3u);
  EXPECT_EQ(s.frames_lost, 4u);  // 3 crc + 1 explicit loss
  EXPECT_GT(s.loss_rate, 0.0);
}

TEST(LinkQualityTest, HighLossTriggersCritical) {
  LinkQualityMonitor q({.window_size = 8, .success_threshold = 4, .healthy_loss_rate = 0.02,
                        .degraded_loss_rate = 0.15});
  for (int i = 0; i < 5; ++i) {
    q.RecordSuccess(1.0);
  }
  for (int i = 0; i < 20; ++i) {
    q.RecordLoss();
  }
  EXPECT_TRUE(q.IsCritical());
}

TEST(LinkQualityTest, RecoverySignalFiresAfterThreshold) {
  LinkQualityMonitor q({.window_size = 8, .success_threshold = 5, .healthy_loss_rate = 0.02,
                        .degraded_loss_rate = 0.15});
  for (int i = 0; i < 10; ++i) {
    q.RecordLoss();
  }
  EXPECT_FALSE(q.ConsumeRecoverySignal());
  for (int i = 0; i < 5; ++i) {
    q.RecordSuccess(1.0);
  }
  EXPECT_TRUE(q.ConsumeRecoverySignal());
  EXPECT_FALSE(q.ConsumeRecoverySignal());  // consume once
}

TEST(LinkQualityTest, ResetClearsState) {
  LinkQualityMonitor q;
  q.RecordSuccess(1.0);
  q.RecordLoss();
  q.Reset();
  EXPECT_EQ(q.CurrentTier(), LinkHealthTier::kUnknown);
  EXPECT_EQ(q.Snapshot().frames_total, 0u);
}

TEST(LinkQualityTest, LatencyStatsTrackMax) {
  LinkQualityMonitor q;
  q.RecordSuccess(10.0);
  q.RecordSuccess(40.0);
  q.RecordSuccess(20.0);
  const auto s = q.Snapshot();
  EXPECT_DOUBLE_EQ(s.max_latency_ms, 40.0);
  EXPECT_DOUBLE_EQ(s.average_latency_ms, (10.0 + 40.0 + 20.0) / 3.0);
}
