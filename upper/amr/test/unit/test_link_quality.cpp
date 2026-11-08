#include "amr_dispatcher_core/chassis/link_quality.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <thread>

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

// A2(a)：长期健康（远超窗口长度全成功）后突发丢包，loss_rate 必须快速反映到
// degraded/critical —— 累计语义下 6/70 ≈ 0.086 测不出，会被长期健康稀释
TEST(LinkQualityTest, WindowedLossDetectsBurstAfterLongHealthy) {
  LinkQualityMonitor q({.window_size = 8, .success_threshold = 4, .healthy_loss_rate = 0.02,
                        .degraded_loss_rate = 0.15});
  for (int i = 0; i < 64; ++i) {
    q.RecordSuccess(1.0);
  }
  EXPECT_TRUE(q.IsHealthy());
  for (int i = 0; i < 6; ++i) {
    q.RecordLoss();
  }
  // 窗口内 6/8 = 0.75 >= 0.15*2 -> critical
  EXPECT_TRUE(q.IsCritical());
  EXPECT_GE(q.Snapshot().loss_rate, 0.3);
}

// A2(b)：丢包消失且窗口滑过后自动恢复 kHealthy，无需 Reset（累计计数保留）
TEST(LinkQualityTest, SlidingWindowRecoversWithoutReset) {
  LinkQualityMonitor q({.window_size = 8, .success_threshold = 4, .healthy_loss_rate = 0.02,
                        .degraded_loss_rate = 0.15});
  for (int i = 0; i < 8; ++i) {
    q.RecordLoss();
  }
  EXPECT_TRUE(q.IsCritical());
  for (int i = 0; i < 8; ++i) {
    q.RecordSuccess(1.0);
  }
  EXPECT_TRUE(q.IsHealthy());
  const auto s = q.Snapshot();
  EXPECT_EQ(s.frames_lost, 8u);  // 累计计数不丢历史
  EXPECT_DOUBLE_EQ(s.loss_rate, 0.0);
}

// A2(c)：window_size=0 构造不崩溃，钳到 1 后行为等价 window=1
TEST(LinkQualityTest, ZeroWindowSizeClampsToSingleSample) {
  LinkQualityMonitor q({.window_size = 0, .success_threshold = 4, .healthy_loss_rate = 0.02,
                        .degraded_loss_rate = 0.15});
  q.RecordSuccess(1.0);  // 不崩溃
  q.RecordLoss();        // window=1：最新样本为 loss
  const auto s = q.Snapshot();
  EXPECT_EQ(s.frames_total, 2u);
  EXPECT_TRUE(s.loss_rate >= 0.99);  // 单样本窗口：loss_rate = 1.0
  EXPECT_FALSE(q.IsHealthy());
}

// A2(d)：恢复信号边沿化 —— 降级后连续成功 100 帧只触发一次
TEST(LinkQualityTest, RecoverySignalFiresOnceOverHundredSuccesses) {
  LinkQualityMonitor q({.window_size = 8, .success_threshold = 5, .healthy_loss_rate = 0.02,
                        .degraded_loss_rate = 0.15});
  for (int i = 0; i < 10; ++i) {
    q.RecordLoss();
  }
  EXPECT_FALSE(q.ConsumeRecoverySignal());
  for (int i = 0; i < 100; ++i) {
    q.RecordSuccess(1.0);
  }
  EXPECT_TRUE(q.ConsumeRecoverySignal());   // 首次达阈值触发
  EXPECT_FALSE(q.ConsumeRecoverySignal());  // 只触发一次
  EXPECT_FALSE(q.ConsumeRecoverySignal());
}

// A2(d)：降级（新丢包）清除未消费的恢复信号；重新达标后可再次触发
TEST(LinkQualityTest, NewLossClearsPendingRecoverySignal) {
  LinkQualityMonitor q({.window_size = 8, .success_threshold = 5, .healthy_loss_rate = 0.02,
                        .degraded_loss_rate = 0.15});
  for (int i = 0; i < 10; ++i) {
    q.RecordLoss();
  }
  for (int i = 0; i < 5; ++i) {
    q.RecordSuccess(1.0);
  }
  // 恢复信号已就绪但尚未消费，此时出现新丢包（降级）应把它清除
  q.RecordLoss();
  EXPECT_FALSE(q.ConsumeRecoverySignal());  // 旧信号已清除，而非补发
  // 连续成功被新丢包打断后重新累计，达到阈值可再次触发（新一轮恢复）
  for (int i = 0; i < 5; ++i) {
    q.RecordSuccess(1.0);
  }
  EXPECT_TRUE(q.ConsumeRecoverySignal());
  EXPECT_FALSE(q.ConsumeRecoverySignal());
}

// M2：CRC 错帧同样是"链路有活动"的证据 —— RecordCrcError 必须刷新 last_seen
TEST(LinkQualityTest, RecordCrcErrorRefreshesLastSeen) {
  LinkQualityMonitor q({.window_size = 8, .success_threshold = 4});
  q.RecordSuccess(1.0);
  const auto first = q.Snapshot().last_seen;
  std::this_thread::sleep_for(std::chrono::milliseconds(15));
  q.RecordCrcError();
  EXPECT_GT(q.Snapshot().last_seen, first);
}
