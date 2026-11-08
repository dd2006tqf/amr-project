#include "amr_dispatcher_core/chassis/command_delivery_tracker.hpp"

#include <gtest/gtest.h>

using amr_dispatcher_core::chassis::CommandDeliveryTracker;

TEST(CommandDeliveryTrackerTest, NoEchoMeansZeroDelivery) {
  CommandDeliveryTracker t(8);
  for (int i = 0; i < 8; ++i) {
    t.NextSequence();
  }
  EXPECT_FALSE(t.HasEcho());
  EXPECT_DOUBLE_EQ(t.DeliveryRate(), 0.0);
  EXPECT_EQ(t.DeliveredCount(), 0u);
  EXPECT_EQ(t.SentCount(), 8u);
}

TEST(CommandDeliveryTrackerTest, PerfectDeliveryIsOne) {
  CommandDeliveryTracker t(8);
  for (int i = 0; i < 8; ++i) {
    const uint8_t seq = t.NextSequence();
    t.OnEcho(seq);
  }
  EXPECT_DOUBLE_EQ(t.DeliveryRate(), 1.0);
  EXPECT_EQ(t.DeliveredCount(), 8u);
}

// MCU 回显滞后几帧是常态（20ms 上报周期 vs 50Hz 指令），不能判成丢包。
TEST(CommandDeliveryTrackerTest, LaggingEchoStillCountsAsDelivered) {
  CommandDeliveryTracker t(16);
  uint8_t last = 0;
  for (int i = 0; i < 10; ++i) {
    last = t.NextSequence();
    if (i >= 3) {
      // 回显滞后 3 帧
      t.OnEcho(static_cast<uint8_t>(last - 3));
    }
  }
  EXPECT_TRUE(t.HasEcho());
  // 窗口内 10 帧，已确认到第 6 帧（下标），第 4..9 帧都算送达
  EXPECT_GT(t.DeliveryRate(), 0.5);
  EXPECT_LT(t.DeliveryRate(), 1.0);
}

TEST(CommandDeliveryTrackerTest, MissingSequencesLowerTheRate) {
  CommandDeliveryTracker t(8);
  // 发 8 帧，但 MCU 只收到 0,1,2,3,4,5（后两帧丢了）
  for (int i = 0; i < 8; ++i) {
    t.NextSequence();
  }
  t.OnEcho(0);
  t.OnEcho(5);
  // 已确认 5；0..5 共 6 帧送达，8 帧窗口
  EXPECT_NEAR(t.DeliveryRate(), 6.0 / 8.0, 1e-9);
  EXPECT_EQ(t.DeliveredCount(), 6u);
}

// seq 是 uint8：255 -> 0 回绕时不能把整窗判成全丢。
TEST(CommandDeliveryTrackerTest, WrapsAroundAtByteBoundary) {
  CommandDeliveryTracker t(8);
  // 先把 next_seq_ 推到 254 附近
  for (int i = 0; i < 254; ++i) {
    t.NextSequence();
  }
  t.OnEcho(253);
  for (int i = 0; i < 6; ++i) {
    const uint8_t seq = t.NextSequence();  // 254,255,0,1,2,3
    t.OnEcho(seq);
  }
  EXPECT_DOUBLE_EQ(t.DeliveryRate(), 1.0);
  EXPECT_EQ(t.LastConfirmedSequence(), 3u);
}

TEST(CommandDeliveryTrackerTest, IgnoresStaleAndDuplicateEchoes) {
  CommandDeliveryTracker t(8);
  for (int i = 0; i < 8; ++i) {
    t.NextSequence();
  }
  t.OnEcho(5);
  const double before = t.DeliveryRate();
  t.OnEcho(5);  // 重复
  t.OnEcho(3);  // 落后超过半圈以外的乱序旧帧（这里落后 2，仍会被忽略吗？）
  // 落后 2 帧属于"delta > 128"分支 -> 忽略；送达率不变
  EXPECT_DOUBLE_EQ(t.DeliveryRate(), before);
  EXPECT_EQ(t.LastConfirmedSequence(), 5u);
}

TEST(CommandDeliveryTrackerTest, WindowSlidesSoOldLossesAgeOut) {
  CommandDeliveryTracker t(4);
  const uint8_t a = t.NextSequence();
  t.OnEcho(a);
  // 后 4 帧全丢：窗口只保留最近 4 帧，最早那一帧被挤出
  for (int i = 0; i < 4; ++i) {
    t.NextSequence();
  }
  EXPECT_EQ(t.SentCount(), 4u);
  // 已确认序号是 a，窗口内 4 帧都在 a 之后 -> 全部未送达
  EXPECT_DOUBLE_EQ(t.DeliveryRate(), 0.0);
}

TEST(CommandDeliveryTrackerTest, ResetClearsEverything) {
  CommandDeliveryTracker t(4);
  for (int i = 0; i < 4; ++i) {
    const uint8_t seq = t.NextSequence();
    t.OnEcho(seq);
  }
  t.Reset();
  EXPECT_FALSE(t.HasEcho());
  EXPECT_EQ(t.SentCount(), 0u);
  EXPECT_DOUBLE_EQ(t.DeliveryRate(), 0.0);
}
