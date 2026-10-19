#include "amr_dispatcher_ros/safety_publish_decision.hpp"

#include <gtest/gtest.h>

#include <chrono>

using namespace amr_dispatcher_ros;
using namespace std::chrono_literals;

namespace {

PeriodicPublishInput MakeInput() {
  PeriodicPublishInput in;
  in.now = std::chrono::steady_clock::now();
  in.last_raw_stamp = in.now;
  in.has_last_raw_stamp = true;
  in.has_decision = true;
  in.last_linear_x_mps = 0.5;
  in.last_angular_z_radps = 0.2;
  in.heartbeat_timeout = 500ms;
  return in;
}

}  // namespace

// B1：estop/auto_fault/bumper/critical 任一激活 -> 周期帧强制零速
TEST(SafetyPublishDecision, EStopForcesZero) {
  auto in = MakeInput();
  in.estop_or_fault = true;
  const auto out = ComputePeriodicPublish(in);
  EXPECT_EQ(out.kind, PeriodicPublishKind::kZero);
  EXPECT_DOUBLE_EQ(out.linear_x_mps, 0.0);
  EXPECT_DOUBLE_EQ(out.angular_z_radps, 0.0);
  EXPECT_EQ(out.reason, "estop");
}

TEST(SafetyPublishDecision, BumperForcesZero) {
  auto in = MakeInput();
  in.bumper = true;
  const auto out = ComputePeriodicPublish(in);
  EXPECT_EQ(out.kind, PeriodicPublishKind::kZero);
  EXPECT_EQ(out.reason, "bumper");
}

TEST(SafetyPublishDecision, CriticalDecisionForcesZero) {
  auto in = MakeInput();
  in.decision_critical = true;
  const auto out = ComputePeriodicPublish(in);
  EXPECT_EQ(out.kind, PeriodicPublishKind::kZero);
  EXPECT_EQ(out.reason, "critical_event");
}

// B1：/cmd_vel_raw 超过 heartbeat_timeout 未收到 -> 周期零速兜底
TEST(SafetyPublishDecision, StaleRawForcesZero) {
  auto in = MakeInput();
  in.last_raw_stamp = in.now - 600ms;
  const auto out = ComputePeriodicPublish(in);
  EXPECT_EQ(out.kind, PeriodicPublishKind::kZero);
  EXPECT_EQ(out.reason, "cmd_vel_raw_stale");
}

// 边界：恰好等于阈值不算过期（严格大于才触发）
TEST(SafetyPublishDecision, BoundaryTimeoutNotStale) {
  auto in = MakeInput();
  in.last_raw_stamp = in.now - 500ms;
  const auto out = ComputePeriodicPublish(in);
  EXPECT_EQ(out.kind, PeriodicPublishKind::kDecision);
  EXPECT_DOUBLE_EQ(out.linear_x_mps, 0.5);
  EXPECT_DOUBLE_EQ(out.angular_z_radps, 0.2);
}

// 正常路径：重放最近一次裁决结果
TEST(SafetyPublishDecision, ReplaysLastDecision) {
  auto in = MakeInput();
  in.last_raw_stamp = in.now - 100ms;  // 新鲜
  const auto out = ComputePeriodicPublish(in);
  EXPECT_EQ(out.kind, PeriodicPublishKind::kDecision);
  EXPECT_DOUBLE_EQ(out.linear_x_mps, 0.5);
  EXPECT_DOUBLE_EQ(out.linear_y_mps, 0.0);
  EXPECT_DOUBLE_EQ(out.angular_z_radps, 0.2);
}

// 从未收到过 raw：不发任何帧（系统还没开始运动，持续零速无意义）
TEST(SafetyPublishDecision, NeverReceivedSkips) {
  auto in = MakeInput();
  in.has_decision = false;
  in.has_last_raw_stamp = false;
  const auto out = ComputePeriodicPublish(in);
  EXPECT_EQ(out.kind, PeriodicPublishKind::kSkip);
}
