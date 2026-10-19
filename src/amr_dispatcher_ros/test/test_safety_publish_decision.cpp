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

// 锁存型安全源覆写（estop/bumper/自动故障）：事件 TTL 过期不等于故障解除，
// 只要锁存状态仍激活，raw 裁决路径必须被强制零速 critical。
namespace {
amr_dispatcher_core::safety::CmdVelGateDecision MakeAllowedDecision() {
  amr_dispatcher_core::safety::CmdVelGateDecision d;
  d.allowed = true;
  d.linear_x_mps = 0.8;
  d.linear_y_mps = 0.1;
  d.angular_z_radps = 0.3;
  d.severity = amr_dispatcher_core::safety::SafetySeverity::kInfo;
  d.reason = "pass";
  return d;
}
}  // namespace

TEST(LatchedBlocks, EstopLatchedOverridesToZero) {
  auto d = MakeAllowedDecision();
  ApplyLatchedBlocks(/*estop_or_fault=*/true, /*bumper=*/false, d);
  EXPECT_FALSE(d.allowed);
  EXPECT_DOUBLE_EQ(d.linear_x_mps, 0.0);
  EXPECT_DOUBLE_EQ(d.linear_y_mps, 0.0);
  EXPECT_DOUBLE_EQ(d.angular_z_radps, 0.0);
  EXPECT_EQ(d.severity, amr_dispatcher_core::safety::SafetySeverity::kCritical);
  EXPECT_EQ(d.reason, "estop");
}

TEST(LatchedBlocks, BumperLatchedOverridesToZero) {
  auto d = MakeAllowedDecision();
  ApplyLatchedBlocks(false, true, d);
  EXPECT_FALSE(d.allowed);
  EXPECT_DOUBLE_EQ(d.linear_x_mps, 0.0);
  EXPECT_EQ(d.severity, amr_dispatcher_core::safety::SafetySeverity::kCritical);
  EXPECT_EQ(d.reason, "bumper");
}

TEST(LatchedBlocks, NoLatchKeepsDecisionUnchanged) {
  auto d = MakeAllowedDecision();
  ApplyLatchedBlocks(false, false, d);
  EXPECT_TRUE(d.allowed);
  EXPECT_DOUBLE_EQ(d.linear_x_mps, 0.8);
  EXPECT_DOUBLE_EQ(d.angular_z_radps, 0.3);
  EXPECT_EQ(d.reason, "pass");
}
