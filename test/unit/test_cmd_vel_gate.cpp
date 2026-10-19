#include "amr_dispatcher_core/safety/cmd_vel_gate.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

using namespace amr_dispatcher_core::safety;
using namespace std::chrono_literals;

TEST(CmdVelGateTest, AllowsCmdUnderLimits) {
  CmdVelGate gate{};
  CmdVelCommand cmd{.linear_x_mps = 0.5, .linear_y_mps = 0.3, .angular_z_radps = 0.4};
  const auto d = gate.Evaluate(cmd);
  EXPECT_TRUE(d.allowed);
  EXPECT_NEAR(d.linear_x_mps, 0.5, 1e-9);
  EXPECT_EQ(d.severity, SafetySeverity::kInfo);
}

TEST(CmdVelGateTest, ClampsOverSpeed) {
  CmdVelGate gate{};
  CmdVelCommand cmd{.linear_x_mps = 5.0, .linear_y_mps = 0.0, .angular_z_radps = 5.0};
  const auto d = gate.Evaluate(cmd);
  EXPECT_DOUBLE_EQ(d.linear_x_mps, 1.5);
  EXPECT_DOUBLE_EQ(d.angular_z_radps, 2.0);
}

TEST(CmdVelGateTest, CriticalEventStopsMotion) {
  CmdVelGate gate({.source_blocked = [](const std::string& src) {
    return src == "bumper";
  }});
  gate.PushEvent({"bumper", SafetySeverity::kCritical, "front bumper hit"});
  CmdVelCommand cmd{.linear_x_mps = 0.5};
  const auto d = gate.Evaluate(cmd);
  EXPECT_FALSE(d.allowed);
  EXPECT_DOUBLE_EQ(d.linear_x_mps, 0.0);
  EXPECT_EQ(d.severity, SafetySeverity::kCritical);
  EXPECT_NE(d.reason.find("bumper"), std::string::npos);
}

TEST(CmdVelGateTest, WarnEventReducesSpeed) {
  CmdVelGate gate{};
  gate.PushEvent({"battery", SafetySeverity::kWarn, "low"});
  const auto d = gate.Evaluate({.linear_x_mps = 1.0});
  EXPECT_TRUE(d.allowed);
  EXPECT_NEAR(d.linear_x_mps, 0.3, 1e-9);
  EXPECT_EQ(d.severity, SafetySeverity::kWarn);
}

TEST(CmdVelGateTest, CriticalWithoutSourceBlockIgnored) {
  CmdVelGate gate{};  // 没有 source_blocked 回调
  gate.PushEvent({"estop", SafetySeverity::kCritical, "pressed"});
  const auto d = gate.Evaluate({.linear_x_mps = 0.5});
  EXPECT_TRUE(d.allowed);
  EXPECT_NEAR(d.linear_x_mps, 0.5, 1e-9);
}

TEST(CmdVelGateTest, EventHistoryBounded) {
  CmdVelGate gate({.event_history_capacity = 3});
  for (int i = 0; i < 10; ++i) {
    gate.PushEvent({"x", SafetySeverity::kInfo, "m" + std::to_string(i)});
  }
  EXPECT_EQ(gate.recent_events().size(), 3u);
  EXPECT_EQ(gate.recent_events().back().message, "m9");
}

TEST(CmdVelGateTest, ClearEvents) {
  CmdVelGate gate{};
  gate.PushEvent({"a", SafetySeverity::kWarn, "x"});
  gate.ClearEvents();
  EXPECT_TRUE(gate.recent_events().empty());
}

// A1：critical 事件在 stamp + event_ttl 内保持阻止，超过 TTL 后自动放行
TEST(CmdVelGateTest, CriticalEventExpiresAfterTtl) {
  CmdVelGate gate({.source_blocked = [](const std::string& src) {
    return src == "bumper";
  }});
  const auto stamp = std::chrono::steady_clock::now();
  gate.PushEvent({"bumper", SafetySeverity::kCritical, "front bumper hit", stamp});
  const CmdVelCommand cmd{.linear_x_mps = 0.5};

  // TTL 内（默认 2000ms）仍然阻止
  const auto within = gate.Evaluate(cmd, stamp + 1500ms);
  EXPECT_FALSE(within.allowed);
  EXPECT_EQ(within.severity, SafetySeverity::kCritical);

  // 超过 TTL 后事件失效，正常放行
  const auto after = gate.Evaluate(cmd, stamp + 2000ms + 1ms);
  EXPECT_TRUE(after.allowed);
  EXPECT_NEAR(after.linear_x_mps, 0.5, 1e-9);
}

// A1：warn 事件同样受 TTL 约束（超时后不再降速）
TEST(CmdVelGateTest, WarnEventExpiresAfterTtl) {
  CmdVelGate gate{};
  const auto stamp = std::chrono::steady_clock::now();
  gate.PushEvent({"battery", SafetySeverity::kWarn, "low", stamp});
  const auto after = gate.Evaluate({.linear_x_mps = 1.0}, stamp + 2000ms + 1ms);
  EXPECT_TRUE(after.allowed);
  EXPECT_DOUBLE_EQ(after.linear_x_mps, 1.0);
  EXPECT_EQ(after.severity, SafetySeverity::kInfo);
}

// A1：NaN 指令归零，避免把非法值写进底盘
TEST(CmdVelGateTest, ScaleNanBecomesZero) {
  CmdVelGate gate{};
  const auto d = gate.Evaluate({.linear_x_mps = std::numeric_limits<double>::quiet_NaN(),
                                .linear_y_mps = std::numeric_limits<double>::quiet_NaN(),
                                .angular_z_radps = std::numeric_limits<double>::quiet_NaN()});
  EXPECT_TRUE(d.allowed);
  EXPECT_TRUE(std::isfinite(d.linear_x_mps));
  EXPECT_DOUBLE_EQ(d.linear_x_mps, 0.0);
  EXPECT_DOUBLE_EQ(d.linear_y_mps, 0.0);
  EXPECT_DOUBLE_EQ(d.angular_z_radps, 0.0);
}

// A1：±Inf 按超限钳到 ±max（不放行非法值，也不直接归零）
TEST(CmdVelGateTest, ScaleInfClampsToLimit) {
  CmdVelGate gate{};
  const auto d = gate.Evaluate(
      {.linear_x_mps = std::numeric_limits<double>::infinity(),
       .angular_z_radps = -std::numeric_limits<double>::infinity()});
  EXPECT_TRUE(d.allowed);
  EXPECT_DOUBLE_EQ(d.linear_x_mps, 1.5);
  EXPECT_DOUBLE_EQ(d.angular_z_radps, -2.0);
}
