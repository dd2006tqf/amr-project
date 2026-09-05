#include "amr_dispatcher_core/safety/cmd_vel_gate.hpp"

#include <gtest/gtest.h>

using namespace amr_dispatcher_core::safety;
using namespace std::chrono_literals;

TEST(CmdVelGateTest, AllowsCmdUnderLimits) {
  CmdVelGate gate({});
  CmdVelCommand cmd{.linear_x_mps = 0.5, .linear_y_mps = 0.3, .angular_z_radps = 0.4};
  const auto d = gate.Evaluate(cmd);
  EXPECT_TRUE(d.allowed);
  EXPECT_NEAR(d.linear_x_mps, 0.5, 1e-9);
  EXPECT_EQ(d.severity, SafetySeverity::kInfo);
}

TEST(CmdVelGateTest, ClampsOverSpeed) {
  CmdVelGate gate({});
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
  CmdVelGate gate({});
  gate.PushEvent({"battery", SafetySeverity::kWarn, "low"});
  const auto d = gate.Evaluate({.linear_x_mps = 1.0});
  EXPECT_TRUE(d.allowed);
  EXPECT_NEAR(d.linear_x_mps, 0.3, 1e-9);
  EXPECT_EQ(d.severity, SafetySeverity::kWarn);
}

TEST(CmdVelGateTest, CriticalWithoutSourceBlockIgnored) {
  CmdVelGate gate({});  // 没有 source_blocked 回调
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
  CmdVelGate gate({});
  gate.PushEvent({"a", SafetySeverity::kWarn, "x"});
  gate.ClearEvents();
  EXPECT_TRUE(gate.recent_events().empty());
}
