#include "amr_dispatcher_core/chassis/chassis_packet.hpp"

#include <gtest/gtest.h>

#include <string>
#include <vector>

namespace {

using amr_dispatcher_core::chassis::ChassisCommand;
using amr_dispatcher_core::chassis::ChassisOdometryPacket;
using amr_dispatcher_core::chassis::ChassisPacketKind;
using amr_dispatcher_core::chassis::ChassisStatePacket;
using amr_dispatcher_core::chassis::DecodeMickBinaryPacket;
using amr_dispatcher_core::chassis::EncodeMickBinaryFrame;
using amr_dispatcher_core::chassis::EncodeMickSpeedCommand;
using amr_dispatcher_core::chassis::MickBinaryFrame;
using amr_dispatcher_core::chassis::ParseCommandLine;
using amr_dispatcher_core::chassis::ParsePacketLine;

TEST(ChassisPacketTest, EncodeCommandFixedPrecision) {
  ChassisCommand cmd;
  cmd.linear_x_mps = 0.5;
  cmd.linear_y_mps = 0.0;
  cmd.angular_z_radps = -0.25;
  EXPECT_EQ(amr_dispatcher_core::chassis::EncodeCommand(cmd), "CMD 0.5000 0.0000 -0.2500\n");
}

TEST(ChassisPacketTest, ParseCommandLineThreeArgs) {
  const auto cmd = ParseCommandLine("CMD 0.5 0.1 -0.25");
  ASSERT_TRUE(cmd.has_value());
  EXPECT_DOUBLE_EQ(cmd->linear_x_mps, 0.5);
  EXPECT_DOUBLE_EQ(cmd->linear_y_mps, 0.1);
  EXPECT_DOUBLE_EQ(cmd->angular_z_radps, -0.25);
}

TEST(ChassisPacketTest, ParseCommandLineTwoArgsFallsToAngular) {
  const auto cmd = ParseCommandLine("CMD 0.5 -0.25");
  ASSERT_TRUE(cmd.has_value());
  EXPECT_DOUBLE_EQ(cmd->linear_x_mps, 0.5);
  EXPECT_DOUBLE_EQ(cmd->linear_y_mps, 0.0);
  EXPECT_DOUBLE_EQ(cmd->angular_z_radps, -0.25);
}

TEST(ChassisPacketTest, ParseCommandLineRejectsTrailingGarbage) {
  EXPECT_FALSE(ParseCommandLine("CMD 0.5 0.1 -0.25 extra").has_value());
  EXPECT_FALSE(ParseCommandLine("ODOM 1 2 3").has_value());
  EXPECT_FALSE(ParseCommandLine("").has_value());
}

TEST(ChassisPacketTest, ParsePacketLineOdometryWithBattery) {
  const auto packet = ParsePacketLine("ODOM 1.0 2.0 0.5 0.3 0.1 24.5");
  ASSERT_TRUE(packet.has_value());
  EXPECT_EQ(packet->kind, ChassisPacketKind::kOdometry);
  EXPECT_DOUBLE_EQ(packet->odometry.x_m, 1.0);
  EXPECT_TRUE(packet->odometry.battery_voltage.has_value());
  EXPECT_DOUBLE_EQ(*packet->odometry.battery_voltage, 24.5);
}

TEST(ChassisPacketTest, ParsePacketLineState) {
  const auto packet = ParsePacketLine("STATE running 23.9");
  ASSERT_TRUE(packet.has_value());
  EXPECT_EQ(packet->kind, ChassisPacketKind::kState);
  EXPECT_EQ(packet->state.status, "running");
  ASSERT_TRUE(packet->state.battery_voltage.has_value());
  EXPECT_DOUBLE_EQ(*packet->state.battery_voltage, 23.9);
}

TEST(ChassisPacketTest, ParsePacketLineRejectsBad) {
  EXPECT_FALSE(ParsePacketLine("ODOM 1 2").has_value());
  EXPECT_FALSE(ParsePacketLine("GARBAGE").has_value());
}

TEST(ChassisPacketTest, MickFrameRoundTripSpeedCommand) {
  ChassisCommand cmd;
  cmd.linear_x_mps = 0.5;
  cmd.linear_y_mps = 0.0;
  cmd.angular_z_radps = 0.25;
  const auto frame_bytes = EncodeMickSpeedCommand(cmd);
  ASSERT_GE(frame_bytes.size(), 7u);
  EXPECT_EQ(frame_bytes[0], 0xae);
  EXPECT_EQ(frame_bytes[1], 0xea);
  EXPECT_EQ(frame_bytes[frame_bytes.size() - 2], 0xef);
  EXPECT_EQ(frame_bytes[frame_bytes.size() - 1], 0xfe);
}

TEST(ChassisPacketTest, DecodeMickOdometryPacket) {
  MickBinaryFrame frame;
  frame.type = 0xa1;
  // 12 字节：x,y,yaw,vx,wz,battery 各 int16 大端。构造 x=1.000 → 1000。
  frame.payload = {
      0x03, (uint8_t)0xe8,  // 1000
      0x00, 0x00,           // 0
      0x01, (uint8_t)0xf4,  // 500
      0x00, (uint8_t)0x2c,  // 44
      0x00, 0x00,           // 0
      0x09, (uint8_t)0xc4,  // 2500 → 25.00V
  };
  const auto packet = DecodeMickBinaryPacket(frame);
  ASSERT_TRUE(packet.has_value());
  EXPECT_EQ(packet->kind, ChassisPacketKind::kOdometry);
  EXPECT_NEAR(packet->odometry.x_m, 1.0, 1e-9);
  ASSERT_TRUE(packet->odometry.battery_voltage.has_value());
  EXPECT_NEAR(*packet->odometry.battery_voltage, 25.0, 1e-9);
}

TEST(ChassisPacketTest, DecodeMickStatePacket) {
  MickBinaryFrame frame;
  frame.type = 0xa2;
  frame.payload = {0x00, 0x09, (uint8_t)0xc4};  // status running, battery 25.00
  const auto packet = DecodeMickBinaryPacket(frame);
  ASSERT_TRUE(packet.has_value());
  EXPECT_EQ(packet->kind, ChassisPacketKind::kState);
  EXPECT_EQ(packet->state.status, "running");
}

}  // namespace
