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

// T3：下行帧带 seq（payload 6 -> 7 字节，整帧 13 -> 14）。
// 理由：没有序号时"链路质量"只能统计读空和 CRC 错，命令真的丢了测不出来。
TEST(ChassisPacketTest, SpeedCommandCarriesSequenceByte) {
  ChassisCommand cmd;
  cmd.linear_x_mps = 0.5;
  cmd.sequence = 0x2a;
  const auto frame_bytes = EncodeMickSpeedCommand(cmd);
  ASSERT_EQ(frame_bytes.size(), 14u);      // 2 magic + len + type + 7 + sum8 + 2 tail
  EXPECT_EQ(frame_bytes[2], 9u);           // len = payload(7) + 2 (type + sum8)
  EXPECT_EQ(frame_bytes[3], 0xf3);
  EXPECT_EQ(frame_bytes[10], 0x2a);        // payload 从 [4] 起，seq 在 payload[6] -> [10]
  // sum8 必须覆盖新增字节，否则 MCU 会把它当成坏帧整帧丢弃
  uint8_t sum = 0;
  for (std::size_t i = 2; i + 3 < frame_bytes.size(); ++i) sum += frame_bytes[i];
  EXPECT_EQ(frame_bytes[frame_bytes.size() - 3], sum);
}

TEST(ChassisPacketTest, SpeedCommandSequenceWrapsAtByteBoundary) {
  ChassisCommand cmd;
  cmd.sequence = 255;
  const auto frame_bytes = EncodeMickSpeedCommand(cmd);
  EXPECT_EQ(frame_bytes[10], 255u);
}

// T2/T3：上行 flags 与 last_cmd_seq 是可选扩展字节。
// 12 字节的旧固件帧必须继续被接受（向后兼容），新增字节缺失时按"无故障"处理。
TEST(ChassisPacketTest, DecodeMickOdometryBackwardCompatibleWith12Bytes) {
  MickBinaryFrame frame;
  frame.type = 0xa1;
  frame.payload = {0x00, 0x64, 0x00, 0x00, 0x00, 0x00,
                   0x00, 0x64, 0x00, 0x00, 0x04, 0xd2};  // battery = 1234 -> 12.34V
  const auto packet = DecodeMickBinaryPacket(frame);
  ASSERT_TRUE(packet.has_value());
  EXPECT_NEAR(packet->odometry.x_m, 0.1, 1e-9);
  ASSERT_TRUE(packet->odometry.battery_voltage.has_value());
  EXPECT_NEAR(*packet->odometry.battery_voltage, 12.34, 1e-9);
  EXPECT_EQ(packet->odometry.flags, 0u);          // 缺字节 -> 无故障
  EXPECT_EQ(packet->odometry.last_cmd_seq, 0u);
}

TEST(ChassisPacketTest, DecodeMickOdometryWithFlagsAndEchoedSeq) {
  MickBinaryFrame frame;
  frame.type = 0xa1;
  frame.payload = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                   0x00, 0x00, 0x00, 0x00, 0x04, 0xd2,
                   0x05,   // flags: bit0 fault | bit2 estop
                   0x07};  // last_cmd_seq
  const auto packet = DecodeMickBinaryPacket(frame);
  ASSERT_TRUE(packet.has_value());
  EXPECT_EQ(packet->odometry.flags, 0x05u);
  EXPECT_EQ(packet->odometry.last_cmd_seq, 0x07u);
}

// T2：MCU 上报 fault 必须能被上层识别为"链路不可信"，而不是只打一行日志。
TEST(ChassisPacketTest, OdometryFlagsExposeFaultAndUartTimeout) {
  using amr_dispatcher_core::chassis::HasMickFlag;
  using amr_dispatcher_core::chassis::MickOdometryFlags;
  EXPECT_TRUE(HasMickFlag(MickOdometryFlags::kFault, MickOdometryFlags::kFault));
  EXPECT_FALSE(HasMickFlag(0, MickOdometryFlags::kFault));
  EXPECT_TRUE(HasMickFlag(MickOdometryFlags::kUartTimeout | MickOdometryFlags::kFault,
                          MickOdometryFlags::kFault));
  EXPECT_TRUE(HasMickFlag(MickOdometryFlags::kEstopInput, MickOdometryFlags::kEstopInput));
  EXPECT_FALSE(HasMickFlag(MickOdometryFlags::kOverCurrent, MickOdometryFlags::kEstopInput));
}

// T2：0xA2 的 fault 状态位必须同时携带"是否可继续运动"的语义。
// 之前 HandleState 只比对字符串 "fault"，MCU 的过流/UART 超时无法触发停车。
TEST(ChassisPacketTest, StatePacketCarriesFaultFlag) {
  MickBinaryFrame fault_frame;
  fault_frame.type = 0xa2;
  fault_frame.payload = {0x01, 0x04, 0xd2};
  const auto fault_packet = DecodeMickBinaryPacket(fault_frame);
  ASSERT_TRUE(fault_packet.has_value());
  EXPECT_EQ(fault_packet->state.status, "fault");
  EXPECT_TRUE(fault_packet->state.fault);
  EXPECT_TRUE(fault_packet->state.battery_voltage.has_value());

  MickBinaryFrame ok_frame;
  ok_frame.type = 0xa2;
  ok_frame.payload = {0x00, 0x04, 0xd2};
  const auto ok_packet = DecodeMickBinaryPacket(ok_frame);
  ASSERT_TRUE(ok_packet.has_value());
  EXPECT_EQ(ok_packet->state.status, "running");
  EXPECT_FALSE(ok_packet->state.fault);
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
