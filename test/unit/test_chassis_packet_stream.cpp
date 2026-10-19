#include "amr_dispatcher_core/chassis/chassis_packet_stream.hpp"

#include <gtest/gtest.h>

#include <string>

using amr_dispatcher_core::chassis::ChassisPacketKind;
using amr_dispatcher_core::chassis::ChassisPacketStream;
using amr_dispatcher_core::chassis::EncodeMickBinaryFrame;
using amr_dispatcher_core::chassis::MickBinaryPacketStream;

TEST(ChassisPacketStreamTest, DrainsCompleteLines) {
  ChassisPacketStream stream;
  stream.Append("ODOM 1 2 0.5 0.3 0.1\n");
  stream.Append("STATE running\n");
  const auto packets = stream.DrainPackets();
  ASSERT_EQ(packets.size(), 2u);
  EXPECT_EQ(packets[0].kind, ChassisPacketKind::kOdometry);
  EXPECT_EQ(packets[1].kind, ChassisPacketKind::kState);
}

TEST(ChassisPacketStreamTest, HandlesSplitPacketAcrossAppends) {
  ChassisPacketStream stream;
  stream.Append("ODOM 1 2 0.5 0.");
  EXPECT_TRUE(stream.DrainPackets().empty());
  stream.Append("3 0.1\n");
  const auto packets = stream.DrainPackets();
  ASSERT_EQ(packets.size(), 1u);
  EXPECT_DOUBLE_EQ(packets[0].odometry.linear_x_mps, 0.3);
}

TEST(ChassisPacketStreamTest, CountsInvalidLines) {
  ChassisPacketStream stream;
  stream.Append("GARBAGE\n");
  stream.Append("ODOM 1 2 0.5 0.3 0.1\n");
  EXPECT_EQ(stream.InvalidLineCount(), 1u);
  EXPECT_EQ(stream.DrainPackets().size(), 1u);
}

TEST(ChassisPacketStreamTest, OverflowClearsBuffer) {
  ChassisPacketStream stream(16);
  stream.Append("ODOM 1 2 0.5 0.3 0.1\n");
  // 溢出：整段缓冲（含这条已完整的行）被丢弃，事件位置位
  EXPECT_TRUE(stream.Overflowed());
  EXPECT_TRUE(stream.DrainPackets().empty());
  // A9：溢出不永久哑掉 —— 缓冲清空后自动恢复，后续正常行照常解包
  stream.Append("STATE running\n");
  EXPECT_EQ(stream.DrainPackets().size(), 1u);
}

TEST(ChassisPacketStreamTest, TrailingPartialLineIsKept) {
  ChassisPacketStream stream;
  stream.Append("STATE running\nODOM 1 2");
  const auto packets = stream.DrainPackets();
  ASSERT_EQ(packets.size(), 1u);
  EXPECT_EQ(packets[0].kind, ChassisPacketKind::kState);
  // 残留半包在下次 Append 后可继续解析
  stream.Append(" 0.5 0.3 0.1\n");
  EXPECT_EQ(stream.DrainPackets().size(), 1u);
}

// A9(a)：超长无 '\n' 行触发溢出后自动恢复 —— 后续正常行仍能解出包，
// Overflowed() 是"曾发生溢出"事件标志（解包成功后仍保持，ClearOverflow 可清）
TEST(ChassisPacketStreamTest, OverflowRecoversAndFlagStaysUntilCleared) {
  ChassisPacketStream stream(64);
  const std::string long_line(512, 'X');  // 远超 64 字节且不含 '\n'
  stream.Append(long_line);
  EXPECT_TRUE(stream.Overflowed());
  EXPECT_TRUE(stream.DrainPackets().empty());

  stream.Append("ODOM 1 2 0.5 0.3 0.1\n");
  const auto packets = stream.DrainPackets();
  ASSERT_EQ(packets.size(), 1u);
  EXPECT_EQ(packets[0].kind, ChassisPacketKind::kOdometry);

  EXPECT_TRUE(stream.Overflowed());  // 事件位不因解包成功而自动复位
  stream.ClearOverflow();
  EXPECT_FALSE(stream.Overflowed());
}

// A9(b)：二进制流溢出后语义一致 —— flag 为事件位、后续帧继续解析
TEST(MickBinaryPacketStreamTest, OverflowRecoversAndParsesNextFrame) {
  MickBinaryPacketStream stream(32);
  stream.Append(std::string(512, 'X'));  // 远超 32 → 溢出
  EXPECT_TRUE(stream.Overflowed());
  EXPECT_TRUE(stream.DrainPackets().empty());

  const auto frame = EncodeMickBinaryFrame(0xa2, {0x00});  // STATE running
  stream.Append(reinterpret_cast<const char*>(frame.data()), frame.size());
  const auto packets = stream.DrainPackets();
  ASSERT_EQ(packets.size(), 1u);
  EXPECT_EQ(packets[0].kind, ChassisPacketKind::kState);
  EXPECT_EQ(packets[0].state.status, "running");

  EXPECT_TRUE(stream.Overflowed());
  stream.ClearOverflow();
  EXPECT_FALSE(stream.Overflowed());
}