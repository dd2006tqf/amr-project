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
// 半帧边界：缓冲区里只剩一个孤立的 0xAE（帧头首字节），第二字节 0xEA 还在
// 串口线上没到。旧实现遇到"找不到 magic"就 buffer_.clear()，把这个 0xAE 一起
// 丢掉，于是 0xEA 到达后帧头再也拼不起来 —— 每次跨读边界的帧都稳定丢一帧。
// Python 侧 tools/bench_serial_probe.py 的 FrameParser 特意保留了它，
// 两边行为必须一致，否则现场抓同一段字节，C++ 与探针的诊断结论会对不上。
TEST(MickBinaryPacketStreamTest, KeepsTrailingMagicByteAcrossDrain) {
  MickBinaryPacketStream stream;
  const auto frame = EncodeMickBinaryFrame(0xa2, {0x00});  // STATE running
  ASSERT_GE(frame.size(), 7u);

  // 先喂到帧头第一字节为止（含噪声前缀，确认噪声被清掉）
  const std::string first_chunk = std::string("noise") + std::string(1, static_cast<char>(0xae));
  stream.Append(first_chunk);
  EXPECT_TRUE(stream.DrainPackets().empty());

  // 再喂其余部分：帧头第二字节 + 完整剩余帧
  stream.Append(reinterpret_cast<const char*>(frame.data()) + 1, frame.size() - 1);
  const auto packets = stream.DrainPackets();
  ASSERT_EQ(packets.size(), 1u) << "0xAE 被丢弃导致跨边界帧永久丢失";
  EXPECT_EQ(packets[0].kind, ChassisPacketKind::kState);
  EXPECT_EQ(packets[0].state.status, "running");
}

// 纯噪声（不含任何 0xAE）必须被彻底清掉，不能让它无限累积到溢出。
TEST(MickBinaryPacketStreamTest, PureNoiseIsDiscardedWithoutOverflow) {
  MickBinaryPacketStream stream(64);
  stream.Append(std::string(32, 'Z'));
  EXPECT_TRUE(stream.DrainPackets().empty());
  EXPECT_FALSE(stream.Overflowed());

  const auto frame = EncodeMickBinaryFrame(0xa2, {0x00});
  stream.Append(reinterpret_cast<const char*>(frame.data()), frame.size());
  const auto packets = stream.DrainPackets();
  ASSERT_EQ(packets.size(), 1u);
  EXPECT_EQ(packets[0].kind, ChassisPacketKind::kState);
}
