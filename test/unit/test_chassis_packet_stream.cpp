#include "amr_dispatcher_core/chassis/chassis_packet_stream.hpp"

#include <gtest/gtest.h>

#include <string>

using amr_dispatcher_core::chassis::ChassisPacketKind;
using amr_dispatcher_core::chassis::ChassisPacketStream;

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
  stream.Append("STATE running\n");
  EXPECT_TRUE(stream.Overflowed());
  EXPECT_TRUE(stream.DrainPackets().empty());
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