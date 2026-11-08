#include "amr_dispatcher_ros/chassis_fault.hpp"

#include <gtest/gtest.h>

#include "amr_dispatcher_core/chassis/chassis_packet.hpp"

using amr_dispatcher_core::chassis::ChassisStatePacket;
using amr_dispatcher_core::chassis::MickOdometryFlags;
using amr_dispatcher_ros::IsChassisFault;

TEST(ChassisFaultTest, CleanFlagsAreNotFault) {
  EXPECT_FALSE(IsChassisFault(static_cast<uint8_t>(0)));
  EXPECT_FALSE(IsChassisFault(static_cast<uint8_t>(MickOdometryFlags::kNoFault)));
}

// 每一个故障位单独置位都必须被识别：漏掉任何一个都会让对应硬件异常静默带病运行。
TEST(ChassisFaultTest, EachFaultBitIsRecognized) {
  EXPECT_TRUE(IsChassisFault(static_cast<uint8_t>(MickOdometryFlags::kFault)));
  EXPECT_TRUE(IsChassisFault(static_cast<uint8_t>(MickOdometryFlags::kUartTimeout)));
  EXPECT_TRUE(IsChassisFault(static_cast<uint8_t>(MickOdometryFlags::kEstopInput)));
  EXPECT_TRUE(IsChassisFault(static_cast<uint8_t>(MickOdometryFlags::kOverCurrent)));
}

TEST(ChassisFaultTest, CombinedBitsStillFault) {
  const auto combined = static_cast<uint8_t>(MickOdometryFlags::kFault |
                                             MickOdometryFlags::kOverCurrent);
  EXPECT_TRUE(IsChassisFault(combined));
}

TEST(ChassisFaultTest, UnusedBitsDoNotFalselyTrip) {
  // bit4..7 目前未定义。将来固件若用它们传非故障信息（如充电中标志），
  // 不能让旧主机误判成故障停车。
  EXPECT_FALSE(IsChassisFault(static_cast<uint8_t>(1u << 4)));
  EXPECT_FALSE(IsChassisFault(static_cast<uint8_t>(1u << 7)));
}

TEST(ChassisFaultTest, StatePacketFaultFlagMapsToFault) {
  ChassisStatePacket ok;
  ok.status = "running";
  ok.fault = false;
  EXPECT_FALSE(IsChassisFault(ok));

  ChassisStatePacket bad;
  bad.status = "fault";
  bad.fault = true;
  EXPECT_TRUE(IsChassisFault(bad));
}
