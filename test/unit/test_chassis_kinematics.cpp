#include "amr_dispatcher_core/chassis/chassis_kinematics.hpp"

#include <cmath>
#include <gtest/gtest.h>

#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923
#endif

using amr_dispatcher_core::chassis::ChassisCommand;
using amr_dispatcher_core::chassis::EstimateCommandFromWheelSpeeds;
using amr_dispatcher_core::chassis::EstimateWheelSpeeds;
using amr_dispatcher_core::chassis::IntegratePlanarMotion;
using amr_dispatcher_core::chassis::NormalizeKinematicsModel;
using amr_dispatcher_core::chassis::PlanarPose;
using amr_dispatcher_core::chassis::WheelSpeeds;

TEST(ChassisKinematicsTest, NormalizeModel) {
  EXPECT_EQ(NormalizeKinematicsModel("mecanum"), "mecanum");
  EXPECT_EQ(NormalizeKinematicsModel("omni"), "mecanum");
  EXPECT_EQ(NormalizeKinematicsModel("diff_drive"), "diff_drive");
  EXPECT_EQ(NormalizeKinematicsModel("ackermann"), "diff_drive");
}

TEST(ChassisKinematicsTest, IntegratePlanarMotion) {
  PlanarPose pose{0.0, 0.0, 0.0};
  ChassisCommand cmd;
  cmd.linear_x_mps = 1.0;
  cmd.angular_z_radps = 0.0;
  const auto next = IntegratePlanarMotion(pose, cmd, 1.0);
  EXPECT_NEAR(next.x_m, 1.0, 1e-9);
  EXPECT_NEAR(next.y_m, 0.0, 1e-9);
}

TEST(ChassisKinematicsTest, IntegratePlanarMotionRotates) {
  PlanarPose pose{0.0, 0.0, M_PI_2};
  ChassisCommand cmd;
  cmd.linear_x_mps = 1.0;
  const auto next = IntegratePlanarMotion(pose, cmd, 1.0);
  EXPECT_NEAR(next.x_m, 0.0, 1e-9);
  EXPECT_NEAR(next.y_m, 1.0, 1e-9);
}

TEST(ChassisKinematicsTest, DiffDriveWheelSpeeds) {
  ChassisCommand cmd;
  cmd.linear_x_mps = 1.0;
  const WheelSpeeds speeds = EstimateWheelSpeeds(cmd, "diff_drive", 0.2, 0.4, 0.5);
  for (std::size_t i = 0; i < speeds.rpm.size(); ++i) {
    EXPECT_NEAR(speeds.rpm[i], 95.4929, 1e-2);
  }
}

TEST(ChassisKinematicsTest, InverseDiffDriveCommand) {
  WheelSpeeds speeds;
  for (auto& rpm : speeds.rpm) {
    rpm = 95.4929;
  }
  const auto cmd = EstimateCommandFromWheelSpeeds(speeds, "diff_drive", 0.2, 0.4, 0.5);
  EXPECT_NEAR(cmd.linear_x_mps, 1.0, 1e-6);
  EXPECT_NEAR(cmd.angular_z_radps, 0.0, 1e-6);
}

TEST(ChassisKinematicsTest, MecanumWheelSpeedsYawComponent) {
  ChassisCommand cmd;
  cmd.linear_x_mps = 0.0;
  cmd.angular_z_radps = 1.0;
  const WheelSpeeds speeds = EstimateWheelSpeeds(cmd, "mecanum", 0.2, 0.4, 0.5);
  EXPECT_NEAR(speeds.rpm[0], -speeds.rpm[1], 1e-6);
  EXPECT_NEAR(speeds.rpm[2], -speeds.rpm[3], 1e-6);
}
