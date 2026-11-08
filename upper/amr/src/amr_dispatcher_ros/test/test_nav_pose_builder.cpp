#include "amr_dispatcher_ros/nav_pose_builder.hpp"

#include <gtest/gtest.h>

#include <cmath>

using namespace amr_dispatcher_ros;

namespace {

geometry_msgs::msg::PoseStamped MakeWp(double x, double y, double qz, double qw) {
  geometry_msgs::msg::PoseStamped ps;
  ps.pose.position.x = x;
  ps.pose.position.y = y;
  ps.pose.orientation.z = qz;
  ps.pose.orientation.w = qw;
  return ps;
}

}  // namespace

// B3：单航点必须保留调用方给的原始四元数（不得被 atan2 覆盖为单位姿态）。
// 例如 dispatcher 在 action 后端下用 wp.theta 编码站点朝向，覆盖成 yaw=0 会让
// 车辆到站后回正——丢失终端姿态约束。
TEST(NavPoseBuilder, SingleWaypointPreservesOrientation) {
  const double target_yaw = 1.2;  // 任意非零朝向
  const double qz = std::sin(target_yaw / 2.0);
  const double qw = std::cos(target_yaw / 2.0);
  const auto out = BuildNavPoses({MakeWp(3.0, 4.0, qz, qw)});
  ASSERT_EQ(out.size(), 1u);
  EXPECT_DOUBLE_EQ(out[0].pose.position.x, 3.0);
  EXPECT_DOUBLE_EQ(out[0].pose.position.y, 4.0);
  EXPECT_DOUBLE_EQ(out[0].pose.orientation.z, qz);
  EXPECT_DOUBLE_EQ(out[0].pose.orientation.w, qw);
  EXPECT_EQ(out[0].header.frame_id, "map");
}

// B3：多航点时仍按相邻点 atan2 重写航向（与原行为一致）
TEST(NavPoseBuilder, MultiWaypointOverwritesOrientation) {
  const auto out = BuildNavPoses({MakeWp(0.0, 0.0, 0.9, 0.1),  // 起点原始姿态应被覆盖
                                  MakeWp(1.0, 0.0, 0.0, 1.0)});
  ASSERT_EQ(out.size(), 2u);
  // 航向 = atan2(0-0, 1-0) = 0 -> z=0 w=1
  EXPECT_DOUBLE_EQ(out[0].pose.orientation.z, 0.0);
  EXPECT_DOUBLE_EQ(out[0].pose.orientation.w, 1.0);
  // 末点沿用最后一段朝向 -> 也是 yaw=0
  EXPECT_DOUBLE_EQ(out[1].pose.orientation.z, 0.0);
  EXPECT_DOUBLE_EQ(out[1].pose.orientation.w, 1.0);
}

// B3：空输入返回空
TEST(NavPoseBuilder, EmptyInputReturnsEmpty) {
  EXPECT_TRUE(BuildNavPoses({}).empty());
}
