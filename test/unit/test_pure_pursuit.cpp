#include "amr_dispatcher_core/path_tracking/pure_pursuit.hpp"

#include <cmath>
#include <gtest/gtest.h>
#include <vector>

using amr_dispatcher_core::path_tracking::Pose2D;
using amr_dispatcher_core::path_tracking::PurePursuitController;

TEST(PurePursuitTest, DrivesStraightToGoal) {
  PurePursuitController c{{0.5, 0.5, 1.0, 0.05}};
  c.SetPath({{0, 0, 0}, {1, 0, 0}, {2, 0, 0}});
  const auto r = c.Compute(Pose2D{0, 0, 0});
  EXPECT_NEAR(r.cmd.linear_x, 0.5, 1e-9);
  EXPECT_NEAR(r.cmd.angular_z, 0.0, 1e-6);
  EXPECT_FALSE(r.goal_reached);
}

TEST(PurePursuitTest, StopsAtGoal) {
  PurePursuitController c{{0.5, 0.5, 1.0, 0.5}};
  c.SetPath({{0, 0, 0}, {1, 0, 0}});
  const auto r = c.Compute(Pose2D{0.9, 0, 0});
  EXPECT_TRUE(r.goal_reached);
  EXPECT_NEAR(r.cmd.linear_x, 0.0, 1e-9);
}

TEST(PurePursuitTest, EmptyPathProducesZeroOutput) {
  PurePursuitController c;
  const auto r = c.Compute(Pose2D{0, 0, 0});
  EXPECT_FALSE(r.goal_reached);
  EXPECT_NEAR(r.cmd.linear_x, 0.0, 1e-9);
  EXPECT_NEAR(r.cmd.angular_z, 0.0, 1e-9);
}

TEST(PurePursuitTest, CurvatureClampedToMaxAngularSpeed) {
  PurePursuitController c{{0.5, 1.0, 0.2, 0.05}};
  c.SetPath({{0, 0, 0}, {0, 5, 0}});  // 90 度拐弯
  const auto r = c.Compute(Pose2D{0, 0, 0});
  EXPECT_NEAR(std::abs(r.cmd.angular_z), 0.2, 1e-6);
}
