#include "amr_dispatcher_core/path_tracking/stanley.hpp"

#include <gtest/gtest.h>

using amr_dispatcher_core::path_tracking::Pose2D;
using amr_dispatcher_core::path_tracking::StanleyController;

TEST(StanleyTest, DrivesStraightToGoal) {
  StanleyController c{{1.0, 0.1, 0.5, 1.0, 0.05}};
  c.SetPath({{0, 0, 0}, {1, 0, 0}, {2, 0, 0}});
  const auto r = c.Compute(Pose2D{0, 0, 0});
  EXPECT_FALSE(r.goal_reached);
  EXPECT_NEAR(r.cmd.linear_x, 0.5, 1e-9);
}

TEST(StanleyTest, StopsAtGoal) {
  StanleyController c{{1.0, 0.1, 0.5, 1.0, 0.5}};
  c.SetPath({{0, 0, 0}, {1, 0, 0}});
  const auto r = c.Compute(Pose2D{0.9, 0, 0});
  EXPECT_TRUE(r.goal_reached);
}

TEST(StanleyTest, EmptyPathProducesZeroOutput) {
  StanleyController c;
  const auto r = c.Compute(Pose2D{0, 0, 0});
  EXPECT_FALSE(r.goal_reached);
  EXPECT_NEAR(r.cmd.linear_x, 0.0, 1e-9);
}
