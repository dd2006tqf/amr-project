#include <gtest/gtest.h>
#include "amr_dispatcher_core/planning/mock_linear_planner.hpp"

using namespace amr_dispatcher_core;

TEST(MockLinearPlannerTest, SameStartAndGoalReturnsSinglePoint) {
  MockLinearPlanner planner(0.1);
  Pose2D start{1.0, 2.0, 0.5};
  Pose2D goal{1.0, 2.0, 0.5};

  auto path = planner.Plan(start, goal);
  ASSERT_EQ(path.size(), 1u);
  EXPECT_DOUBLE_EQ(path.front().x, 1.0);
  EXPECT_DOUBLE_EQ(path.front().y, 2.0);
}

TEST(MockLinearPlannerTest, LinearInterpolationCheck) {
  MockLinearPlanner planner(0.2); // 0.2m step
  Pose2D start{0.0, 0.0, 0.0};
  Pose2D goal{1.0, 0.0, 1.57};

  auto path = planner.Plan(start, goal);
  // distance is 1.0, step is 0.2 -> 5 segments -> 6 points
  ASSERT_EQ(path.size(), 6u);
  EXPECT_NEAR(path.front().x, 0.0, 1e-5);
  EXPECT_NEAR(path.back().x, 1.0, 1e-5);
  EXPECT_NEAR(path.back().theta, 1.57, 1e-5);

  // intermediate point check
  EXPECT_NEAR(path[2].x, 0.4, 1e-5);
  EXPECT_NEAR(path[2].y, 0.0, 1e-5);
}

TEST(MockLinearPlannerTest, DiagonalInterpolationAndHeading) {
  MockLinearPlanner planner(0.5);
  Pose2D start{0.0, 0.0, 0.0};
  Pose2D goal{3.0, 4.0, 0.0}; // distance = 5.0m -> 10 steps -> 11 points

  auto path = planner.Plan(start, goal);
  ASSERT_EQ(path.size(), 11u);
  EXPECT_NEAR(path.front().x, 0.0, 1e-5);
  EXPECT_NEAR(path.front().y, 0.0, 1e-5);
  EXPECT_NEAR(path.back().x, 3.0, 1e-5);
  EXPECT_NEAR(path.back().y, 4.0, 1e-5);

  // Check intermediate heading is atan2(4, 3) ~ 0.927 rad
  double expected_heading = std::atan2(4.0, 3.0);
  EXPECT_NEAR(path[1].theta, expected_heading, 1e-4);
}
