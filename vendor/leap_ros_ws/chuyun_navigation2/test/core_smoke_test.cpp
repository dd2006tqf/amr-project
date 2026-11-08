#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "chuyun_navigation2/a_star_planner.hpp"
#include "chuyun_navigation2/correlative_scan_matcher.hpp"
#include "chuyun_navigation2/grid_map.hpp"
#include "chuyun_navigation2/local_trajectory_planner.hpp"
#include "chuyun_navigation2/path_follower.hpp"

namespace chuyun_navigation2
{
namespace
{

class CoreSmokeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    directory_ = std::filesystem::temp_directory_path() / "chuyun_navigation2_core_test";
    std::filesystem::create_directories(directory_);
    const auto pgm_path = directory_ / "map.pgm";
    const auto yaml_path = directory_ / "map.yaml";

    constexpr int width = 80;
    constexpr int height = 80;
    std::vector<int> pixels(width * height, 254);
    const auto set_occupied = [&](int x, int y) {
        const int image_row = height - 1 - y;
        pixels[image_row * width + x] = 0;
      };
    for (int i = 0; i < width; ++i) {
      set_occupied(i, 0);
      set_occupied(i, height - 1);
    }
    for (int i = 0; i < height; ++i) {
      set_occupied(0, i);
      set_occupied(width - 1, i);
    }
    for (int y = 15; y <= 70; ++y) set_occupied(60, y);
    for (int x = 20; x <= 60; ++x) set_occupied(x, 60);

    std::ofstream pgm(pgm_path);
    pgm << "P2\n" << width << " " << height << "\n255\n";
    for (int pixel : pixels) pgm << pixel << '\n';
    pgm.close();

    std::ofstream yaml(yaml_path);
    yaml << "image: map.pgm\n"
         << "resolution: 0.05\n"
         << "origin: [0.0, 0.0, 0.0]\n"
         << "negate: 0\n"
         << "occupied_thresh: 0.65\n"
         << "free_thresh: 0.25\n";
    yaml.close();

    map_.load(yaml_path.string(), "map");
    map_.buildFields(0.10, 2.0);
  }

  void TearDown() override
  {
    std::filesystem::remove_all(directory_);
  }

  std::filesystem::path directory_;
  GridMap map_;
};

TEST_F(CoreSmokeTest, PlansAroundInflatedObstacles)
{
  AStarPlanner planner(map_, PlannerParameters());
  const PlanResult result = planner.plan({0.50, 0.50, 0.0}, {3.50, 3.50, 0.2});
  ASSERT_TRUE(result.success) << result.message;
  ASSERT_GT(result.path.size(), 2U);
  EXPECT_NEAR(result.path.back().yaw, 0.2, 1e-9);
}

TEST_F(CoreSmokeTest, ScanMatcherCorrectsOdometryPrediction)
{
  const Pose2D true_pose{1.0, 1.0, 0.0};
  std::vector<Point2D> points;
  for (int y = 16; y <= 68; y += 2) {
    const Point2D world = map_.cellToWorld(60, y);
    points.push_back({world.x - true_pose.x, world.y - true_pose.y});
  }
  for (int x = 22; x <= 58; x += 2) {
    const Point2D world = map_.cellToWorld(x, 60);
    points.push_back({world.x - true_pose.x, world.y - true_pose.y});
  }

  ScanMatcherParameters parameters;
  parameters.search_translation = 0.25;
  parameters.search_rotation = 0.12;
  parameters.coarse_translation_step = 0.05;
  parameters.coarse_rotation_step = 0.04;
  parameters.fine_translation_step = 0.01;
  parameters.fine_rotation_step = 0.01;
  parameters.minimum_score = 0.60;
  parameters.minimum_valid_fraction = 0.90;
  parameters.prior_weight = 0.01;
  parameters.correction_gain = 1.0;
  parameters.minimum_points = 20;
  parameters.maximum_points = 100;

  CorrelativeScanMatcher matcher(map_, parameters);
  const Pose2D predicted{1.15, 0.90, 0.08};
  const ScanMatchResult result = matcher.match(predicted, points);
  ASSERT_TRUE(result.accepted);
  EXPECT_LT(std::hypot(
      result.corrected_pose.x - true_pose.x,
      result.corrected_pose.y - true_pose.y), 0.04);
  EXPECT_LT(std::abs(normalizeAngle(result.corrected_pose.yaw - true_pose.yaw)), 0.03);
  EXPECT_GT(result.score, 0.90);
}

TEST_F(CoreSmokeTest, TemporaryObstacleChangesGlobalPath)
{
  PlannerParameters parameters;
  parameters.temporary_obstacle_radius = 0.30;
  AStarPlanner planner(map_, parameters);
  const PlanResult result = planner.plan(
    {0.50, 0.50, 0.0}, {3.50, 0.50, 0.0}, {{2.0, 0.50}});
  ASSERT_TRUE(result.success) << result.message;
  bool detoured = false;
  for (const Pose2D & pose : result.path) {
    if (std::abs(pose.y - 0.50) > 0.30) detoured = true;
  }
  EXPECT_TRUE(detoured);
}

TEST_F(CoreSmokeTest, SmoothedTemporaryDetourHasNoSharpGridCorner)
{
  PlannerParameters parameters;
  parameters.temporary_obstacle_radius = 0.30;
  parameters.path_smoothing_iterations = 2;
  AStarPlanner planner(map_, parameters);
  const PlanResult result = planner.plan(
    {0.50, 0.50, 0.0}, {2.60, 0.50, 0.0}, {{1.50, 0.50}});
  ASSERT_TRUE(result.success) << result.message;

  double maximum_heading_change = 0.0;
  size_t maximum_heading_index = 0;
  for (size_t i = 2; i < result.path.size(); ++i) {
    const double first_heading = std::atan2(
      result.path[i - 1].y - result.path[i - 2].y,
      result.path[i - 1].x - result.path[i - 2].x);
    const double second_heading = std::atan2(
      result.path[i].y - result.path[i - 1].y,
      result.path[i].x - result.path[i - 1].x);
    const double heading_change =
      std::abs(normalizeAngle(second_heading - first_heading));
    if (heading_change > maximum_heading_change) {
      maximum_heading_change = heading_change;
      maximum_heading_index = i;
    }
  }
  EXPECT_LT(maximum_heading_change, 0.80) <<
    " path_size=" << result.path.size() <<
    " corner_index=" << maximum_heading_index;
}

TEST(LocalTrajectoryPlannerTest, SelectsAnAvoidanceTrajectory)
{
  LocalPlannerParameters parameters;
  parameters.max_vx = 0.20;
  parameters.max_vy = 0.15;
  parameters.max_wz = 0.70;
  parameters.robot_radius = 0.15;
  parameters.safety_margin = 0.02;
  parameters.activation_distance = 0.80;
  parameters.preferred_clearance = 0.20;
  LocalTrajectoryPlanner planner(parameters);
  std::vector<Point2D> obstacle_points;
  for (int i = -3; i <= 3; ++i) obstacle_points.push_back({0.45, i * 0.04});

  const LocalTrajectoryResult result = planner.plan(
    {0.20, 0.0, 0.0}, {1.0, 0.0}, obstacle_points, {});
  EXPECT_TRUE(result.valid);
  EXPECT_TRUE(result.avoiding);
  EXPECT_FALSE(result.blocked);
  EXPECT_TRUE(
    std::abs(result.velocity.y) > 0.01 ||
    std::abs(result.velocity.yaw) > 0.01 ||
    result.velocity.x < 0.10);
}

TEST(LocalTrajectoryPlannerTest, RecoversFromInsideSafetyClearance)
{
  LocalPlannerParameters parameters;
  parameters.max_vx = 0.20;
  parameters.max_vy = 0.15;
  parameters.robot_radius = 0.15;
  parameters.safety_margin = 0.03;
  parameters.recovery_speed = 0.08;
  parameters.recovery_distance = 0.30;
  LocalTrajectoryPlanner planner(parameters);
  std::vector<Point2D> obstacle_points;
  for (int i = -2; i <= 2; ++i) obstacle_points.push_back({0.12, i * 0.025});

  const LocalTrajectoryResult result = planner.plan(
    {0.20, 0.0, 0.0}, {1.0, 0.0}, obstacle_points, {});
  EXPECT_TRUE(result.valid);
  EXPECT_TRUE(result.recovering);
  EXPECT_FALSE(result.blocked);
  EXPECT_GT(std::abs(result.velocity.y), std::abs(result.velocity.x));
  EXPECT_DOUBLE_EQ(result.velocity.yaw, 0.0);
}

TEST(PathFollowerTest, UsesLateralMotionInsteadOfRotateInPlace)
{
  ControllerParameters parameters;
  parameters.max_vy = 0.16;
  parameters.heading_deadband = 0.70;
  parameters.max_ay = 1.0;
  parameters.max_aw = 1.0;
  PathFollower follower{parameters};
  follower.setPath({{0.0, 0.0, 0.0}, {0.0, 1.0, 0.0}}, {0.0, 1.0, 0.0});

  const ControlResult result = follower.update({0.0, 0.0, 0.0}, {}, true, 0.1);
  EXPECT_EQ(result.status, "TRACKING");
  EXPECT_GT(result.velocity.y, 0.05);
  EXPECT_LT(std::abs(result.velocity.yaw), 0.11);
}

TEST(LocalTrajectoryPlannerTest, PrefersSideStepForFrontalObstacle)
{
  LocalPlannerParameters parameters;
  parameters.max_vx = 0.20;
  parameters.max_vy = 0.16;
  parameters.max_wz = 0.70;
  parameters.robot_radius = 0.15;
  parameters.safety_margin = 0.02;
  parameters.activation_distance = 0.80;
  parameters.heading_weight = 1.2;
  parameters.recovery_lateral_weight = 1.4;
  LocalTrajectoryPlanner planner(parameters);
  std::vector<Point2D> obstacle_points;
  for (int i = -4; i <= 4; ++i) obstacle_points.push_back({0.35, i * 0.035});

  const LocalTrajectoryResult result = planner.plan(
    {0.18, 0.0, 0.0}, {1.0, 0.0}, obstacle_points, {});
  EXPECT_TRUE(result.valid);
  EXPECT_TRUE(result.avoiding);
  EXPECT_GT(std::abs(result.velocity.y), 0.02);
  EXPECT_LT(std::abs(result.velocity.yaw), 0.20);
}

TEST(PathFollowerTest, StopsWhenScanIsStale)
{
  PathFollower follower{ControllerParameters()};
  follower.setPath({{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}}, {1.0, 0.0, 0.0});
  const ControlResult result = follower.update({0.0, 0.0, 0.0}, {}, false, 0.1);
  EXPECT_EQ(result.status, "SCAN_STALE_STOP");
  EXPECT_DOUBLE_EQ(result.velocity.x, 0.0);
  EXPECT_DOUBLE_EQ(result.velocity.y, 0.0);
  EXPECT_DOUBLE_EQ(result.velocity.yaw, 0.0);
}

}  // namespace
}  // namespace chuyun_navigation2
