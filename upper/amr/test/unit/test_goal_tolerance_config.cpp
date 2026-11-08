// T4：终点判定容差必须是**可配置**的，而不是控制器里的一个常量。
//
// 为什么值得单独测试：台架上 odom_origin_* 标定偏几十厘米是常态，
// 10cm 的到达窗口会让车"到了但不 arrived"，任务挂到 10 秒后被死锁检测
// 判成 kIdleActive——现象看起来像调度 bug，根因却在控制器的默认值上。
// 这里锁定的是"容差真的改变了到达判定边界"这件事本身。
#include "amr_dispatcher_core/path_tracking/pure_pursuit.hpp"
#include "amr_dispatcher_core/path_tracking/stanley.hpp"

#include <gtest/gtest.h>
#include <vector>

using amr_dispatcher_core::path_tracking::Pose2D;
using amr_dispatcher_core::path_tracking::PurePursuitController;
using amr_dispatcher_core::path_tracking::StanleyController;

namespace {

// 一条 2 米直线，终点在 (2, 0)。车停在 0.25 米外偏终点处。
constexpr double kGoalX = 2.0;
constexpr double kRobotX = 1.75;

std::vector<Pose2D> StraightPath() {
  return {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {kGoalX, 0.0, 0.0}};
}

}  // namespace

// 默认容差（0.10）下，距离 0.25m 不该判到达——这正是台架卡住的现场。
TEST(GoalToleranceTest, DefaultToleranceDoesNotReachAt25cm) {
  PurePursuitController pp{PurePursuitController::Config{}};
  EXPECT_DOUBLE_EQ(pp.config().goal_tolerance, 0.10);
  pp.SetPath(StraightPath());
  const auto r = pp.Compute(Pose2D{kRobotX, 0.0, 0.0});
  EXPECT_FALSE(r.goal_reached);
}

// 把容差配成 0.3 后，同一位置应判到达。
TEST(GoalToleranceTest, RaisedToleranceReachesAt25cm) {
  PurePursuitController::Config cfg;
  cfg.goal_tolerance = 0.30;
  PurePursuitController pp{cfg};
  pp.SetPath(StraightPath());
  const auto r = pp.Compute(Pose2D{kRobotX, 0.0, 0.0});
  EXPECT_TRUE(r.goal_reached);
  // 到达时必须是零速，否则"到达"瞬间还会继续往前拱。
  EXPECT_DOUBLE_EQ(r.cmd.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(r.cmd.angular_z, 0.0);
}

// Stanley 走同一条 IsFinalWaypointReached，容差必须同样生效——
// 两条控制律只能有一套到达语义，否则换 controller_type 就换了行为。
TEST(GoalToleranceTest, StanleyHonorsSameTolerance) {
  StanleyController::Config strict;
  strict.goal_tolerance = 0.10;
  StanleyController stanley_strict{strict};
  stanley_strict.SetPath(StraightPath());
  EXPECT_FALSE(stanley_strict.Compute(Pose2D{kRobotX, 0.0, 0.0}).goal_reached);

  StanleyController::Config loose;
  loose.goal_tolerance = 0.30;
  StanleyController stanley_loose{loose};
  stanley_loose.SetPath(StraightPath());
  EXPECT_TRUE(stanley_loose.Compute(Pose2D{kRobotX, 0.0, 0.0}).goal_reached);
}

// 负容差不能变成"永远到达"或"永远不到达"这种未定义行为：
// 两个控制器都在构造时把负值钳成 0，语义等价于"必须精确压线"。
TEST(GoalToleranceTest, NegativeToleranceClampsToZero) {
  PurePursuitController::Config pp_cfg;
  pp_cfg.goal_tolerance = -1.0;
  PurePursuitController pp{pp_cfg};
  EXPECT_DOUBLE_EQ(pp.config().goal_tolerance, 0.0);

  StanleyController::Config st_cfg;
  st_cfg.goal_tolerance = -1.0;
  StanleyController stanley{st_cfg};
  EXPECT_DOUBLE_EQ(stanley.config().goal_tolerance, 0.0);

  // 钳到 0 之后，距终点 0.25m 依然不得到达（不是"负容差放行一切"）。
  pp.SetPath(StraightPath());
  EXPECT_FALSE(pp.Compute(Pose2D{kRobotX, 0.0, 0.0}).goal_reached);
}

// 容差只放宽"多远算到"，不放宽"必须走到路径末端"这个前提——
// 车停在路径中段即使容差很大也不该判到达，否则任务会在半路被误结束。
TEST(GoalToleranceTest, LargeToleranceStillRequiresPathEnd) {
  PurePursuitController::Config cfg;
  cfg.goal_tolerance = 5.0;
  PurePursuitController pp{cfg};
  pp.SetPath(StraightPath());
  // 车在起点附近，最近点是索引 0（不是末点），距终点 2m < 5m 容差。
  const auto r = pp.Compute(Pose2D{0.0, 0.0, 0.0});
  EXPECT_FALSE(r.goal_reached);
}
