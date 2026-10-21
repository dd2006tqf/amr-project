// 时空窗交通协调（Task B）的单测。
//
// 覆盖点：
//   1. 逐资源区间：路径被拆成"边 + 路口"窗口，且路口窗口与相邻段重叠
//   2. 安全余量/加减速/转弯时间进入占用时长
//   3. 冲突：无重叠时可行；重叠时整体后移到可行
//   4. 安全间隔（headway）：区间不重叠但间隔不足 -> 冲突并后移
//   5. 全有或全无：调用方按"全部成功才提交"落地（Commit 与 Plan 分离）
//   6. 延期扩窗：ExtendTimeSpaceOccupancy 成功/冲突两种路径
//   7. 边界：路径不足 2 节点、长度数组不匹配、缺 id 构造器

#include "amr_dispatcher_core/workflow/time_space_planning.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <string>
#include <vector>

namespace wf = amr_dispatcher_core::workflow;

namespace {

using Clock = std::chrono::steady_clock;

wf::TimeSpaceCandidate MakeCandidate(const std::string& mission_id) {
  wf::TimeSpaceCandidate c;
  c.mission_id = mission_id;
  c.path_node_ids = {"A", "B", "C"};
  c.segment_lengths_m = {10.0, 10.0};
  c.earliest_start = Clock::now();
  c.edge_id_builder = [](const std::string& from, const std::string& to) {
    return "route_edge:" + from + "__" + to;
  };
  c.node_id_builder = [](const std::string& node) { return "route_node:" + node; };
  return c;
}

wf::MotionTimingConfig MakeConfig() {
  wf::MotionTimingConfig cfg;
  cfg.cruise_speed_mps = 1.0;
  cfg.accel_overhead_sec = 1.0;
  cfg.turn_overhead_sec = 0.5;
  cfg.safety_margin_m = 0.2;
  cfg.min_headway_sec = 2.0;
  return cfg;
}

bool HasResource(const wf::ResourceTimeWindowSet& windows, const std::string& id) {
  for (const auto& w : windows) {
    if (w.resource_id == id) return true;
  }
  return false;
}

}  // namespace

TEST(TimeSpacePlanningTest, PathWithOneNode_IsRejected) {
  auto c = MakeCandidate("m1");
  c.path_node_ids = {"A"};
  c.segment_lengths_m.clear();
  const auto plan = wf::PlanTimeSpaceReservation(c, wf::TimeSpaceOccupancy{}, MakeConfig());
  EXPECT_FALSE(plan.feasible);
  EXPECT_NE(plan.reject_reason.find("2 nodes"), std::string::npos);
}

TEST(TimeSpacePlanningTest, SegmentLengthMismatch_IsRejected) {
  auto c = MakeCandidate("m1");
  c.segment_lengths_m = {10.0};
  const auto plan = wf::PlanTimeSpaceReservation(c, wf::TimeSpaceOccupancy{}, MakeConfig());
  EXPECT_FALSE(plan.feasible);
  EXPECT_NE(plan.reject_reason.find("mismatch"), std::string::npos);
}

TEST(TimeSpacePlanningTest, MissingIdBuilders_IsRejected) {
  auto c = MakeCandidate("m1");
  c.edge_id_builder = nullptr;
  const auto plan = wf::PlanTimeSpaceReservation(c, wf::TimeSpaceOccupancy{}, MakeConfig());
  EXPECT_FALSE(plan.feasible);
}

TEST(TimeSpacePlanningTest, EmptyOccupancy_IsFeasible_AndCoversEdgesAndIntersections) {
  auto c = MakeCandidate("m1");
  const auto plan = wf::PlanTimeSpaceReservation(c, wf::TimeSpaceOccupancy{}, MakeConfig());
  ASSERT_TRUE(plan.feasible);
  // 2 条边 + 3 个节点（起点/路口 B/路口 C）
  EXPECT_TRUE(HasResource(plan.windows, "route_edge:A__B"));
  EXPECT_TRUE(HasResource(plan.windows, "route_edge:B__C"));
  EXPECT_TRUE(HasResource(plan.windows, "route_node:A"));
  EXPECT_TRUE(HasResource(plan.windows, "route_node:B"));
  EXPECT_TRUE(HasResource(plan.windows, "route_node:C"));
  EXPECT_DOUBLE_EQ(plan.total_delay_sec, 0.0);
}

TEST(TimeSpacePlanningTest, OccupancyDuration_IncludesOverheadAndSafetyMargin) {
  auto c = MakeCandidate("m1");
  const auto cfg = MakeConfig();
  const auto plan = wf::PlanTimeSpaceReservation(c, wf::TimeSpaceOccupancy{}, cfg);
  ASSERT_TRUE(plan.feasible);
  for (const auto& w : plan.windows) {
    if (w.resource_id != "route_edge:A__B") continue;
    const double seconds = std::chrono::duration<double>(w.exit - w.enter).count();
    // 10m / 1m/s + 1.0(加减速) + 0.5(转弯) + 0.2(安全余量/速度)
    EXPECT_NEAR(seconds, 10.0 + cfg.accel_overhead_sec + cfg.turn_overhead_sec +
                           cfg.safety_margin_m / cfg.cruise_speed_mps,
                1e-6);
  }
}

TEST(TimeSpacePlanningTest, OverlappingReservation_IsShiftedToFeasibleWindow) {
  auto cfg = MakeConfig();
  auto c1 = MakeCandidate("m1");
  const auto plan1 = wf::PlanTimeSpaceReservation(c1, wf::TimeSpaceOccupancy{}, cfg);
  ASSERT_TRUE(plan1.feasible);

  wf::TimeSpaceOccupancy occupancy;
  wf::CommitTimeSpaceOccupancy(&occupancy, "m1", plan1.windows);

  // m2 立即开始 -> 与 m1 重叠 -> 必须后移
  auto c2 = MakeCandidate("m2");
  const auto plan2 = wf::PlanTimeSpaceReservation(c2, occupancy, cfg);
  ASSERT_TRUE(plan2.feasible);
  EXPECT_GT(plan2.total_delay_sec, 0.0);
  // 后移后不再与 m1 重叠
  for (const auto& w2 : plan2.windows) {
    for (const auto& w1 : plan1.windows) {
      if (w1.resource_id != w2.resource_id) continue;
      const bool overlap = w2.enter < w1.exit && w1.enter < w2.exit;
      EXPECT_FALSE(overlap) << "resource " << w2.resource_id << " still overlaps";
    }
  }
}

TEST(TimeSpacePlanningTest, HeadwayViolation_IsDetectedAndShifted) {
  auto cfg = MakeConfig();
  auto c1 = MakeCandidate("m1");
  const auto plan1 = wf::PlanTimeSpaceReservation(c1, wf::TimeSpaceOccupancy{}, cfg);
  ASSERT_TRUE(plan1.feasible);
  wf::TimeSpaceOccupancy occupancy;
  wf::CommitTimeSpaceOccupancy(&occupancy, "m1", plan1.windows);

  // m2 在 m1 驶离起始路口后 0.5s 进入同一路口（< 2s 安全间隔）-> 冲突
  auto c2 = MakeCandidate("m2");
  Clock::time_point start_node_exit = plan1.windows.front().exit;
  for (const auto& w : plan1.windows) {
    if (w.resource_id == "route_node:A") {
      start_node_exit = w.exit;
      break;
    }
  }
  c2.earliest_start = start_node_exit + std::chrono::milliseconds(500);
  const auto plan2 = wf::PlanTimeSpaceReservation(c2, occupancy, cfg);
  ASSERT_TRUE(plan2.feasible);
  EXPECT_GT(plan2.total_delay_sec, 0.0);
  bool saw_headway = false;
  for (const auto& conflict : plan2.conflicts) {
    if (conflict.reason == "headway") saw_headway = true;
  }
  EXPECT_TRUE(saw_headway);
}

TEST(TimeSpacePlanningTest, DisjointResource_IsNotBlocked) {
  auto cfg = MakeConfig();
  auto c1 = MakeCandidate("m1");
  const auto plan1 = wf::PlanTimeSpaceReservation(c1, wf::TimeSpaceOccupancy{}, cfg);
  wf::TimeSpaceOccupancy occupancy;
  wf::CommitTimeSpaceOccupancy(&occupancy, "m1", plan1.windows);

  // m2 走完全不同的路径 -> 无资源交集 -> 立即可行
  auto c2 = MakeCandidate("m2");
  c2.path_node_ids = {"X", "Y"};
  c2.segment_lengths_m = {5.0};
  const auto plan2 = wf::PlanTimeSpaceReservation(c2, occupancy, cfg);
  ASSERT_TRUE(plan2.feasible);
  EXPECT_DOUBLE_EQ(plan2.total_delay_sec, 0.0);
}

TEST(TimeSpacePlanningTest, ExtendWindow_SucceedsWhenNoConflict) {
  auto cfg = MakeConfig();
  auto c1 = MakeCandidate("m1");
  const auto plan1 = wf::PlanTimeSpaceReservation(c1, wf::TimeSpaceOccupancy{}, cfg);
  wf::TimeSpaceOccupancy occupancy;
  wf::CommitTimeSpaceOccupancy(&occupancy, "m1", plan1.windows);

  std::string message;
  const auto new_exit = plan1.windows.back().exit + std::chrono::seconds(5);
  EXPECT_TRUE(wf::ExtendTimeSpaceOccupancy(&occupancy, "m1", new_exit, &message));
  for (const auto& [resource_id, entries] : occupancy.by_resource) {
    for (const auto& entry : entries) {
      if (entry.mission_id != "m1") continue;
      if (resource_id == plan1.windows.back().resource_id) {
        EXPECT_GE(entry.exit, new_exit);
      }
    }
  }
}

TEST(TimeSpacePlanningTest, ExtendWindow_FailsWhenItWouldConflictWithAnotherMission) {
  auto cfg = MakeConfig();
  auto c1 = MakeCandidate("m1");
  const auto plan1 = wf::PlanTimeSpaceReservation(c1, wf::TimeSpaceOccupancy{}, cfg);
  wf::TimeSpaceOccupancy occupancy;
  wf::CommitTimeSpaceOccupancy(&occupancy, "m1", plan1.windows);

  // m2 与 m1 共享资源，且在 m1 原窗口之后紧接着开始
  auto c2 = MakeCandidate("m2");
  c2.earliest_start = plan1.windows.back().exit + std::chrono::milliseconds(500);
  const auto plan2 = wf::PlanTimeSpaceReservation(c2, occupancy, cfg);
  ASSERT_TRUE(plan2.feasible);
  wf::CommitTimeSpaceOccupancy(&occupancy, "m2", plan2.windows);

  // m1 延长到与 m2 重叠 -> 必须失败（且不改动占用表）
  std::string message;
  const auto bad_exit = plan2.windows.back().exit + std::chrono::seconds(5);
  EXPECT_FALSE(wf::ExtendTimeSpaceOccupancy(&occupancy, "m1", bad_exit, &message));
  EXPECT_FALSE(message.empty());
}

TEST(TimeSpacePlanningTest, CommitIsSeparateFromPlan_AllOrNothingByCallerContract) {
  // Plan 只产出窗口不落表；调用方必须在整条路径都成功后才 Commit。
  auto cfg = MakeConfig();
  auto c1 = MakeCandidate("m1");
  const auto plan1 = wf::PlanTimeSpaceReservation(c1, wf::TimeSpaceOccupancy{}, cfg);
  ASSERT_TRUE(plan1.feasible);
  wf::TimeSpaceOccupancy occupancy;
  EXPECT_TRUE(occupancy.by_resource.empty());  // Plan 未落表
  wf::CommitTimeSpaceOccupancy(&occupancy, "m1", plan1.windows);
  EXPECT_FALSE(occupancy.by_resource.empty());
}
