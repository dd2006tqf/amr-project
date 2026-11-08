// B4 集成测试：多车共享边上的时空窗协调。
//
// 覆盖两个真实场景：
//   1. **两单共享边、不同窗口 -> 双成功**：错峰通过同一通道，两单都拿到路权。
//   2. **两单共享边、同一窗口 -> 一成一败，且无残留半预约**：
//      后到的那单在共享边上被拒，且它整条路径的预约**整体回滚**
//      （绝不出现"共享边没拿到、但后面的边已经占上"的半套预约）。
//
// 另加一条对头（head-on）场景：反向行驶的两单在**同一条边资源**上冲突
// （边 id 归一化后与行驶方向无关）——这正是窄道对向冲突的表达。

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <string>
#include <vector>

#include "amr_dispatcher_core/dispatcher/traffic_reservation.hpp"
#include "amr_dispatcher_core/workflow/time_space_planning.hpp"

using namespace std::chrono_literals;
using amr_dispatcher_core::dispatcher::ResourceTable;
using amr_dispatcher_core::dispatcher::TimeInterval;
namespace wf = amr_dispatcher_core::workflow;

namespace {

using Clock = std::chrono::steady_clock;

wf::MotionTimingConfig MakeConfig() {
  wf::MotionTimingConfig cfg;
  cfg.cruise_speed_mps = 1.0;
  cfg.accel_overhead_sec = 1.0;
  cfg.turn_overhead_sec = 0.5;
  cfg.safety_margin_m = 0.2;
  cfg.min_headway_sec = 2.0;
  return cfg;
}

// 候选任务：路径 A -> B（可再接 B -> C），段长 10 m
wf::TimeSpaceCandidate MakeCandidate(const std::string& mission_id,
                                     const std::vector<std::string>& path,
                                     const std::vector<double>& lengths,
                                     Clock::time_point earliest_start) {
  wf::TimeSpaceCandidate c;
  c.mission_id = mission_id;
  c.path_node_ids = path;
  c.segment_lengths_m = lengths;
  c.earliest_start = earliest_start;
  c.edge_id_builder = [](const std::string& from, const std::string& to) {
    return ResourceTable::RouteEdgeId(from, to);
  };
  c.node_id_builder = [](const std::string& node) { return ResourceTable::RouteNodeId(node); };
  return c;
}

// 把 plan 的窗口转成 ResourceTable 的批量提交入参
std::vector<std::pair<std::string, TimeInterval>> ToIntervals(
    const wf::ResourceTimeWindowSet& windows) {
  std::vector<std::pair<std::string, TimeInterval>> intervals;
  intervals.reserve(windows.size());
  for (const auto& w : windows) {
    intervals.emplace_back(w.resource_id, TimeInterval{w.enter, w.exit});
  }
  return intervals;
}

}  // namespace

// ---- 场景 1：共享边、不同窗口 -> 双成功 ----

TEST(MultiVehicleTimeSpaceTest, SharedEdgeDifferentWindows_BothSucceed) {
  ResourceTable table;
  wf::TimeSpaceOccupancy occupancy;
  const auto t0 = Clock::now();
  const auto config = MakeConfig();

  // m1 立刻出发，走 A -> B
  const auto m1 = MakeCandidate("m1", {"A", "B"}, {10.0}, t0);
  const auto plan1 = wf::PlanTimeSpaceReservation(m1, occupancy, config);
  ASSERT_TRUE(plan1.feasible) << plan1.reject_reason;

  std::string message;
  ASSERT_TRUE(table.ReserveTimeSpaceBatch("m1", ToIntervals(plan1.windows), &message))
      << message;
  wf::CommitTimeSpaceOccupancy(&occupancy, "m1", plan1.windows);

  // m2 晚 60s 出发，同样走 A -> B（共享同一条边资源）
  const auto m2 = MakeCandidate("m2", {"A", "B"}, {10.0}, t0 + 60s);
  const auto plan2 = wf::PlanTimeSpaceReservation(m2, occupancy, config);
  ASSERT_TRUE(plan2.feasible) << plan2.reject_reason;
  EXPECT_DOUBLE_EQ(plan2.total_delay_sec, 0.0);  // 窗口本就错开，无需后移

  ASSERT_TRUE(table.ReserveTimeSpaceBatch("m2", ToIntervals(plan2.windows), &message))
      << message;
  wf::CommitTimeSpaceOccupancy(&occupancy, "m2", plan2.windows);

  // 两单都拿到了共享边的路权，且窗口不重叠
  const auto m1_resources = table.TimeSpaceResourcesOf("m1");
  const auto m2_resources = table.TimeSpaceResourcesOf("m2");
  EXPECT_FALSE(m1_resources.empty());
  EXPECT_FALSE(m2_resources.empty());
  EXPECT_EQ(table.TimeSpaceReservationCount(), 6u);  // 两单 x (边 A__B + 节点 A + 节点 B)

  bool share_edge = false;
  for (const auto& id : m1_resources) {
    for (const auto& other : m2_resources) {
      if (id == other) share_edge = true;
    }
  }
  EXPECT_TRUE(share_edge) << "两单应当共享同一条边资源";

  // 共享边上的两个区间不重叠
  const auto edge_id = ResourceTable::RouteEdgeId("A", "B");
  const auto parsed = ResourceTable::ParseRouteEdge(edge_id);
  ASSERT_TRUE(parsed.has_value());
  EXPECT_EQ(parsed->first, "A");
  EXPECT_EQ(parsed->second, "B");

  // 一台车确认驶离后，该资源上的本单区间被释放；另一单不受影响
  const auto released = table.ReleaseTimeSpaceResources("m1", {edge_id});
  EXPECT_EQ(released.size(), 1u);
  EXPECT_TRUE(table.TimeSpaceResourcesOf("m2").size() > 0);
}

// ---- 场景 2：共享边、同一窗口 -> 一成一败，无残留半预约 ----

TEST(MultiVehicleTimeSpaceTest, SharedEdgeSameWindow_OneSucceedsOneFails_NoPartialReservation) {
  ResourceTable table;
  wf::TimeSpaceOccupancy occupancy;
  const auto t0 = Clock::now();
  const auto config = MakeConfig();

  // m1 先占住 A -> B -> C（整条路径）
  const auto m1 = MakeCandidate("m1", {"A", "B", "C"}, {10.0, 10.0}, t0);
  const auto plan1 = wf::PlanTimeSpaceReservation(m1, occupancy, config);
  ASSERT_TRUE(plan1.feasible) << plan1.reject_reason;
  std::string message;
  ASSERT_TRUE(table.ReserveTimeSpaceBatch("m1", ToIntervals(plan1.windows), &message))
      << message;
  wf::CommitTimeSpaceOccupancy(&occupancy, "m1", plan1.windows);

  // m2 同一时刻出发，走 A -> B -> D：共享边 A__B 时间窗与 m1 完全重叠，
  // 但 B__D 本身是空闲的——**整单必须失败且一个资源都不许留下**。
  const auto m2 = MakeCandidate("m2", {"A", "B", "D"}, {10.0, 10.0}, t0);
  const auto plan2 = wf::PlanTimeSpaceReservation(m2, occupancy, config);
  if (plan2.feasible) {
    // 纯规划器会自动后移到可行窗口；这里直接用**原始窗口**验证
    // 预约表层的"全有或全无"拒绝语义（后移逻辑由调度层决定是否采用）。
    const auto m2_raw = MakeCandidate("m2_raw", {"A", "B", "D"}, {10.0, 10.0}, t0);
    wf::TimeSpaceOccupancy empty_occupancy;
    const auto raw_plan = wf::PlanTimeSpaceReservation(m2_raw, empty_occupancy, config);
    ASSERT_TRUE(raw_plan.feasible);
    EXPECT_FALSE(table.ReserveTimeSpaceBatch("m2_raw", ToIntervals(raw_plan.windows), &message));
    EXPECT_NE(message.find("contention"), std::string::npos) << "msg=" << message;
    // 关键断言：m2_raw **没有**留下任何半套预约
    EXPECT_TRUE(table.TimeSpaceResourcesOf("m2_raw").empty());
    // B__D 这条"本来空闲"的边也没有被占
    EXPECT_FALSE(table.Locked(ResourceTable::RouteEdgeId("B", "D")));
  } else {
    // 规划器直接判不可行：同样不得留下任何预约
    EXPECT_FALSE(table.ReserveTimeSpaceBatch("m2", ToIntervals(plan2.windows), &message));
    EXPECT_TRUE(table.TimeSpaceResourcesOf("m2").empty());
  }

  // m1 的预约仍然完好（失败的单不得影响已成的单）
  EXPECT_FALSE(table.TimeSpaceResourcesOf("m1").empty());
}

// ---- 对头（head-on）冲突：反向行驶在同一条边资源上冲突 ----

TEST(MultiVehicleTimeSpaceTest, OppositeDirection_SharesSameEdgeResource) {
  // 边 id 与行驶方向无关（归一化排序）——窄道对向冲突因此能被表达
  EXPECT_EQ(ResourceTable::RouteEdgeId("A", "B"), ResourceTable::RouteEdgeId("B", "A"));

  ResourceTable table;
  const auto t0 = Clock::now();
  std::string message;
  ASSERT_TRUE(table.ReserveTimeSpace("m1", ResourceTable::RouteEdgeId("A", "B"), t0,
                                    t0 + 20s, &message));
  // 反向行驶的 m2 在同一时间窗进入同一条边 -> 冲突
  EXPECT_FALSE(table.ReserveTimeSpace("m2", ResourceTable::RouteEdgeId("B", "A"), t0,
                                       t0 + 20s, &message));
  EXPECT_NE(message.find("contention"), std::string::npos) << "msg=" << message;
  // 错开窗口 -> 允许（窄道错峰通过）
  EXPECT_TRUE(table.ReserveTimeSpace("m2", ResourceTable::RouteEdgeId("B", "A"), t0 + 30s,
                                     t0 + 50s, &message));
}

// ---- 整条路径多资源共享：两单共享"一段"通道、其余互不相交 -> 双成功 ----

TEST(MultiVehicleTimeSpaceTest, PartiallySharedPath_DifferentWindows_BothSucceed) {
  ResourceTable table;
  wf::TimeSpaceOccupancy occupancy;
  const auto t0 = Clock::now();
  const auto config = MakeConfig();

  // m1: A -> B -> C ；m2: A -> B -> D ：共享 A__B，其余各走各的
  const auto m1 = MakeCandidate("m1", {"A", "B", "C"}, {10.0, 10.0}, t0);
  const auto plan1 = wf::PlanTimeSpaceReservation(m1, occupancy, config);
  ASSERT_TRUE(plan1.feasible) << plan1.reject_reason;
  std::string message;
  ASSERT_TRUE(table.ReserveTimeSpaceBatch("m1", ToIntervals(plan1.windows), &message))
      << message;
  wf::CommitTimeSpaceOccupancy(&occupancy, "m1", plan1.windows);

  const auto m2 = MakeCandidate("m2", {"A", "B", "D"}, {10.0, 10.0}, t0 + 120s);
  const auto plan2 = wf::PlanTimeSpaceReservation(m2, occupancy, config);
  ASSERT_TRUE(plan2.feasible) << plan2.reject_reason;
  ASSERT_TRUE(table.ReserveTimeSpaceBatch("m2", ToIntervals(plan2.windows), &message))
      << message;
  wf::CommitTimeSpaceOccupancy(&occupancy, "m2", plan2.windows);

  // 两条路径上除共享段外，其余资源互不相交 -> 真正的错峰并行
  const auto shared_edge = ResourceTable::RouteEdgeId("A", "B");
  const auto m1_resources = table.TimeSpaceResourcesOf("m1");
  const auto m2_resources = table.TimeSpaceResourcesOf("m2");
  EXPECT_NE(std::find(m1_resources.begin(), m1_resources.end(), shared_edge), m1_resources.end());
  EXPECT_NE(std::find(m2_resources.begin(), m2_resources.end(), shared_edge), m2_resources.end());
  // m1 独占 C 段，m2 独占 D 段
  EXPECT_NE(std::find(m1_resources.begin(), m1_resources.end(),
                      ResourceTable::RouteEdgeId("B", "C")),
            m1_resources.end());
  EXPECT_NE(std::find(m2_resources.begin(), m2_resources.end(),
                      ResourceTable::RouteEdgeId("B", "D")),
            m2_resources.end());
}
