// Task 5（P3）：逐段路权动态释放与断链安全守卫 —— 纯核心单测
//
// Rule 1（Spec §5.3 逐段释放）：小车 state 确认 lastNodeId 沿订单路径推进后，
// 释放其身后已脱离的路由锁；当前位置的物理足迹（当前节点 + 正在行驶/刚驶离的边）
// 保持锁定。lastNodeId 不在订单路径上、或 lastNodeSequenceId 与订单不一致时，
// 一律不释放（保守处置），返回处置指引。
//
// 足迹规则（保守）：确认位置 = path[k] 时
//   释放：path[0..k-1] 的节点锁 + path[0..k-2] 的边锁（即完全越过的部分）
//   保留：path[k] 节点锁（当前所在节点）、(k-1,k) 边锁（刚驶离/尾部悬跨）、
//         (k,k+1) 边锁（正在行驶）以及前方全部锁
//
// Rule 2（Spec §5.2 断链锁定保护）：online=false（LWT CONNECTIONBROKEN / 心跳超时）
// 或状态陈旧时，任何链路守卫决策都必须保留全部锁（retain_all_locks 恒 true），
// 离线另需挂起派发 + 看门狗告警。
//
// 本文件仅依赖 amr_dispatcher_core（零 ROS/MQTT）。

#include "amr_dispatcher_core/workflow/dynamic_unlocking.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <string>
#include <unordered_map>
#include <vector>

#include "amr_dispatcher_core/dispatcher/traffic_reservation.hpp"

namespace amr_dispatcher_core::workflow {
namespace {

using dispatcher::ResourceTable;

constexpr const char* kMission = "mission_alpha";

// 4 节点折线路径：station_a -> station_b -> station_c -> station_d
const std::vector<std::string> kPath = {"station_a", "station_b", "station_c", "station_d"};

// 订单全量锁（含路口节点锁与边锁），与 PlanMissionRouteLocks 的产出同形。
std::vector<std::string> AllPathLocks(const std::vector<std::string>& path) {
  std::vector<std::string> ids;
  for (const auto& node : path) {
    ids.push_back(ResourceTable::RouteNodeId(node));
  }
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    ids.push_back(ResourceTable::RouteEdgeId(path[i], path[i + 1]));
  }
  return ids;
}

// 构造 resource_id -> owner 映射（默认全部归属 kMission，可注入他人锁）。
std::unordered_map<std::string, std::string> BuildHeld(
    const std::vector<std::string>& lock_ids, const std::string& owner = kMission) {
  std::unordered_map<std::string, std::string> held;
  for (const auto& id : lock_ids) {
    held[id] = owner;
  }
  return held;
}

bool Owns(const std::unordered_map<std::string, std::string>& held, const std::string& id,
          const std::string& owner) {
  const auto it = held.find(id);
  return it != held.end() && it->second == owner;
}

bool Contains(const std::vector<std::string>& vec, const std::string& v) {
  return std::find(vec.begin(), vec.end(), v) != vec.end();
}

std::string NodeLock(const std::string& id) { return ResourceTable::RouteNodeId(id); }
std::string EdgeLock(const std::string& from, const std::string& to) {
  return ResourceTable::RouteEdgeId(from, to);
}

// ---------------- Rule 1：逐段释放 ----------------

TEST(DynamicUnlockingTest, AtFirstNodePathIndex0_ReleasesNothing_FootprintKeepsWholePath) {
  const auto held = BuildHeld(AllPathLocks(kPath));

  // 车辆确认在 path[0]（lastNodeSequenceId = 0*2）：身后没有任何路网，全路径保持锁定
  const auto decision = PlanTrailingUnlock(kPath, "station_a", 0, kMission, held);

  EXPECT_TRUE(decision.confirmed);
  EXPECT_TRUE(decision.lock_ids_to_release.empty());
}

TEST(DynamicUnlockingTest, AtPathIndex1_ReleasesOnlyDepartedNode0_KeepsTraversedEdgeABLocked) {
  const auto held = BuildHeld(AllPathLocks(kPath));

  // 车辆确认推进到 path[1]（sequence = 1*2）：只释放出发节点，
  // 足迹节点 path[1] 与仍在占用/刚驶离的边 (a,b) 保持锁定
  const auto decision = PlanTrailingUnlock(kPath, "station_b", 2, kMission, held);

  EXPECT_TRUE(decision.confirmed);
  ASSERT_EQ(decision.lock_ids_to_release.size(), 1U);
  EXPECT_EQ(decision.lock_ids_to_release[0], NodeLock("station_a"));
  EXPECT_FALSE(Contains(decision.lock_ids_to_release, EdgeLock("station_a", "station_b")));
  EXPECT_FALSE(Contains(decision.lock_ids_to_release, NodeLock("station_b")));
}

TEST(DynamicUnlockingTest, AtPathIndex2_ReleasesNode0EdgeABNode1_KeepsEdgeBCLocked) {
  const auto held = BuildHeld(AllPathLocks(kPath));

  // 车辆确认推进到 path[2]（sequence = 2*2）：释放 node_a + edge(a,b) + node_b，
  // 足迹边 (b,c)、当前节点 path[2] 与前方 (c,d) 保持锁定
  const auto decision = PlanTrailingUnlock(kPath, "station_c", 4, kMission, held);

  EXPECT_TRUE(decision.confirmed);
  EXPECT_TRUE(Contains(decision.lock_ids_to_release, NodeLock("station_a")));
  EXPECT_TRUE(Contains(decision.lock_ids_to_release, EdgeLock("station_a", "station_b")));
  EXPECT_TRUE(Contains(decision.lock_ids_to_release, NodeLock("station_b")));
  EXPECT_EQ(decision.lock_ids_to_release.size(), 3U);

  EXPECT_FALSE(Contains(decision.lock_ids_to_release, EdgeLock("station_b", "station_c")));
  EXPECT_FALSE(Contains(decision.lock_ids_to_release, NodeLock("station_c")));
  EXPECT_FALSE(Contains(decision.lock_ids_to_release, NodeLock("station_d")));
  EXPECT_FALSE(Contains(decision.lock_ids_to_release, EdgeLock("station_c", "station_d")));
}

TEST(DynamicUnlockingTest, AtTerminalPathIndex3_ReleasesThroughNode2_KeepsTerminalNodeAndEdgeCD) {
  const auto held = BuildHeld(AllPathLocks(kPath));

  // 车辆确认到达末节点 path[3]（sequence = 3*2）：身后全部越过部分释放，
  // 终点节点与其到达边 (c,d) 保持锁定（终点执行动作期间的物理足迹）
  const auto decision = PlanTrailingUnlock(kPath, "station_d", 6, kMission, held);

  EXPECT_TRUE(decision.confirmed);
  EXPECT_TRUE(Contains(decision.lock_ids_to_release, NodeLock("station_a")));
  EXPECT_TRUE(Contains(decision.lock_ids_to_release, EdgeLock("station_a", "station_b")));
  EXPECT_TRUE(Contains(decision.lock_ids_to_release, NodeLock("station_b")));
  EXPECT_TRUE(Contains(decision.lock_ids_to_release, EdgeLock("station_b", "station_c")));
  EXPECT_TRUE(Contains(decision.lock_ids_to_release, NodeLock("station_c")));
  EXPECT_EQ(decision.lock_ids_to_release.size(), 5U);

  EXPECT_FALSE(Contains(decision.lock_ids_to_release, EdgeLock("station_c", "station_d")));
  EXPECT_FALSE(Contains(decision.lock_ids_to_release, NodeLock("station_d")));
}

TEST(DynamicUnlockingTest, UnknownLastNodeId_ReleasesNothingAndCarriesGuidance) {
  const auto held = BuildHeld(AllPathLocks(kPath));

  const auto decision = PlanTrailingUnlock(kPath, "station_z", 0, kMission, held);

  EXPECT_FALSE(decision.confirmed);
  EXPECT_TRUE(decision.lock_ids_to_release.empty());
  EXPECT_FALSE(decision.message.empty());  // 保守处置须附带指引日志
}

TEST(DynamicUnlockingTest, SequenceIdMismatchWithOrder_ReleasesNothingAndCarriesGuidance) {
  const auto held = BuildHeld(AllPathLocks(kPath));

  // lastNodeId=station_b 应配 sequence=2，上报 4 与订单不符 -> 释放空集
  const auto decision = PlanTrailingUnlock(kPath, "station_b", 4, kMission, held);

  EXPECT_FALSE(decision.confirmed);
  EXPECT_TRUE(decision.lock_ids_to_release.empty());
  EXPECT_FALSE(decision.message.empty());
}

TEST(DynamicUnlockingTest, EmptyLastNodeId_ReleasesNothing) {
  const auto held = BuildHeld(AllPathLocks(kPath));

  const auto decision = PlanTrailingUnlock(kPath, "", 0, kMission, held);

  EXPECT_FALSE(decision.confirmed);
  EXPECT_TRUE(decision.lock_ids_to_release.empty());
}

TEST(DynamicUnlockingTest, ForeignOwnedTrailingLocks_NeverReturnedForRelease) {
  // 身后的三个锁属于别的任务（撞单场景），足迹锁属于本任务
  auto held = BuildHeld({NodeLock("station_a"), EdgeLock("station_a", "station_b"),
                         NodeLock("station_b")},
                        "mission_beta");
  for (const auto& id : AllPathLocks(kPath)) {
    if (!Owns(held, id, "mission_beta")) {
      held[id] = kMission;
    }
  }

  const auto decision = PlanTrailingUnlock(kPath, "station_c", 4, kMission, held);

  EXPECT_TRUE(decision.confirmed);  // 位置本身是确认的
  // k=2 的理论释放集 {node_a, edge_ab, node_b} 全部归他人 -> 实际释放集为空
  EXPECT_TRUE(decision.lock_ids_to_release.empty());
  for (const auto& id : decision.lock_ids_to_release) {
    EXPECT_TRUE(Owns(held, id, kMission)) << "released foreign lock: " << id;
  }
}

TEST(DynamicUnlockingTest, SingleNodePath_ReleasesNothing) {
  const std::vector<std::string> single = {"station_a"};
  const auto held = BuildHeld(AllPathLocks(single));

  const auto decision = PlanTrailingUnlock(single, "station_a", 0, kMission, held);

  EXPECT_TRUE(decision.confirmed);
  EXPECT_TRUE(decision.lock_ids_to_release.empty());
}

TEST(DynamicUnlockingTest, HeldLocksMissingFromSnapshot_DoesNotAffectOwnedReleaseSet) {
  // 快照中根本没有的锁 id 不会凭空出现（释放集只来自路径推导 ∩ 实际持有）
  auto held = BuildHeld({NodeLock("station_a"), EdgeLock("station_a", "station_b")});

  const auto decision = PlanTrailingUnlock(kPath, "station_c", 4, kMission, held);

  EXPECT_TRUE(decision.confirmed);
  // k=2 理论释放 {node_a, edge_ab, node_b}，但 held 中没有 node_b -> 只返回实际持有的两个
  ASSERT_EQ(decision.lock_ids_to_release.size(), 2U);
  EXPECT_TRUE(Contains(decision.lock_ids_to_release, NodeLock("station_a")));
  EXPECT_TRUE(Contains(decision.lock_ids_to_release, EdgeLock("station_a", "station_b")));
}

// ---------------- Rule 1：ResourceTable 部分释放（幂等 + 所有权） ----------------

TEST(DynamicUnlockingTest, Idempotency_RecomputeAfterPartialRelease_YieldsNothingLeft) {
  ResourceTable table;
  const auto all_locks = AllPathLocks(kPath);
  ASSERT_TRUE(table.Reserve(kMission, all_locks).success);

  // 第一次计算：k=2 应释放身后 3 把锁
  auto held = BuildHeld(all_locks);
  const auto first = PlanTrailingUnlock(kPath, "station_c", 4, kMission, held);
  ASSERT_TRUE(first.confirmed);
  ASSERT_EQ(first.lock_ids_to_release.size(), 3U);

  const auto released = table.ReleaseResources(kMission, first.lock_ids_to_release);
  EXPECT_EQ(released.size(), 3U);

  // 第二次计算：输入完全相同（小车重复上报），但足迹锁仍在、身后锁已不持有
  std::unordered_map<std::string, std::string> snapshot_held;
  for (const auto& [resource_id, owner] : table.Snapshot()) {
    snapshot_held[resource_id] = owner;
  }
  const auto second = PlanTrailingUnlock(kPath, "station_c", 4, kMission, snapshot_held);
  EXPECT_TRUE(second.confirmed);
  EXPECT_TRUE(second.lock_ids_to_release.empty());  // 无剩余可释放 -> 幂等

  // 足迹锁仍被本任务持有
  EXPECT_TRUE(table.Locked(NodeLock("station_c")));
  EXPECT_TRUE(table.Locked(EdgeLock("station_b", "station_c")));
  EXPECT_TRUE(table.Locked(EdgeLock("station_c", "station_d")));

  // 对已释放 id 再次部分释放是安全 no-op
  EXPECT_TRUE(table.ReleaseResources(kMission, first.lock_ids_to_release).empty());
}

TEST(DynamicUnlockingTest, ReleaseResources_SkipsIdsOwnedByOtherMissions) {
  ResourceTable table;
  ASSERT_TRUE(table.Reserve("mission_beta", {NodeLock("station_b")}).success);
  ASSERT_TRUE(table.Reserve(kMission, {NodeLock("station_a")}).success);

  const auto released = table.ReleaseResources(
      kMission, {NodeLock("station_a"), NodeLock("station_b")});

  // 只释放自己持有的；他人锁纹丝不动
  ASSERT_EQ(released.size(), 1U);
  EXPECT_EQ(released[0], NodeLock("station_a"));
  EXPECT_FALSE(table.Locked(NodeLock("station_a")));
  EXPECT_TRUE(table.Locked(NodeLock("station_b")));
  EXPECT_EQ(table.Owner(NodeLock("station_b")), "mission_beta");

  // traffic_block 锁对任务部分释放不可见
  std::string msg;
  ASSERT_TRUE(table.Block(NodeLock("station_c"), "repair", &msg));
  EXPECT_TRUE(table.ReleaseResources(kMission, {NodeLock("station_c")}).empty());
  EXPECT_TRUE(table.Locked(NodeLock("station_c")));
}

// ---------------- Rule 2：断链安全守卫 ----------------

TEST(DynamicUnlockingTest, LinkGuard_OfflineVehicle_RetainsAllLocksBlocksDispatchAndWarns) {
  const auto guard = EvaluateVehicleLinkGuard(/*vehicle_online=*/false, /*state_stale=*/false);

  EXPECT_TRUE(guard.retain_all_locks);      // 铁律：离线绝不释放
  EXPECT_TRUE(guard.block_new_dispatch);    // 挂起向该车的新派发
  EXPECT_TRUE(guard.emit_watchdog_warning); // 软看门狗告警
  EXPECT_FALSE(guard.message.empty());
}

TEST(DynamicUnlockingTest, LinkGuard_StaleStateWhileOnline_RetainsLocksWarnsWithoutRelease) {
  // 在线但 state 陈旧：告警、保留锁；状态陈旧本身不构成释放依据
  const auto guard = EvaluateVehicleLinkGuard(/*vehicle_online=*/true, /*state_stale=*/true);

  EXPECT_TRUE(guard.retain_all_locks);
  EXPECT_TRUE(guard.emit_watchdog_warning);
  EXPECT_FALSE(guard.message.empty());
}

TEST(DynamicUnlockingTest, LinkGuard_OnlineFresh_RetainsLocksByDefault_AllowsDispatch) {
  const auto guard = EvaluateVehicleLinkGuard(/*vehicle_online=*/true, /*state_stale=*/false);

  EXPECT_TRUE(guard.retain_all_locks);  // 释放只来自 Rule 1 的显式位置推进
  EXPECT_FALSE(guard.block_new_dispatch);
  EXPECT_FALSE(guard.emit_watchdog_warning);
}

}  // namespace
}  // namespace amr_dispatcher_core::workflow
