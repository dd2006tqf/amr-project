// 选车策略纯函数 PlanVehicleAssignment 的单测（v2 修订语义）。
//
// 覆盖点：
//   1. 显式指定：命中即 assigned/strategy="explicit"；不满足硬约束 -> rejected；
//      车忙 -> deferred（留队等待，不判负）
//   2. 自动选车硬约束：online ∧ enabled ∧ !busy ∧ !quarantined ∧ 能力 ∧ 电量
//      ∧ 位置有效 ∧ 状态新鲜
//   3. 三态结果：deferred（车忙）/ indeterminate（从未收到状态）/ rejected（永久不满足）
//   4. 距离不可算 -> 不盲派（rejected），不再退化到清单首台
//   5. 空 capabilities 视为"无能力限制"

#include "amr_dispatcher_core/fleet/vehicle_assignment.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <string>
#include <vector>

namespace fleet = amr_dispatcher_core::fleet;

namespace {

// 新鲜状态时刻（当前时刻）
fleet::VehicleRuntimeState Fresh(const std::string& id,
                                 std::vector<std::string> capabilities = {"transport"}) {
  fleet::VehicleRuntimeState v;
  v.vehicle_id = id;
  v.online = true;
  v.busy = false;
  v.enabled = true;
  v.battery_percent = 100.0;
  v.capabilities = std::move(capabilities);
  v.position_valid = true;
  v.current_station_id = "S0";
  v.state_time = std::chrono::steady_clock::now();
  return v;
}

// 从未收到过状态的车
fleet::VehicleRuntimeState NeverSeen(const std::string& id) {
  fleet::VehicleRuntimeState v = Fresh(id);
  v.state_time = fleet::VehicleRuntimeState{}.state_time;  // 默认构造值
  v.position_valid = false;
  return v;
}

fleet::VehicleAssignmentRequest MakeRequest(std::vector<fleet::VehicleRuntimeState> vehicles) {
  fleet::VehicleAssignmentRequest req;
  req.vehicles = std::move(vehicles);
  req.pickup_station_id = "A";
  req.dropoff_station_id = "B";
  req.required_capability = "transport";
  req.min_battery_percent = 0.0;
  req.state_max_age_ms = 5000;
  return req;
}

}  // namespace

// ---- 显式指定 ----

TEST(VehicleAssignmentTest, ExplicitVehicle_Valid_Assigns) {
  auto req = MakeRequest({Fresh("v1"), Fresh("v2")});
  req.requested_vehicle_id = "v1";
  const auto d = fleet::PlanVehicleAssignment(req);
  EXPECT_EQ(d.outcome, fleet::AssignmentOutcome::kAssigned);
  EXPECT_EQ(d.vehicle_id, "v1");
  EXPECT_EQ(d.strategy, "explicit");
}

TEST(VehicleAssignmentTest, ExplicitVehicle_Unknown_IsRejected_NotReassigned) {
  auto req = MakeRequest({Fresh("v1")});
  req.requested_vehicle_id = "ghost";
  const auto d = fleet::PlanVehicleAssignment(req);
  EXPECT_EQ(d.outcome, fleet::AssignmentOutcome::kRejected);
  EXPECT_TRUE(d.vehicle_id.empty());
}

TEST(VehicleAssignmentTest, ExplicitVehicle_Offline_IsDeferred_NotRejected) {
  // 离线是**暂时**状态（会自行恢复）：任务留队等待，而不是判负
  auto v = Fresh("v1");
  v.online = false;
  auto req = MakeRequest({v});
  req.requested_vehicle_id = "v1";
  const auto d = fleet::PlanVehicleAssignment(req);
  EXPECT_EQ(d.outcome, fleet::AssignmentOutcome::kDeferred);
  EXPECT_EQ(d.strategy, "deferred");
}

TEST(VehicleAssignmentTest, ExplicitVehicle_Disabled_IsRejected) {
  auto v = Fresh("v1");
  v.enabled = false;
  auto req = MakeRequest({v});
  req.requested_vehicle_id = "v1";
  const auto d = fleet::PlanVehicleAssignment(req);
  EXPECT_EQ(d.outcome, fleet::AssignmentOutcome::kRejected);
}

TEST(VehicleAssignmentTest, ExplicitVehicle_Quarantined_IsDeferred_NotRejected) {
  // 隔离态待人工恢复：任务留队等待，不判负
  auto v = Fresh("v1");
  v.quarantined = true;
  auto req = MakeRequest({v});
  req.requested_vehicle_id = "v1";
  const auto d = fleet::PlanVehicleAssignment(req);
  EXPECT_EQ(d.outcome, fleet::AssignmentOutcome::kDeferred);
}

TEST(VehicleAssignmentTest, ExplicitVehicle_MissingCapability_IsRejected) {
  auto req = MakeRequest({Fresh("v1", {"charging"})});
  req.requested_vehicle_id = "v1";
  const auto d = fleet::PlanVehicleAssignment(req);
  EXPECT_EQ(d.outcome, fleet::AssignmentOutcome::kRejected);
}

TEST(VehicleAssignmentTest, ExplicitVehicle_BelowBatteryThreshold_IsRejected) {
  auto v = Fresh("v1");
  v.battery_percent = 5.0;
  auto req = MakeRequest({v});
  req.requested_vehicle_id = "v1";
  req.min_battery_percent = 20.0;
  const auto d = fleet::PlanVehicleAssignment(req);
  EXPECT_EQ(d.outcome, fleet::AssignmentOutcome::kRejected);
}

TEST(VehicleAssignmentTest, ExplicitVehicle_Busy_IsDeferred_NotRejected) {
  auto v = Fresh("v1");
  v.busy = true;
  auto req = MakeRequest({v});
  req.requested_vehicle_id = "v1";
  const auto d = fleet::PlanVehicleAssignment(req);
  // 车忙是暂时状态：任务应留队等待，而不是被判负
  EXPECT_EQ(d.outcome, fleet::AssignmentOutcome::kDeferred);
  EXPECT_EQ(d.strategy, "deferred");
  EXPECT_EQ(d.blocking_vehicle_id, "v1");
}

TEST(VehicleAssignmentTest, ExplicitVehicle_IgnoresPositionValidity) {
  // 人工点名：知道车在哪，位置未上报不构成拒绝理由
  auto v = Fresh("v1");
  v.position_valid = false;
  auto req = MakeRequest({v});
  req.requested_vehicle_id = "v1";
  const auto d = fleet::PlanVehicleAssignment(req);
  EXPECT_EQ(d.outcome, fleet::AssignmentOutcome::kAssigned);
}

TEST(VehicleAssignmentTest, ExplicitVehicle_WinsOverNearerAutoCandidate) {
  auto req = MakeRequest({Fresh("v1"), Fresh("v2")});
  req.requested_vehicle_id = "v2";
  req.distance_to_pickup_m = {{"v1", 1.0}, {"v2", 99.0}};
  const auto d = fleet::PlanVehicleAssignment(req);
  EXPECT_EQ(d.outcome, fleet::AssignmentOutcome::kAssigned);
  EXPECT_EQ(d.vehicle_id, "v2");
  EXPECT_EQ(d.strategy, "explicit");
}

// ---- 自动选车 ----

TEST(VehicleAssignmentTest, AutoSelect_NearestWins) {
  auto req = MakeRequest({Fresh("v1"), Fresh("v2")});
  req.distance_to_pickup_m = {{"v1", 10.0}, {"v2", 2.0}};
  const auto d = fleet::PlanVehicleAssignment(req);
  EXPECT_EQ(d.outcome, fleet::AssignmentOutcome::kAssigned);
  EXPECT_EQ(d.vehicle_id, "v2");
  EXPECT_EQ(d.strategy, "nearest");
}

TEST(VehicleAssignmentTest, AutoSelect_TieBreaksByInputOrder) {
  auto req = MakeRequest({Fresh("v1"), Fresh("v2")});
  req.distance_to_pickup_m = {{"v1", 5.0}, {"v2", 5.0}};
  const auto d = fleet::PlanVehicleAssignment(req);
  EXPECT_EQ(d.outcome, fleet::AssignmentOutcome::kAssigned);
  EXPECT_EQ(d.vehicle_id, "v1");
}

TEST(VehicleAssignmentTest, AutoSelect_NoDistanceForAnyCandidate_IsRejected_NotBlindDispatch) {
  auto req = MakeRequest({Fresh("v1"), Fresh("v2")});
  const auto d = fleet::PlanVehicleAssignment(req);
  // 距离全部不可算 -> 不盲派（不派给可能到不了取货点的车）
  EXPECT_EQ(d.outcome, fleet::AssignmentOutcome::kRejected);
  EXPECT_TRUE(d.vehicle_id.empty());
}

TEST(VehicleAssignmentTest, AutoSelect_CandidateWithoutDistanceRanksAfterThoseWithDistance) {
  auto req = MakeRequest({Fresh("v1"), Fresh("v2")});
  req.distance_to_pickup_m = {{"v2", 7.0}};  // v1 无距离
  const auto d = fleet::PlanVehicleAssignment(req);
  EXPECT_EQ(d.outcome, fleet::AssignmentOutcome::kAssigned);
  EXPECT_EQ(d.vehicle_id, "v2");
}

TEST(VehicleAssignmentTest, AutoSelect_AllBusy_IsDeferred) {
  auto v1 = Fresh("v1");
  v1.busy = true;
  auto v2 = Fresh("v2");
  v2.busy = true;
  auto req = MakeRequest({v1, v2});
  req.distance_to_pickup_m = {{"v1", 1.0}, {"v2", 2.0}};
  const auto d = fleet::PlanVehicleAssignment(req);
  EXPECT_EQ(d.outcome, fleet::AssignmentOutcome::kDeferred);
}

TEST(VehicleAssignmentTest, AutoSelect_NeverSeenState_IsIndeterminate) {
  auto req = MakeRequest({NeverSeen("v1")});
  const auto d = fleet::PlanVehicleAssignment(req);
  // 从未收到状态 -> 状态不确定：暂停自动分配，等状态恢复
  EXPECT_EQ(d.outcome, fleet::AssignmentOutcome::kIndeterminate);
  EXPECT_EQ(d.strategy, "indeterminate");
}

TEST(VehicleAssignmentTest, AutoSelect_StaleState_IsDeferred) {
  auto v = Fresh("v1");
  v.state_time = std::chrono::steady_clock::now() - std::chrono::seconds(30);
  auto req = MakeRequest({v});
  const auto d = fleet::PlanVehicleAssignment(req);
  EXPECT_EQ(d.outcome, fleet::AssignmentOutcome::kDeferred);
}

TEST(VehicleAssignmentTest, AutoSelect_InvalidPosition_IsDeferred) {
  auto v = Fresh("v1");
  v.position_valid = false;
  auto req = MakeRequest({v});
  const auto d = fleet::PlanVehicleAssignment(req);
  EXPECT_EQ(d.outcome, fleet::AssignmentOutcome::kDeferred);
}

TEST(VehicleAssignmentTest, AutoSelect_OfflineAndDisabledAndQuarantinedExcluded) {
  auto off = Fresh("v_off");
  off.online = false;
  auto dis = Fresh("v_dis");
  dis.enabled = false;
  auto qua = Fresh("v_qua");
  qua.quarantined = true;
  auto req = MakeRequest({off, dis, qua, Fresh("v_ok")});
  req.distance_to_pickup_m = {{"v_off", 0.1}, {"v_dis", 0.2}, {"v_qua", 0.3}, {"v_ok", 9.0}};
  const auto d = fleet::PlanVehicleAssignment(req);
  EXPECT_EQ(d.outcome, fleet::AssignmentOutcome::kAssigned);
  EXPECT_EQ(d.vehicle_id, "v_ok");
}

TEST(VehicleAssignmentTest, AutoSelect_CapabilityFilterExcludes) {
  auto req = MakeRequest({Fresh("v1", {"charging"}), Fresh("v2", {"transport"})});
  req.distance_to_pickup_m = {{"v1", 0.1}, {"v2", 9.0}};
  const auto d = fleet::PlanVehicleAssignment(req);
  EXPECT_EQ(d.outcome, fleet::AssignmentOutcome::kAssigned);
  EXPECT_EQ(d.vehicle_id, "v2");
}

TEST(VehicleAssignmentTest, AutoSelect_BatteryThresholdExcludes) {
  auto low = Fresh("v_low");
  low.battery_percent = 5.0;
  auto req = MakeRequest({low, Fresh("v_ok")});
  req.min_battery_percent = 20.0;
  req.distance_to_pickup_m = {{"v_low", 0.1}, {"v_ok", 9.0}};
  const auto d = fleet::PlanVehicleAssignment(req);
  EXPECT_EQ(d.outcome, fleet::AssignmentOutcome::kAssigned);
  EXPECT_EQ(d.vehicle_id, "v_ok");
}

TEST(VehicleAssignmentTest, AutoSelect_NoEligible_RejectedWithReason) {
  auto v = Fresh("v1", {"charging"});
  auto req = MakeRequest({v});
  const auto d = fleet::PlanVehicleAssignment(req);
  EXPECT_EQ(d.outcome, fleet::AssignmentOutcome::kRejected);
  EXPECT_NE(d.reject_reason.find("capability"), std::string::npos);
}

TEST(VehicleAssignmentTest, EmptyVehicleList_Rejected) {
  auto req = MakeRequest({});
  const auto d = fleet::PlanVehicleAssignment(req);
  EXPECT_EQ(d.outcome, fleet::AssignmentOutcome::kRejected);
}

TEST(VehicleAssignmentTest, AutoSelect_AllOffline_IsDeferred_NotRejected) {
  auto v = Fresh("v1");
  v.online = false;
  auto req = MakeRequest({v});
  req.distance_to_pickup_m = {{"v1", 1.0}};
  const auto d = fleet::PlanVehicleAssignment(req);
  // 全部车离线 -> 暂时不可用：留队等链路恢复
  EXPECT_EQ(d.outcome, fleet::AssignmentOutcome::kDeferred);
}

TEST(VehicleAssignmentTest, AutoSelect_AllQuarantined_IsDeferred) {
  auto v = Fresh("v1");
  v.quarantined = true;
  auto req = MakeRequest({v});
  req.distance_to_pickup_m = {{"v1", 1.0}};
  const auto d = fleet::PlanVehicleAssignment(req);
  EXPECT_EQ(d.outcome, fleet::AssignmentOutcome::kDeferred);
}

TEST(VehicleAssignmentTest, AutoSelect_DisabledOnly_IsRejected_Permanent) {
  auto v = Fresh("v1");
  v.enabled = false;
  auto req = MakeRequest({v});
  req.distance_to_pickup_m = {{"v1", 1.0}};
  const auto d = fleet::PlanVehicleAssignment(req);
  // 禁用是永久状态（需运维改配置）-> 拒绝
  EXPECT_EQ(d.outcome, fleet::AssignmentOutcome::kRejected);
}

// ---- 能力与新鲜度辅助 ----

TEST(VehicleAssignmentTest, VehicleHasCapability_EmptyListMeansNoRestriction) {
  auto v = Fresh("v1", {});
  EXPECT_TRUE(fleet::VehicleHasCapability(v, "transport"));
}

TEST(VehicleAssignmentTest, VehicleHasCapability_NonEmptyListMustContain) {
  auto v = Fresh("v1", {"charging"});
  EXPECT_FALSE(fleet::VehicleHasCapability(v, "transport"));
  EXPECT_TRUE(fleet::VehicleHasCapability(v, "charging"));
}

TEST(VehicleAssignmentTest, VehicleStateIsFresh_NeverSeenIsNotFresh) {
  auto v = NeverSeen("v1");
  EXPECT_FALSE(fleet::VehicleStateIsFresh(v, 5000));
  auto f = Fresh("v2");
  EXPECT_TRUE(fleet::VehicleStateIsFresh(f, 5000));
}

TEST(VehicleAssignmentTest, BackendLabelRoundTrip) {
  fleet::VehicleExecutionBackend backend = fleet::VehicleExecutionBackend::kLocalSim;
  EXPECT_TRUE(fleet::ParseVehicleBackend("vda5050", &backend));
  EXPECT_EQ(backend, fleet::VehicleExecutionBackend::kVda5050);
  EXPECT_STREQ(fleet::VehicleBackendLabel(backend), "vda5050");
  EXPECT_FALSE(fleet::ParseVehicleBackend("weird", &backend));
}
