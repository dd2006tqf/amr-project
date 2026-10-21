// A1：选车策略纯函数 PlanVehicleAssignment 的单测。
//
// 覆盖点（规则顺序固定）：
//   1. 显式指定车辆 -> 命中则 assigned/strategy="explicit"；否则拒绝（不静默改派）
//   2. 未指定 -> 过滤 online && !busy && enabled && 能力匹配 && 电量达标
//   3. 候选按到 pickup 的距离升序取最近；同距按输入序；无距离信息排在有距离之后；
//      全部无距离 -> 首个可用（strategy="first_available"）
//   4. 无候选 -> 带原因的拒绝
//   5. 空车辆表 -> 拒绝
//   6. 空 capabilities 视为"无能力限制"（单车最小配置仍可选中）

#include "amr_dispatcher_core/fleet/vehicle_assignment.hpp"

#include <gtest/gtest.h>

#include <string>
#include <vector>

namespace fleet = amr_dispatcher_core::fleet;

namespace {

fleet::VehicleRuntimeState MakeVehicle(const std::string& id,
                                        std::vector<std::string> capabilities = {"transport"}) {
  fleet::VehicleRuntimeState v;
  v.vehicle_id = id;
  v.online = true;
  v.busy = false;
  v.enabled = true;
  v.battery_percent = 100.0;
  v.capabilities = std::move(capabilities);
  return v;
}

fleet::VehicleAssignmentRequest MakeRequest(std::vector<fleet::VehicleRuntimeState> vehicles) {
  fleet::VehicleAssignmentRequest req;
  req.vehicles = std::move(vehicles);
  req.pickup_station_id = "A";
  req.dropoff_station_id = "B";
  req.required_capability = "transport";
  req.min_battery_percent = 0.0;
  return req;
}

}  // namespace

// ---- 规则 1：显式指定 ----

TEST(VehicleAssignmentTest, ExplicitVehicle_Valid_Assigns) {
  auto req = MakeRequest({MakeVehicle("v1"), MakeVehicle("v2")});
  req.requested_vehicle_id = "v2";
  const auto d = fleet::PlanVehicleAssignment(req);
  EXPECT_TRUE(d.assigned);
  EXPECT_EQ(d.vehicle_id, "v2");
  EXPECT_EQ(d.strategy, "explicit");
  EXPECT_TRUE(d.reject_reason.empty());
}

TEST(VehicleAssignmentTest, ExplicitVehicle_Unknown_IsRejected_NotReassigned) {
  auto req = MakeRequest({MakeVehicle("v1")});
  req.requested_vehicle_id = "ghost";
  const auto d = fleet::PlanVehicleAssignment(req);
  EXPECT_FALSE(d.assigned);
  EXPECT_TRUE(d.vehicle_id.empty());
  EXPECT_EQ(d.strategy, "rejected");
  EXPECT_NE(d.reject_reason.find("ghost"), std::string::npos);
  // 关键：显式指定失败时不得静默改派给清单内的车
  EXPECT_EQ(d.reject_reason.find("v1"), std::string::npos);
}

TEST(VehicleAssignmentTest, ExplicitVehicle_Offline_IsRejected) {
  auto v = MakeVehicle("v1");
  v.online = false;
  auto req = MakeRequest({v});
  req.requested_vehicle_id = "v1";
  const auto d = fleet::PlanVehicleAssignment(req);
  EXPECT_FALSE(d.assigned);
  EXPECT_EQ(d.strategy, "rejected");
  EXPECT_NE(d.reject_reason.find("offline"), std::string::npos);
}

TEST(VehicleAssignmentTest, ExplicitVehicle_Busy_IsRejected) {
  auto v = MakeVehicle("v1");
  v.busy = true;
  auto req = MakeRequest({v});
  req.requested_vehicle_id = "v1";
  const auto d = fleet::PlanVehicleAssignment(req);
  EXPECT_FALSE(d.assigned);
  EXPECT_NE(d.reject_reason.find("busy"), std::string::npos);
}

TEST(VehicleAssignmentTest, ExplicitVehicle_Disabled_IsRejected) {
  auto v = MakeVehicle("v1");
  v.enabled = false;
  auto req = MakeRequest({v});
  req.requested_vehicle_id = "v1";
  const auto d = fleet::PlanVehicleAssignment(req);
  EXPECT_FALSE(d.assigned);
  EXPECT_NE(d.reject_reason.find("disabled"), std::string::npos);
}

TEST(VehicleAssignmentTest, ExplicitVehicle_MissingCapability_IsRejected) {
  auto req = MakeRequest({MakeVehicle("v1", {"charging"})});
  req.requested_vehicle_id = "v1";
  req.required_capability = "transport";
  const auto d = fleet::PlanVehicleAssignment(req);
  EXPECT_FALSE(d.assigned);
  EXPECT_NE(d.reject_reason.find("transport"), std::string::npos);
}

TEST(VehicleAssignmentTest, ExplicitVehicle_WinsOverNearerAutoCandidate) {
  auto req = MakeRequest({MakeVehicle("v1"), MakeVehicle("v2")});
  req.requested_vehicle_id = "v2";
  req.distance_to_pickup_m = {{"v1", 1.0}, {"v2", 100.0}};
  const auto d = fleet::PlanVehicleAssignment(req);
  EXPECT_TRUE(d.assigned);
  EXPECT_EQ(d.vehicle_id, "v2");
  EXPECT_EQ(d.strategy, "explicit");
}

// ---- 规则 2/3：自动选车 ----

TEST(VehicleAssignmentTest, AutoSelect_NearestWins) {
  auto req = MakeRequest({MakeVehicle("v1"), MakeVehicle("v2"), MakeVehicle("v3")});
  req.distance_to_pickup_m = {{"v1", 30.0}, {"v2", 5.0}, {"v3", 12.0}};
  const auto d = fleet::PlanVehicleAssignment(req);
  EXPECT_TRUE(d.assigned);
  EXPECT_EQ(d.vehicle_id, "v2");
  EXPECT_EQ(d.strategy, "nearest");
}

TEST(VehicleAssignmentTest, AutoSelect_TieBreaksByInputOrder) {
  auto req = MakeRequest({MakeVehicle("v1"), MakeVehicle("v2")});
  req.distance_to_pickup_m = {{"v1", 7.0}, {"v2", 7.0}};
  const auto d = fleet::PlanVehicleAssignment(req);
  EXPECT_TRUE(d.assigned);
  EXPECT_EQ(d.vehicle_id, "v1");
  EXPECT_EQ(d.strategy, "nearest");
}

TEST(VehicleAssignmentTest, AutoSelect_CandidateWithoutDistanceRanksAfterThoseWithDistance) {
  auto req = MakeRequest({MakeVehicle("v1"), MakeVehicle("v2")});
  req.distance_to_pickup_m = {{"v2", 99.0}};
  const auto d = fleet::PlanVehicleAssignment(req);
  EXPECT_TRUE(d.assigned);
  EXPECT_EQ(d.vehicle_id, "v2");
  EXPECT_EQ(d.strategy, "nearest");
}

TEST(VehicleAssignmentTest, AutoSelect_NoDistanceInfoAnywhere_FirstEligibleInInputOrder) {
  auto req = MakeRequest({MakeVehicle("v1"), MakeVehicle("v2")});
  const auto d = fleet::PlanVehicleAssignment(req);
  EXPECT_TRUE(d.assigned);
  EXPECT_EQ(d.vehicle_id, "v1");
  EXPECT_EQ(d.strategy, "first_available");
}

TEST(VehicleAssignmentTest, AutoSelect_CapabilityFilterExcludes) {
  auto req = MakeRequest({MakeVehicle("v1", {"charging"}), MakeVehicle("v2", {"transport"})});
  const auto d = fleet::PlanVehicleAssignment(req);
  EXPECT_TRUE(d.assigned);
  EXPECT_EQ(d.vehicle_id, "v2");
}

TEST(VehicleAssignmentTest, AutoSelect_BatteryThresholdExcludes) {
  auto low = MakeVehicle("v1");
  low.battery_percent = 10.0;
  auto ok = MakeVehicle("v2");
  ok.battery_percent = 60.0;
  auto req = MakeRequest({low, ok});
  req.min_battery_percent = 50.0;
  const auto d = fleet::PlanVehicleAssignment(req);
  EXPECT_TRUE(d.assigned);
  EXPECT_EQ(d.vehicle_id, "v2");
}

TEST(VehicleAssignmentTest, AutoSelect_BusyVehicleExcluded) {
  auto busy = MakeVehicle("v1");
  busy.busy = true;
  auto req = MakeRequest({busy, MakeVehicle("v2")});
  const auto d = fleet::PlanVehicleAssignment(req);
  EXPECT_TRUE(d.assigned);
  EXPECT_EQ(d.vehicle_id, "v2");
}

TEST(VehicleAssignmentTest, AutoSelect_OfflineAndDisabledExcluded) {
  auto off = MakeVehicle("v1");
  off.online = false;
  auto dis = MakeVehicle("v2");
  dis.enabled = false;
  auto req = MakeRequest({off, dis, MakeVehicle("v3")});
  const auto d = fleet::PlanVehicleAssignment(req);
  EXPECT_TRUE(d.assigned);
  EXPECT_EQ(d.vehicle_id, "v3");
}

TEST(VehicleAssignmentTest, AutoSelect_NoEligible_RejectedWithReason) {
  auto off = MakeVehicle("v1");
  off.online = false;
  auto req = MakeRequest({off});
  const auto d = fleet::PlanVehicleAssignment(req);
  EXPECT_FALSE(d.assigned);
  EXPECT_EQ(d.strategy, "rejected");
  EXPECT_FALSE(d.reject_reason.empty());
  EXPECT_NE(d.reject_reason.find("transport"), std::string::npos);
}

TEST(VehicleAssignmentTest, AutoSelect_AllBusy_RejectedWithBusyReason) {
  auto a = MakeVehicle("v1");
  a.busy = true;
  auto b = MakeVehicle("v2");
  b.busy = true;
  auto req = MakeRequest({a, b});
  const auto d = fleet::PlanVehicleAssignment(req);
  EXPECT_FALSE(d.assigned);
  EXPECT_NE(d.reject_reason.find("busy"), std::string::npos);
}

TEST(VehicleAssignmentTest, AutoSelect_AllLowBattery_RejectedWithBatteryReason) {
  auto a = MakeVehicle("v1");
  a.battery_percent = 5.0;
  auto req = MakeRequest({a});
  req.min_battery_percent = 30.0;
  const auto d = fleet::PlanVehicleAssignment(req);
  EXPECT_FALSE(d.assigned);
  EXPECT_NE(d.reject_reason.find("battery"), std::string::npos);
}

TEST(VehicleAssignmentTest, EmptyVehicleList_Rejected) {
  auto req = MakeRequest({});
  const auto d = fleet::PlanVehicleAssignment(req);
  EXPECT_FALSE(d.assigned);
  EXPECT_EQ(d.strategy, "rejected");
  EXPECT_NE(d.reject_reason.find("no managed vehicles"), std::string::npos);
}

// ---- 能力判定辅助函数 ----

TEST(VehicleAssignmentTest, VehicleHasCapability_EmptyListMeansNoRestriction) {
  auto v = MakeVehicle("v1", {});
  EXPECT_TRUE(fleet::VehicleHasCapability(v, "transport"));
  EXPECT_TRUE(fleet::VehicleHasCapability(v, "charging"));
}

TEST(VehicleAssignmentTest, VehicleHasCapability_NonEmptyListMustContain) {
  auto v = MakeVehicle("v1", {"transport", "charging"});
  EXPECT_TRUE(fleet::VehicleHasCapability(v, "transport"));
  EXPECT_TRUE(fleet::VehicleHasCapability(v, "charging"));
  EXPECT_FALSE(fleet::VehicleHasCapability(v, "warehouse"));
}

TEST(VehicleAssignmentTest, AutoSelect_EmptyCapabilitiesVehicleStillEligible) {
  auto req = MakeRequest({MakeVehicle("v1", {}), MakeVehicle("v2", {"charging"})});
  const auto d = fleet::PlanVehicleAssignment(req);
  EXPECT_TRUE(d.assigned);
  EXPECT_EQ(d.vehicle_id, "v1");
}
