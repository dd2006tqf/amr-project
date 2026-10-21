// 人工强清 / 恢复决策（纯核心）的单测。
//
// 覆盖点：
//   * 强清准入：空 id / 确认 id 不匹配 / 缺 operator_id / 缺 reason / 不在清单 /
//     在线 / 已隔离 —— 全部拒绝且无副作用
//   * 锁归属：只认 mission->vehicle 绑定；未绑定的 owner 一律不动
//   * 恢复准入：必须已隔离 + 在线 + 操作员已核实位置

#include "amr_dispatcher_core/fleet/force_clear_link.hpp"

#include <gtest/gtest.h>

#include <string>
#include <unordered_map>

namespace fleet = amr_dispatcher_core::fleet;

namespace {

fleet::ForceClearLinkRequest MakeRequest(const std::string& vehicle_id) {
  fleet::ForceClearLinkRequest req;
  req.vehicle_id = vehicle_id;
  req.confirm_vehicle_id = vehicle_id;
  req.operator_id = "op_alice";
  req.reason = "vehicle physically removed from line";
  req.vehicle_in_managed_list = true;
  req.vehicle_online = false;
  return req;
}

fleet::RestoreLinkRequest MakeRestoreRequest(const std::string& vehicle_id) {
  fleet::RestoreLinkRequest req;
  req.vehicle_id = vehicle_id;
  req.confirm_vehicle_id = vehicle_id;
  req.operator_id = "op_alice";
  req.reason = "vehicle back on line, position confirmed";
  req.vehicle_in_managed_list = true;
  req.vehicle_quarantined = true;
  req.vehicle_online = true;
  req.position_verified = true;
  return req;
}

}  // namespace

TEST(ForceClearLinkTest, EmptyVehicleId_IsRejected) {
  auto req = MakeRequest("");
  const auto decision = fleet::PlanForceClearLink(req);
  EXPECT_FALSE(decision.allowed);
  EXPECT_FALSE(decision.reject_reason.empty());
  EXPECT_TRUE(decision.locks_by_mission.empty());
  EXPECT_TRUE(decision.missions_to_abort.empty());
}

TEST(ForceClearLinkTest, ConfirmVehicleIdMismatch_IsRejected) {
  auto req = MakeRequest("amr_01");
  req.confirm_vehicle_id = "amr_02";  // 打错车
  const auto decision = fleet::PlanForceClearLink(req);
  EXPECT_FALSE(decision.allowed);
  EXPECT_NE(decision.reject_reason.find("does not match"), std::string::npos);
}

TEST(ForceClearLinkTest, MissingOperatorId_IsRejected) {
  auto req = MakeRequest("amr_01");
  req.operator_id.clear();
  const auto decision = fleet::PlanForceClearLink(req);
  EXPECT_FALSE(decision.allowed);
  EXPECT_NE(decision.reject_reason.find("operator_id"), std::string::npos);
}

TEST(ForceClearLinkTest, MissingReason_IsRejected) {
  auto req = MakeRequest("amr_01");
  req.reason.clear();
  const auto decision = fleet::PlanForceClearLink(req);
  EXPECT_FALSE(decision.allowed);
  EXPECT_NE(decision.reject_reason.find("reason"), std::string::npos);
}

TEST(ForceClearLinkTest, VehicleNotInManagedList_IsRejected) {
  auto req = MakeRequest("amr_09");
  req.vehicle_in_managed_list = false;
  const auto decision = fleet::PlanForceClearLink(req);
  EXPECT_FALSE(decision.allowed);
  EXPECT_NE(decision.reject_reason.find("not in managed list"), std::string::npos);
}

TEST(ForceClearLinkTest, OnlineVehicle_IsRejected_ToAvoidKillingLiveLink) {
  auto req = MakeRequest("amr_01");
  req.vehicle_online = true;
  const auto decision = fleet::PlanForceClearLink(req);
  EXPECT_FALSE(decision.allowed);
  EXPECT_NE(decision.reject_reason.find("ONLINE"), std::string::npos);
  // 拒绝路径不得产生任何副作用
  EXPECT_TRUE(decision.locks_by_mission.empty());
  EXPECT_TRUE(decision.missions_to_abort.empty());
}

TEST(ForceClearLinkTest, AlreadyQuarantined_IsRejected) {
  auto req = MakeRequest("amr_01");
  req.vehicle_quarantined = true;
  const auto decision = fleet::PlanForceClearLink(req);
  EXPECT_FALSE(decision.allowed);
  EXPECT_NE(decision.reject_reason.find("already quarantined"), std::string::npos);
}

TEST(ForceClearLinkTest, OfflineVehicle_CollectsOwnLocksAndMissions) {
  auto req = MakeRequest("amr_01");
  req.mission_vehicle_bindings = {{"m_1", "amr_01"}, {"m_2", "amr_02"}};
  req.held_locks = {
      {"route_node:A", "m_1"},
      {"route_edge:A__B", "m_1"},
      {"route_node:C", "m_2"},
      {"route_node:D", "traffic_block:reason"},
  };

  const auto decision = fleet::PlanForceClearLink(req);
  ASSERT_TRUE(decision.allowed);
  ASSERT_EQ(decision.missions_to_abort.size(), 1U);
  EXPECT_EQ(decision.missions_to_abort[0], "m_1");
  ASSERT_EQ(decision.locks_by_mission.size(), 1U);
  const auto it = decision.locks_by_mission.find("m_1");
  ASSERT_NE(it, decision.locks_by_mission.end());
  EXPECT_EQ(it->second.size(), 2U);
  // 他车 mission 的锁与 traffic_block 一律不得归入本车
  EXPECT_EQ(decision.locks_by_mission.count("m_2"), 0U);
}

TEST(ForceClearLinkTest, UnboundOwnerLocks_AreNeverClaimed) {
  auto req = MakeRequest("amr_01");
  req.mission_vehicle_bindings = {{"m_1", "amr_01"}};
  req.held_locks = {
      {"route_node:A", "m_1"},
      {"route_node:Z", "some_unrelated_mission"},
      {"route_node:Y", "traffic_block:manual"},
  };

  const auto decision = fleet::PlanForceClearLink(req);
  ASSERT_TRUE(decision.allowed);
  const auto it = decision.locks_by_mission.find("m_1");
  ASSERT_NE(it, decision.locks_by_mission.end());
  ASSERT_EQ(it->second.size(), 1U);
  EXPECT_EQ(it->second[0], "route_node:A");
}

TEST(ForceClearLinkTest, NoBindings_AllowedButReleasesNothing) {
  auto req = MakeRequest("amr_01");
  req.held_locks = {{"route_node:A", "m_9"}};
  const auto decision = fleet::PlanForceClearLink(req);
  EXPECT_TRUE(decision.allowed);
  EXPECT_TRUE(decision.locks_by_mission.empty());
  EXPECT_TRUE(decision.missions_to_abort.empty());
}

// ---- 恢复（解除隔离） ----

TEST(RestoreLinkTest, ValidRestore_IsAllowed) {
  const auto decision = fleet::PlanRestoreLink(MakeRestoreRequest("amr_01"));
  EXPECT_TRUE(decision.allowed);
  EXPECT_TRUE(decision.reject_reason.empty());
}

TEST(RestoreLinkTest, NotQuarantined_IsRejected) {
  auto req = MakeRestoreRequest("amr_01");
  req.vehicle_quarantined = false;
  const auto decision = fleet::PlanRestoreLink(req);
  EXPECT_FALSE(decision.allowed);
  EXPECT_NE(decision.reject_reason.find("not quarantined"), std::string::npos);
}

TEST(RestoreLinkTest, StillOffline_IsRejected) {
  auto req = MakeRestoreRequest("amr_01");
  req.vehicle_online = false;
  const auto decision = fleet::PlanRestoreLink(req);
  EXPECT_FALSE(decision.allowed);
  EXPECT_NE(decision.reject_reason.find("offline"), std::string::npos);
}

TEST(RestoreLinkTest, PositionNotVerified_IsRejected) {
  auto req = MakeRestoreRequest("amr_01");
  req.position_verified = false;
  const auto decision = fleet::PlanRestoreLink(req);
  EXPECT_FALSE(decision.allowed);
  EXPECT_NE(decision.reject_reason.find("position_verified"), std::string::npos);
}

TEST(RestoreLinkTest, ConfirmIdMismatch_IsRejected) {
  auto req = MakeRestoreRequest("amr_01");
  req.confirm_vehicle_id = "amr_09";
  const auto decision = fleet::PlanRestoreLink(req);
  EXPECT_FALSE(decision.allowed);
  EXPECT_NE(decision.reject_reason.find("does not match"), std::string::npos);
}
