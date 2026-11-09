#include "amr_dispatcher_core/fleet/force_clear_link.hpp"

#include <gtest/gtest.h>

#include <string>
#include <unordered_map>

namespace fleet = amr_dispatcher_core::fleet;

namespace {

fleet::ForceClearLinkRequest MakeRequest(const std::string& vehicle_id) {
  fleet::ForceClearLinkRequest req;
  req.vehicle_id = vehicle_id;
  req.vehicle_in_managed_list = true;
  req.vehicle_online = false;
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
