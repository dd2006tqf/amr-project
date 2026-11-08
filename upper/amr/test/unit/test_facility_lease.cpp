#include "amr_dispatcher_core/facility/facility_reservation.hpp"

#include <gtest/gtest.h>

using amr_dispatcher_core::facility::FacilityReservationMap;
using amr_dispatcher_core::facility::FacilityResource;
using amr_dispatcher_core::facility::MissionResourceMap;
using amr_dispatcher_core::facility::PruneExpiredFacilityReservations;
using amr_dispatcher_core::facility::ReserveFacilityResourceWithLease;

TEST(FacilityReservationLeaseTest, LeaseExpiresAndGetsPruned) {
  FacilityReservationMap reservations;
  MissionResourceMap resource_by_mission;

  FacilityResource door{"door_1", "door", "station_A", true, true};

  // 申请 2000ms 租约，当前时间 1000
  auto res = ReserveFacilityResourceWithLease(
      reservations, door, "amr_01", "m_1", false, 2000, 1000);
  ASSERT_TRUE(res.success);
  EXPECT_EQ(reservations["door_1"].lease_expires_at_ms, 3000);

  resource_by_mission["m_1"] = "door_1";

  // 在 2500ms 时检查：尚未过期
  auto pruned_early = PruneExpiredFacilityReservations(reservations, resource_by_mission, 2500);
  EXPECT_TRUE(pruned_early.empty());
  EXPECT_EQ(reservations.size(), 1u);

  // 在 3001ms 时检查：已经过期，自动回收
  auto pruned_late = PruneExpiredFacilityReservations(reservations, resource_by_mission, 3001);
  ASSERT_EQ(pruned_late.size(), 1u);
  EXPECT_EQ(pruned_late[0], "door_1");
  EXPECT_TRUE(reservations.empty());
  EXPECT_TRUE(resource_by_mission.empty());
}

TEST(FacilityReservationLeaseTest, ZeroLeaseNeverExpires) {
  FacilityReservationMap reservations;
  MissionResourceMap resource_by_mission;
  FacilityResource lift{"lift_1", "elevator", "station_B", true, true};

  // 永不过期租约
  ReserveFacilityResourceWithLease(
      reservations, lift, "amr_01", "m_1", false, 0, 1000);
  EXPECT_EQ(reservations["lift_1"].lease_expires_at_ms, 0);

  auto pruned = PruneExpiredFacilityReservations(reservations, resource_by_mission, 9999999);
  EXPECT_TRUE(pruned.empty());
  EXPECT_EQ(reservations.size(), 1u);
}
