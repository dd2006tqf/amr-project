#include "amr_dispatcher_core/facility/facility_adapter_interface.hpp"

#include <gtest/gtest.h>

using amr_dispatcher_core::facility::FacilityState;
using amr_dispatcher_core::facility::FacilityType;
using amr_dispatcher_core::facility::MockFacilityAdapter;

TEST(FacilityAdapterTest, DoorOperationsAndStateTransitions) {
  MockFacilityAdapter door("door_zone_a", FacilityType::kDoor);
  EXPECT_EQ(door.GetFacilityId(), "door_zone_a");
  EXPECT_EQ(door.GetFacilityType(), FacilityType::kDoor);

  auto initial = door.QueryState();
  EXPECT_EQ(initial.state, FacilityState::kReady);
  EXPECT_EQ(initial.door_status, "closed");

  auto open_res = door.ExecuteAction("open");
  EXPECT_TRUE(open_res.success);
  EXPECT_EQ(door.QueryState().door_status, "open");

  auto close_res = door.ExecuteAction("close");
  EXPECT_TRUE(close_res.success);
  EXPECT_EQ(door.QueryState().door_status, "closed");
}

TEST(FacilityAdapterTest, ElevatorFloorCallAndDoorControl) {
  MockFacilityAdapter elevator("lift_north", FacilityType::kElevator);
  auto call_res = elevator.ExecuteAction("call", "3");
  EXPECT_TRUE(call_res.success);
  EXPECT_EQ(elevator.QueryState().current_floor, "3");

  EXPECT_TRUE(elevator.ExecuteAction("open_door").success);
  EXPECT_EQ(elevator.QueryState().door_status, "open");

  EXPECT_TRUE(elevator.ExecuteAction("close_door").success);
  EXPECT_EQ(elevator.QueryState().door_status, "closed");
}

TEST(FacilityAdapterTest, FaultOrOfflineRejectsCommands) {
  MockFacilityAdapter door("door_faulty", FacilityType::kDoor);
  door.SetState(FacilityState::kFault);

  auto res = door.ExecuteAction("open");
  EXPECT_FALSE(res.success);
  EXPECT_EQ(res.error_code, 1);
}

TEST(FacilityAdapterTest, LockAcquireAndReleaseExclusivity) {
  MockFacilityAdapter door("door_secure", FacilityType::kDoor);
  EXPECT_TRUE(door.AcquireLock("amr_01"));
  EXPECT_TRUE(door.QueryState().locked);
  EXPECT_EQ(door.QueryState().locked_by, "amr_01");

  // 另一台车无法抢占
  EXPECT_FALSE(door.AcquireLock("amr_02"));

  // 非持有者无法释放
  EXPECT_FALSE(door.ReleaseLock("amr_02"));

  // 持有者成功释放
  EXPECT_TRUE(door.ReleaseLock("amr_01"));
  EXPECT_FALSE(door.QueryState().locked);

  // 释放后其他车可以获取
  EXPECT_TRUE(door.AcquireLock("amr_02"));
}
