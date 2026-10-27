#include "amr_dispatcher_core/facility/facility_interlocking_planning.hpp"

#include <gtest/gtest.h>

using amr_dispatcher_core::facility::FacilityApproachDecision;
using amr_dispatcher_core::facility::FacilityApproachRequest;
using amr_dispatcher_core::facility::FacilityDepartureRequest;
using amr_dispatcher_core::facility::FacilityResource;
using amr_dispatcher_core::facility::FacilityState;
using amr_dispatcher_core::facility::FacilityStateReport;
using amr_dispatcher_core::facility::FacilityType;
using amr_dispatcher_core::facility::PlanFacilityApproach;
using amr_dispatcher_core::facility::PlanFacilityDeparture;
using amr_dispatcher_core::facility::ResourceReservation;

TEST(FacilityInterlockingPlanningTest, UnheldFacilityRequestsAcquireAndOpen) {
  FacilityResource door{"door_A", "door", "station_A", true, true};
  FacilityApproachRequest req;
  req.mission_id = "m1";
  req.vehicle_id = "amr_01";
  req.current_station_id = "station_A";
  req.resource = &door;
  req.current_reservation = nullptr; // 尚未持锁

  auto dec = PlanFacilityApproach(req);
  EXPECT_EQ(dec.action, FacilityApproachDecision::Action::kAcquireAndOpen);
  EXPECT_EQ(dec.command, "open");
}

TEST(FacilityInterlockingPlanningTest, HeldAndFullyOpenAllowsProceed) {
  FacilityResource door{"door_A", "door", "station_A", true, true};
  ResourceReservation res{"amr_01", "m1", "reserved", true, 0};
  FacilityStateReport state;
  state.facility_id = "door_A";
  state.type = FacilityType::kDoor;
  state.state = FacilityState::kReady;
  state.door_status = "open";

  FacilityApproachRequest req;
  req.mission_id = "m1";
  req.vehicle_id = "amr_01";
  req.current_station_id = "station_A";
  req.resource = &door;
  req.current_reservation = &res;
  req.facility_state = state;

  auto dec = PlanFacilityApproach(req);
  EXPECT_EQ(dec.action, FacilityApproachDecision::Action::kProceed);
}

TEST(FacilityInterlockingPlanningTest, DoorMovingWaitsInPlace) {
  FacilityResource door{"door_A", "door", "station_A", true, true};
  ResourceReservation res{"amr_01", "m1", "reserved", true, 0};
  FacilityStateReport state;
  state.door_status = "closed";

  FacilityApproachRequest req;
  req.mission_id = "m1";
  req.vehicle_id = "amr_01";
  req.current_station_id = "station_A";
  req.resource = &door;
  req.current_reservation = &res;
  req.facility_state = state;
  req.wait_duration_ms = 2000;
  req.timeout_threshold_ms = 10000;

  auto dec = PlanFacilityApproach(req);
  EXPECT_EQ(dec.action, FacilityApproachDecision::Action::kWaitInPlace);
}

TEST(FacilityInterlockingPlanningTest, DoorTimeoutTriggersDegradation) {
  FacilityResource door{"door_A", "door", "station_A", true, true};
  ResourceReservation res{"amr_01", "m1", "reserved", true, 0};
  FacilityStateReport state;
  state.door_status = "closed";

  FacilityApproachRequest req;
  req.mission_id = "m1";
  req.vehicle_id = "amr_01";
  req.current_station_id = "station_A";
  req.resource = &door;
  req.current_reservation = &res;
  req.facility_state = state;
  req.wait_duration_ms = 12000; // 超出 10s 阈值
  req.timeout_threshold_ms = 10000;

  auto dec = PlanFacilityApproach(req);
  EXPECT_EQ(dec.action, FacilityApproachDecision::Action::kTimeoutDegrade);
}

TEST(FacilityInterlockingPlanningTest, DepartureTriggersCloseAndRelease) {
  FacilityResource door{"door_A", "door", "station_A", true, true};
  ResourceReservation res{"amr_01", "m1", "reserved", true, 0};

  FacilityDepartureRequest req;
  req.mission_id = "m1";
  req.vehicle_id = "amr_01";
  req.departed_station_id = "station_A";
  req.resource = &door;
  req.current_reservation = &res;

  auto dec = PlanFacilityDeparture(req);
  EXPECT_TRUE(dec.should_close);
  EXPECT_TRUE(dec.should_release_lock);
  EXPECT_EQ(dec.command, "close");
}
