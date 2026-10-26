#include "amr_dispatcher_core/workflow/auto_charging_planning.hpp"

#include <gtest/gtest.h>

using amr_dispatcher_core::workflow::AutoChargingDecision;
using amr_dispatcher_core::workflow::AutoChargingRequest;
using amr_dispatcher_core::workflow::PlanAutoCharging;

namespace {

// 一辆"可以插单"的车：在线、空闲、未隔离、电量低于机会阈值、充电站已知。
AutoChargingRequest MakeEligible(double battery_percent = 20.0) {
  AutoChargingRequest request;
  request.enabled = true;
  request.vehicle_id = "amr_01";
  request.battery_percent = battery_percent;
  request.charger_station_id = "station_charge_1";
  request.charger_station_known = true;
  return request;
}

}  // namespace

TEST(AutoChargingPlanningTest, DisabledByDefault) {
  AutoChargingRequest request;  // enabled=false
  EXPECT_EQ(PlanAutoCharging(request).kind, AutoChargingDecision::Kind::kDisabled);
}

TEST(AutoChargingPlanningTest, HealthyBatteryDoesNotQueue) {
  EXPECT_EQ(PlanAutoCharging(MakeEligible(80.0)).kind,
            AutoChargingDecision::Kind::kBatteryOk);
}

TEST(AutoChargingPlanningTest, LowBatteryQueuesOpportunityCharge) {
  const auto decision = PlanAutoCharging(MakeEligible(25.0));
  ASSERT_EQ(decision.kind, AutoChargingDecision::Kind::kQueueCharge);
  EXPECT_FALSE(decision.critical);
  EXPECT_EQ(decision.priority, 0);  // opportunity_priority
  EXPECT_EQ(decision.charger_station_id, "station_charge_1");
}

TEST(AutoChargingPlanningTest, CriticalBatteryUsesHigherPriority) {
  const auto decision = PlanAutoCharging(MakeEligible(10.0));
  ASSERT_EQ(decision.kind, AutoChargingDecision::Kind::kQueueCharge);
  EXPECT_TRUE(decision.critical);
  EXPECT_EQ(decision.priority, 5);  // critical_priority
}

// 阈值边界：等于机会阈值即插单（电量只会继续降），等于临界阈值算临界。
TEST(AutoChargingPlanningTest, ThresholdBoundariesAreInclusive) {
  EXPECT_EQ(PlanAutoCharging(MakeEligible(30.0)).kind,
            AutoChargingDecision::Kind::kQueueCharge);
  EXPECT_TRUE(PlanAutoCharging(MakeEligible(15.0)).critical);
  EXPECT_FALSE(PlanAutoCharging(MakeEligible(15.01)).critical);
}

// 去重是这套逻辑最容易漏的一条：没有它，每 tick 都会再插一条充电单。
TEST(AutoChargingPlanningTest, ExistingChargeMissionIsNotDuplicated) {
  auto request = MakeEligible(5.0);
  request.has_pending_charge_mission = true;
  EXPECT_EQ(PlanAutoCharging(request).kind,
            AutoChargingDecision::Kind::kAlreadyQueued);
}

// 去重先于可用性判定：车忙但已经有充电单时，报"已有充电单"而不是"车忙"，
// 否则运维会以为系统没在管这辆车。
TEST(AutoChargingPlanningTest, DedupBeatsAvailabilityReason) {
  auto request = MakeEligible(5.0);
  request.has_pending_charge_mission = true;
  request.busy = true;
  EXPECT_EQ(PlanAutoCharging(request).kind,
            AutoChargingDecision::Kind::kAlreadyQueued);
}

TEST(AutoChargingPlanningTest, BusyVehicleIsSkipped) {
  auto request = MakeEligible();
  request.busy = true;
  const auto decision = PlanAutoCharging(request);
  EXPECT_EQ(decision.kind, AutoChargingDecision::Kind::kVehicleUnavailable);
  EXPECT_NE(decision.reason.find("busy"), std::string::npos);
}

TEST(AutoChargingPlanningTest, OfflineAndQuarantinedVehiclesAreSkipped) {
  auto offline = MakeEligible();
  offline.online = false;
  EXPECT_EQ(PlanAutoCharging(offline).kind,
            AutoChargingDecision::Kind::kVehicleUnavailable);

  auto quarantined = MakeEligible();
  quarantined.quarantined = true;
  const auto decision = PlanAutoCharging(quarantined);
  EXPECT_EQ(decision.kind, AutoChargingDecision::Kind::kVehicleUnavailable);
  EXPECT_NE(decision.reason.find("quarantined"), std::string::npos);
}

// 电量未知（例如 VDA 5050 车还没上报过 state）：不插单，也不当成 0%。
TEST(AutoChargingPlanningTest, UnknownBatteryIsNotTreatedAsEmpty) {
  auto request = MakeEligible(0.0);
  request.battery_known = false;
  EXPECT_EQ(PlanAutoCharging(request).kind,
            AutoChargingDecision::Kind::kBatteryUnknown);
}

TEST(AutoChargingPlanningTest, MissingChargerStationIsNotGuessed) {
  auto request = MakeEligible();
  request.charger_station_known = false;
  EXPECT_EQ(PlanAutoCharging(request).kind,
            AutoChargingDecision::Kind::kChargerUnknown);

  auto empty_id = MakeEligible();
  empty_id.charger_station_id.clear();
  EXPECT_EQ(PlanAutoCharging(empty_id).kind,
            AutoChargingDecision::Kind::kChargerUnknown);
}

TEST(AutoChargingPlanningTest, CustomThresholdsAndPriorities) {
  auto request = MakeEligible(45.0);
  request.opportunity_threshold_percent = 50.0;
  request.critical_threshold_percent = 20.0;
  request.opportunity_priority = 3;
  request.critical_priority = 9;
  const auto decision = PlanAutoCharging(request);
  ASSERT_EQ(decision.kind, AutoChargingDecision::Kind::kQueueCharge);
  EXPECT_FALSE(decision.critical);
  EXPECT_EQ(decision.priority, 3);
}
