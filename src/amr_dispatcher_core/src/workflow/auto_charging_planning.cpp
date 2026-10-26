#include "amr_dispatcher_core/workflow/auto_charging_planning.hpp"

#include <string>

namespace amr_dispatcher_core::workflow {

AutoChargingDecision PlanAutoCharging(const AutoChargingRequest& request) {
  AutoChargingDecision decision;
  decision.charger_station_id = request.charger_station_id;

  if (!request.enabled) {
    decision.kind = AutoChargingDecision::Kind::kDisabled;
    decision.reason = "auto charging disabled";
    return decision;
  }
  if (!request.battery_known) {
    // 没有电量就不猜：宁可让车继续跑任务，也不凭一次缺失的报文把它叫回充电站
    decision.kind = AutoChargingDecision::Kind::kBatteryUnknown;
    decision.reason = "battery level unknown for vehicle '" + request.vehicle_id + "'";
    return decision;
  }
  if (request.battery_percent > request.opportunity_threshold_percent) {
    decision.kind = AutoChargingDecision::Kind::kBatteryOk;
    decision.reason = "battery above opportunity threshold";
    return decision;
  }
  if (request.has_pending_charge_mission) {
    // 去重：没有这一条，每个 tick 都会给同一台车再插一条充电单
    decision.kind = AutoChargingDecision::Kind::kAlreadyQueued;
    decision.reason = "vehicle '" + request.vehicle_id + "' already has a charge mission";
    return decision;
  }
  if (request.quarantined) {
    decision.kind = AutoChargingDecision::Kind::kVehicleUnavailable;
    decision.reason = "vehicle '" + request.vehicle_id + "' is quarantined";
    return decision;
  }
  if (!request.online) {
    decision.kind = AutoChargingDecision::Kind::kVehicleUnavailable;
    decision.reason = "vehicle '" + request.vehicle_id + "' is offline";
    return decision;
  }
  if (request.busy) {
    decision.kind = AutoChargingDecision::Kind::kVehicleUnavailable;
    decision.reason = "vehicle '" + request.vehicle_id + "' is busy";
    return decision;
  }
  if (!request.charger_station_known || request.charger_station_id.empty()) {
    decision.kind = AutoChargingDecision::Kind::kChargerUnknown;
    decision.reason = "charger station not resolved";
    return decision;
  }

  decision.kind = AutoChargingDecision::Kind::kQueueCharge;
  decision.critical = request.battery_percent <= request.critical_threshold_percent;
  decision.priority = decision.critical ? request.critical_priority : request.opportunity_priority;
  decision.reason = decision.critical ? "critical battery" : "opportunity charging";
  return decision;
}

}  // namespace amr_dispatcher_core::workflow
