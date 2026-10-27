#include "amr_dispatcher_core/facility/facility_interlocking_planning.hpp"

namespace amr_dispatcher_core::facility {

FacilityApproachDecision PlanFacilityApproach(const FacilityApproachRequest& request) {
  FacilityApproachDecision decision;

  if (request.resource == nullptr || !request.resource->enabled) {
    decision.action = FacilityApproachDecision::Action::kProceed;
    decision.message = "no facility linked or facility disabled";
    return decision;
  }

  // 1. 检查锁持有情况
  const bool is_held_by_us =
      request.current_reservation != nullptr &&
      (request.current_reservation->holder_id == request.vehicle_id ||
       request.current_reservation->holder_id == request.mission_id);

  if (!is_held_by_us) {
    // 尚未持有锁：发起申请与开门指令
    decision.action = FacilityApproachDecision::Action::kAcquireAndOpen;
    decision.command = (request.resource->type == "door") ? "open" : "call";
    decision.message = "acquire facility lock and trigger open/call";
    return decision;
  }

  // 2. 已持锁：检查设施物理状态
  if (request.resource->type == "door") {
    if (request.facility_state.door_status == "open") {
      decision.action = FacilityApproachDecision::Action::kProceed;
      decision.message = "door is fully open; proceed";
      return decision;
    }
    // 门仍在开启中或关闭中
    if (request.wait_duration_ms > request.timeout_threshold_ms) {
      decision.action = FacilityApproachDecision::Action::kTimeoutDegrade;
      decision.message = "door failed to open within timeout (" +
                         std::to_string(request.wait_duration_ms) + " ms)";
      return decision;
    }
    decision.action = FacilityApproachDecision::Action::kWaitInPlace;
    decision.message = "waiting for door to open; current status=" + request.facility_state.door_status;
    return decision;
  }

  // 默认放行
  decision.action = FacilityApproachDecision::Action::kProceed;
  return decision;
}

FacilityDepartureDecision PlanFacilityDeparture(const FacilityDepartureRequest& request) {
  FacilityDepartureDecision decision;

  if (request.resource == nullptr || !request.resource->enabled) {
    return decision;
  }

  const bool is_held_by_us =
      request.current_reservation != nullptr &&
      (request.current_reservation->holder_id == request.vehicle_id ||
       request.current_reservation->holder_id == request.mission_id);

  if (is_held_by_us) {
    decision.should_close = (request.resource->type == "door");
    decision.should_release_lock = true;
    decision.command = "close";
    decision.message = "vehicle departed; close door and release facility lock";
  }

  return decision;
}

}  // namespace amr_dispatcher_core::facility
