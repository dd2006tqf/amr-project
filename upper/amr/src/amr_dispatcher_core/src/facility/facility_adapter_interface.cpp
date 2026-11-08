#include "amr_dispatcher_core/facility/facility_adapter_interface.hpp"

namespace amr_dispatcher_core::facility {

MockFacilityAdapter::MockFacilityAdapter(std::string facility_id, FacilityType type)
    : facility_id_(std::move(facility_id)), type_(type) {}

FacilityStateReport MockFacilityAdapter::QueryState() {
  FacilityStateReport report;
  report.facility_id = facility_id_;
  report.type = type_;
  report.state = state_;
  report.current_floor = current_floor_;
  report.door_status = door_status_;
  report.locked = locked_;
  report.locked_by = locked_by_;
  return report;
}

FacilityCommandResult MockFacilityAdapter::ExecuteAction(const std::string& action,
                                                        const std::string& params) {
  FacilityCommandResult res;
  res.facility_id = facility_id_;
  res.action = action;

  if (state_ == FacilityState::kFault || state_ == FacilityState::kOffline) {
    res.success = false;
    res.message = "facility is not operational (fault or offline)";
    res.error_code = 1;
    return res;
  }

  if (type_ == FacilityType::kDoor) {
    if (action == "open") {
      door_status_ = "open";
      res.success = true;
      res.message = "door opened successfully";
      return res;
    }
    if (action == "close") {
      door_status_ = "closed";
      res.success = true;
      res.message = "door closed successfully";
      return res;
    }
  } else if (type_ == FacilityType::kElevator) {
    if (action == "call" || action == "send_to_floor") {
      if (!params.empty()) {
        current_floor_ = params;
      }
      res.success = true;
      res.message = "elevator moved to floor " + current_floor_;
      return res;
    }
    if (action == "open_door") {
      door_status_ = "open";
      res.success = true;
      res.message = "elevator door opened";
      return res;
    }
    if (action == "close_door") {
      door_status_ = "closed";
      res.success = true;
      res.message = "elevator door closed";
      return res;
    }
  }

  res.success = false;
  res.message = "unsupported action: " + action;
  res.error_code = 2;
  return res;
}

bool MockFacilityAdapter::AcquireLock(const std::string& holder_id) {
  if (locked_ && locked_by_ != holder_id) {
    return false;
  }
  locked_ = true;
  locked_by_ = holder_id;
  return true;
}

bool MockFacilityAdapter::ReleaseLock(const std::string& holder_id) {
  if (!locked_ || locked_by_ != holder_id) {
    return false;
  }
  locked_ = false;
  locked_by_.clear();
  return true;
}

}  // namespace amr_dispatcher_core::facility
