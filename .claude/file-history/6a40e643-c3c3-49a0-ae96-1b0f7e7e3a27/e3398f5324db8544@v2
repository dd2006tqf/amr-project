#include "amr_dispatcher_core/facility/modbus_facility_adapter.hpp"

namespace amr_dispatcher_core::facility {

ModbusFacilityAdapter::ModbusFacilityAdapter(ModbusFacilityConfig config)
    : config_(std::move(config)),
      client_(std::make_shared<ModbusTcpClient>(
          config_.ip, config_.port, config_.unit_id, config_.timeout_ms)) {}

ModbusFacilityAdapter::ModbusFacilityAdapter(
    ModbusFacilityConfig config, std::shared_ptr<ModbusTcpClient> client)
    : config_(std::move(config)), client_(std::move(client)) {}

FacilityStateReport ModbusFacilityAdapter::QueryState() {
  FacilityStateReport report;
  report.facility_id = config_.facility_id;
  report.type = config_.type;
  report.locked = locked_;
  report.locked_by = locked_by_;

  if (!client_) {
    report.state = FacilityState::kOffline;
    return report;
  }

  if (config_.type == FacilityType::kDoor) {
    // 读 2 个线圈状态：0=Open Limit, 1=Closed Limit
    auto resp = client_->ReadCoils(0x0000, 2);
    if (!resp.success || resp.coil_values.size() < 2) {
      report.state = FacilityState::kOffline;
      return report;
    }
    const bool is_open = resp.coil_values[0];
    const bool is_closed = resp.coil_values[1];
    report.state = FacilityState::kReady;
    if (is_open && !is_closed) {
      report.door_status = "open";
    } else if (!is_open && is_closed) {
      report.door_status = "closed";
    } else {
      report.door_status = "moving";
      report.state = FacilityState::kBusy;
    }
  } else if (config_.type == FacilityType::kElevator) {
    // 读保持寄存器 0x0001 (当前楼层)
    auto resp = client_->ReadHoldingRegisters(0x0001, 1);
    if (!resp.success || resp.register_values.empty()) {
      report.state = FacilityState::kOffline;
      return report;
    }
    report.current_floor = std::to_string(resp.register_values[0]);
    report.state = FacilityState::kReady;
    report.door_status = "closed";
  }

  return report;
}

FacilityCommandResult ModbusFacilityAdapter::ExecuteAction(
    const std::string& action, const std::string& params) {
  FacilityCommandResult res;
  res.facility_id = config_.facility_id;
  res.action = action;

  if (!client_) {
    res.success = false;
    res.message = "client uninitialized";
    res.error_code = 1;
    return res;
  }

  if (config_.type == FacilityType::kDoor) {
    if (action == "open") {
      auto resp = client_->WriteSingleCoil(0x0000, true);
      res.success = resp.success;
      res.message = resp.success ? "door open command sent" : resp.error_message;
      return res;
    }
    if (action == "close") {
      auto resp = client_->WriteSingleCoil(0x0001, true);
      res.success = resp.success;
      res.message = resp.success ? "door close command sent" : resp.error_message;
      return res;
    }
  } else if (config_.type == FacilityType::kElevator) {
    if (action == "call" || action == "send_to_floor") {
      int floor = 1;
      try {
        if (!params.empty()) floor = std::stoi(params);
      } catch (...) {}
      auto resp = client_->WriteSingleRegister(0x0000, static_cast<uint16_t>(floor));
      res.success = resp.success;
      res.message = resp.success ? "elevator target floor sent" : resp.error_message;
      return res;
    }
  }

  res.success = false;
  res.message = "unsupported action: " + action;
  res.error_code = 2;
  return res;
}

bool ModbusFacilityAdapter::AcquireLock(const std::string& holder_id) {
  if (locked_ && locked_by_ != holder_id) {
    return false;
  }
  locked_ = true;
  locked_by_ = holder_id;
  return true;
}

bool ModbusFacilityAdapter::ReleaseLock(const std::string& holder_id) {
  if (!locked_ || locked_by_ != holder_id) {
    return false;
  }
  locked_ = false;
  locked_by_.clear();
  return true;
}

}  // namespace amr_dispatcher_core::facility
