#include "amr_dispatcher_core/fleet/vehicle_adapter_interface.hpp"

namespace amr_dispatcher_core::fleet {

bool MockVehicleAdapter::DispatchOrder(const std::string& vehicle_id, const VehicleOrder& order) {
  orders_[vehicle_id] = order;
  auto it = states_.find(vehicle_id);
  if (it != states_.end()) {
    it->second.order_id = order.order_id;
    it->second.last_report_time = std::chrono::steady_clock::now();
  }
  return true;
}

bool MockVehicleAdapter::CancelOrder(const std::string& vehicle_id, const std::string& order_id) {
  auto it = orders_.find(vehicle_id);
  if (it == orders_.end() || it->second.order_id != order_id) {
    return false;
  }
  orders_.erase(it);
  auto state = states_.find(vehicle_id);
  if (state != states_.end()) {
    state->second.order_id.clear();
    state->second.last_report_time = std::chrono::steady_clock::now();
  }
  return true;
}

bool MockVehicleAdapter::PauseVehicle(const std::string& vehicle_id) {
  auto it = states_.find(vehicle_id);
  if (it == states_.end()) {
    return false;
  }
  it->second.operating_mode = "PAUSED";
  it->second.last_report_time = std::chrono::steady_clock::now();
  return true;
}

bool MockVehicleAdapter::ResumeVehicle(const std::string& vehicle_id) {
  auto it = states_.find(vehicle_id);
  if (it == states_.end()) {
    return false;
  }
  it->second.operating_mode = "AUTOMATIC";
  it->second.last_report_time = std::chrono::steady_clock::now();
  return true;
}

std::optional<VehicleStateReport> MockVehicleAdapter::GetVehicleState(
    const std::string& vehicle_id) const {
  auto it = states_.find(vehicle_id);
  if (it == states_.end()) {
    return std::nullopt;
  }
  return it->second;
}

void MockVehicleAdapter::SetVehicleState(const VehicleStateReport& report) {
  states_[report.vehicle_id] = report;
}

std::optional<VehicleOrder> MockVehicleAdapter::GetDispatchedOrder(
    const std::string& vehicle_id) const {
  auto it = orders_.find(vehicle_id);
  if (it == orders_.end()) {
    return std::nullopt;
  }
  return it->second;
}

}  // namespace amr_dispatcher_core::fleet
