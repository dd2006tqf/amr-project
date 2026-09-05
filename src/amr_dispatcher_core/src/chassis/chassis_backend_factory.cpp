#include "amr_dispatcher_core/chassis/chassis_backend_factory.hpp"

#include "amr_dispatcher_core/chassis/mock_backend.hpp"
#include "amr_dispatcher_core/chassis/serial_backend.hpp"
#include "amr_dispatcher_core/chassis/udp_backend.hpp"

namespace amr_dispatcher_core::chassis {

std::unique_ptr<ChassisBackend> CreateChassisBackend(
    const std::string& backend, const ChassisBackendConfig& config) {
  if (backend == "serial") {
    return std::make_unique<SerialBackend>(config);
  }
  if (backend == "udp") {
    return std::make_unique<UdpBackend>(config);
  }
  return std::make_unique<MockBackend>();
}

std::vector<BackendDescriptor> DefaultBackendDescriptors() {
  return {
      {"serial",
       [](const ChassisBackendConfig& c) { return std::make_unique<SerialBackend>(c); }},
      {"udp",
       [](const ChassisBackendConfig& c) { return std::make_unique<UdpBackend>(c); }},
      {"mock", [](const ChassisBackendConfig&) { return std::make_unique<MockBackend>(); }},
  };
}

}  // namespace amr_dispatcher_core::chassis
