#pragma once

#include <optional>
#include <string>

#include "amr_dispatcher_core/chassis/chassis_packet.hpp"

namespace amr_dispatcher_core::chassis {

struct ChassisBackendConfig {
  std::string serial_device = "/dev/ttyUSB0";
  int serial_baud = 115200;
  std::string udp_host = "192.168.1.10";
  int udp_port = 9000;
  std::string protocol = "text";
};

class ChassisBackend {
 public:
  virtual ~ChassisBackend() = default;

  virtual std::string Name() const = 0;
  virtual bool Open(std::string* error) = 0;
  virtual void Close() = 0;
  virtual bool IsOpen() const = 0;
  virtual bool WriteCommand(const ChassisCommand& command, std::string* error) = 0;
  virtual std::optional<std::string> Read(std::string* error) = 0;
};

}  // namespace amr_dispatcher_core::chassis
