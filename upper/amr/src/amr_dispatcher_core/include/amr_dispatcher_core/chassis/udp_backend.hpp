#pragma once

#include <netinet/in.h>

#include "amr_dispatcher_core/chassis/chassis_backend.hpp"

namespace amr_dispatcher_core::chassis {

class UdpBackend final : public ChassisBackend {
 public:
  explicit UdpBackend(ChassisBackendConfig config);
  ~UdpBackend() override;

  std::string Name() const override;
  bool Open(std::string* error) override;
  void Close() override;
  bool IsOpen() const override;
  bool WriteCommand(const ChassisCommand& command, std::string* error) override;
  std::optional<std::string> Read(std::string* error) override;

 private:
  ChassisBackendConfig config_;
  int fd_ = -1;
  sockaddr_in remote_addr_ {};
};

}  // namespace amr_dispatcher_core::chassis

