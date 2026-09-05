#pragma once

#include <string>

#include "amr_dispatcher_core/chassis/chassis_backend.hpp"

namespace amr_dispatcher_core::chassis {

class SerialBackend final : public ChassisBackend {
 public:
  explicit SerialBackend(ChassisBackendConfig config);
  ~SerialBackend() override;

  std::string Name() const override;
  bool Open(std::string* error) override;
  void Close() override;
  bool IsOpen() const override;
  bool WriteCommand(const ChassisCommand& command, std::string* error) override;
  std::optional<std::string> Read(std::string* error) override;

 private:
  ChassisBackendConfig config_;
  int fd_ = -1;
};

}  // namespace amr_dispatcher_core::chassis

