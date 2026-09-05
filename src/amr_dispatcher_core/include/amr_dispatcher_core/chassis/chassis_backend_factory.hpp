#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "amr_dispatcher_core/chassis/chassis_backend.hpp"

namespace amr_dispatcher_core::chassis {

// 描述一个后端：name + 工厂。factory 接受 ChassisBackendConfig 并产出 backend。
// BackendDegradationController 用一组 descriptor 实现 "preferred list" 切换。
struct BackendDescriptor {
  std::string name;
  std::function<std::unique_ptr<ChassisBackend>(const ChassisBackendConfig&)> factory;
};

// 通用工厂：把后端 name 映射成实际对象（与 chassis_backend_factory.cpp 兼容）。
std::unique_ptr<ChassisBackend> CreateChassisBackend(
    const std::string& backend, const ChassisBackendConfig& config);

// 默认 preferred list：serial → udp → mock
std::vector<BackendDescriptor> DefaultBackendDescriptors();

}  // namespace amr_dispatcher_core::chassis
