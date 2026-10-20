#pragma once

#include "amr_dispatcher_core/fleet/vehicle_adapter_interface.hpp"

#include <optional>
#include <string>
#include <vector>

namespace amr_dispatcher_core::fleet {

// 1. 将内部 VehicleOrder 序列化为符合 VDA 5050 规范的 Order JSON 报文
// version 参数支持 "v2"（默认，协议版本 2.0.0）或 "v3"（协议版本 3.0.0）
std::string SerializeVda5050Order(const VehicleOrder& order,
                                  const std::string& version = "v2",
                                  const std::string& manufacturer = "RobotManufacturer");

// 2. 序列化 InstantActions 报文（用于急停 cancelOrder, pause, resume 等）
std::string SerializeVda5050InstantActions(const std::string& vehicle_id,
                                          const std::vector<VehicleAction>& actions,
                                          const std::string& version = "v2",
                                          const std::string& manufacturer = "RobotManufacturer");

// 3. 反序列化小车上报的 VDA 5050 State JSON 报文为内部 VehicleStateReport
// 若 JSON 非法或缺少必须字段，返回 std::nullopt
std::optional<VehicleStateReport> DeserializeVda5050State(const std::string& json_str);

// 4. 反序列化小车上报的 Connection JSON 报文
// 返回 connectionState 字符串（如 "ONLINE", "CONNECTIONBROKEN", "OFFLINE"），若非法返回 std::nullopt
std::optional<std::string> DeserializeVda5050Connection(const std::string& json_str);

}  // namespace amr_dispatcher_core::fleet
