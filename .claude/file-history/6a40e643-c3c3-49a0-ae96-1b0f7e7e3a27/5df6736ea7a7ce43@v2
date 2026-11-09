// 工业物理设施 Modbus-TCP 适配器实现。
//
// 对应 Phase 8 (P0)：
// 将通用的 IFacilityAdapter 接口映射到底层 Modbus-TCP 寄存器与线圈：
// 1. 自动卷帘门映射：
//    - 线圈 0x0000: 开门指令 (写 1)
//    - 线圈 0x0001: 关门指令 (写 1)
//    - 离散输入 0x0000: 门已完全打开 (Limit Switch Open)
//    - 离散输入 0x0001: 门已完全关闭 (Limit Switch Closed)
// 2. 跨楼层电梯映射：
//    - 保持寄存器 0x0000: 目标呼梯楼层
//    - 保持寄存器 0x0001: 当前实时楼层
//    - 离散输入/线圈 0x0002: 梯门状态 (1=Open, 0=Closed)
#pragma once

#include <memory>
#include <string>

#include "amr_dispatcher_core/facility/facility_adapter_interface.hpp"
#include "amr_dispatcher_core/facility/modbus_tcp_client.hpp"

namespace amr_dispatcher_core::facility {

struct ModbusFacilityConfig {
  std::string facility_id;
  FacilityType type = FacilityType::kDoor;
  std::string ip = "127.0.0.1";
  uint16_t port = 502;
  uint8_t unit_id = 1;
  int timeout_ms = 1000;
};

class ModbusFacilityAdapter : public IFacilityAdapter {
 public:
  explicit ModbusFacilityAdapter(ModbusFacilityConfig config);
  // 支持传入注入 client 的构造器（用于单元测试无缝 Mock）
  ModbusFacilityAdapter(ModbusFacilityConfig config,
                        std::shared_ptr<ModbusTcpClient> client);
  ~ModbusFacilityAdapter() override = default;

  std::string GetFacilityId() const override { return config_.facility_id; }
  FacilityType GetFacilityType() const override { return config_.type; }

  FacilityStateReport QueryState() override;
  FacilityCommandResult ExecuteAction(const std::string& action,
                                      const std::string& params = "") override;

  bool AcquireLock(const std::string& holder_id) override;
  bool ReleaseLock(const std::string& holder_id) override;

 private:
  ModbusFacilityConfig config_;
  std::shared_ptr<ModbusTcpClient> client_;
  bool locked_ = false;
  std::string locked_by_;
};

}  // namespace amr_dispatcher_core::facility
