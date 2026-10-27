// Modbus-TCP 纯 C++ 客户端实现（POSIX Socket，零外部三方依赖）。
//
// 对应 Phase 8 (P0)：
// 支持工业标准 Modbus-TCP 报文构建与解析，用于与现场 PLC、自动门控制器与提升机通讯。
//
// 协议规范：
// 1. MBAP 报文头（7 字节）：
//    - Transaction ID (2 字节): 事务标识，自增保序
//    - Protocol ID    (2 字节): 固定 0x0000 (Modbus)
//    - Length         (2 字节): 后续字节数 (Unit ID + PDU)
//    - Unit ID        (1 字节): 设备从站地址 (通常为 1 或 255)
// 2. 支持功能码 (Function Codes)：
//    - 0x01: Read Coils（读线圈/输出状态，如开门中/门已完全开启）
//    - 0x02: Read Discrete Inputs（读离散输入，如限位开关）
//    - 0x03: Read Holding Registers（读保持寄存器，如电梯当前楼层/故障字）
//    - 0x05: Write Single Coil（写单线圈，如触发开门/关门指令）
//    - 0x06: Write Single Register（写单寄存器，如设定电梯目标楼层）
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace amr_dispatcher_core::facility {

struct ModbusResponse {
  bool success = false;
  uint8_t function_code = 0;
  uint8_t exception_code = 0;
  std::vector<bool> coil_values;
  std::vector<uint16_t> register_values;
  std::string error_message;
};

// 纯函数报文编解码器（供单元测试和客户端共享，100% 内存单测可覆盖）
class ModbusPacketCodec {
 public:
  static std::vector<uint8_t> BuildReadCoilsRequest(
      uint16_t transaction_id, uint8_t unit_id, uint16_t start_addr, uint16_t count);

  static std::vector<uint8_t> BuildReadHoldingRegistersRequest(
      uint16_t transaction_id, uint8_t unit_id, uint16_t start_addr, uint16_t count);

  static std::vector<uint8_t> BuildWriteSingleCoilRequest(
      uint16_t transaction_id, uint8_t unit_id, uint16_t coil_addr, bool value);

  static std::vector<uint8_t> BuildWriteSingleRegisterRequest(
      uint16_t transaction_id, uint8_t unit_id, uint16_t reg_addr, uint16_t value);

  static ModbusResponse ParseResponse(
      uint16_t expected_tx_id, const uint8_t* data, std::size_t length);
};

// Modbus-TCP POSIX Socket 客户端
class ModbusTcpClient {
 public:
  ModbusTcpClient(std::string ip, uint16_t port, uint8_t unit_id = 1,
                  int timeout_ms = 2000);
  ~ModbusTcpClient();

  // 禁止拷贝
  ModbusTcpClient(const ModbusTcpClient&) = delete;
  ModbusTcpClient& operator=(const ModbusTcpClient&) = delete;

  bool Connect(std::string* error_msg = nullptr);
  void Disconnect();
  bool IsConnected() const;

  ModbusResponse ReadCoils(uint16_t start_addr, uint16_t count);
  ModbusResponse ReadHoldingRegisters(uint16_t start_addr, uint16_t count);
  ModbusResponse WriteSingleCoil(uint16_t coil_addr, bool value);
  ModbusResponse WriteSingleRegister(uint16_t reg_addr, uint16_t value);

 private:
  ModbusResponse Transceive(const std::vector<uint8_t>& request, uint16_t tx_id);

  std::string ip_;
  uint16_t port_;
  uint8_t unit_id_;
  int timeout_ms_;
  int sock_fd_ = -1;
  uint16_t next_transaction_id_ = 1;
};

}  // namespace amr_dispatcher_core::facility
