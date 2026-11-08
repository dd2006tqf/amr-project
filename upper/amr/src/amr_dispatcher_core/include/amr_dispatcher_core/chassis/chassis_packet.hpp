#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace amr_dispatcher_core::chassis {

struct ChassisCommand {
  double linear_x_mps = 0.0;
  double linear_y_mps = 0.0;
  double angular_z_radps = 0.0;
  // 下行帧序号（uint8 回绕）。仅 mick_binary 协议使用：MCU 回显到上行
  // 0xA1 的 last_cmd_seq，主机据此算“命令送达率”。
  // 没有它，ChassisLinkHealth 的链路质量只能统计读空与 CRC 错，
  // 命令真的在串口上丢了是测不出来的（avg_latency_ms 也恒为 0）。
  uint8_t sequence = 0;
};

// 上行 0xA1 的第 12 字节（可选扩展）。缺字节 = 全部为 0 = 无故障。
enum MickOdometryFlags : uint8_t {
  kNoFault = 0,
  kFault = 1u << 0,        // MCU 侧综合故障（PID 发散、编码器异常等）
  kUartTimeout = 1u << 1,  // 连续 200ms 未收到合法下行帧
  kEstopInput = 1u << 2,   // 急停输入有效（部分底盘会把急停状态回传）
  kOverCurrent = 1u << 3,  // 驱动器过流/过温
};

inline bool HasMickFlag(const uint8_t flags, const MickOdometryFlags bit) {
  return (flags & static_cast<uint8_t>(bit)) != 0;
}

struct ChassisOdometryPacket {
  double x_m = 0.0;
  double y_m = 0.0;
  double yaw_rad = 0.0;
  double linear_x_mps = 0.0;
  double angular_z_radps = 0.0;
  std::optional<double> battery_voltage;
  // T2/T3：上行扩展字节。旧固件（payload 12 字节）解码后两者均为 0。
  uint8_t flags = 0;
  uint8_t last_cmd_seq = 0;
};

struct ChassisStatePacket {
  std::string status;
  std::optional<double> battery_voltage;
  // T2：0xA2 的 payload[0] != 0 一直映射成 status == "fault"，但上层只比对字符串。
  // 单独暴露布尔位，让“底盘报故障”能真正进安全链（心跳下沉 -> 安全门零速）。
  bool fault = false;
};

enum class ChassisPacketKind { kOdometry, kState };

struct ChassisPacket {
  ChassisPacketKind kind = ChassisPacketKind::kState;
  ChassisOdometryPacket odometry;
  ChassisStatePacket state;
};

std::string EncodeCommand(const ChassisCommand& command);
std::string EncodeOdometry(const ChassisOdometryPacket& odometry);
std::string EncodeState(const ChassisStatePacket& state);
std::optional<ChassisCommand> ParseCommandLine(const std::string& line);
std::optional<ChassisPacket> ParsePacketLine(const std::string& line);

struct MickBinaryFrame {
  uint8_t type = 0;
  std::vector<uint8_t> payload;
};

std::vector<uint8_t> EncodeMickBinaryFrame(uint8_t type, const std::vector<uint8_t>& payload);
std::vector<uint8_t> EncodeMickSpeedCommand(const ChassisCommand& command);
std::vector<uint8_t> EncodeMickClearOdometryCommand();
std::optional<ChassisPacket> DecodeMickBinaryPacket(const MickBinaryFrame& frame);

}  // namespace amr_dispatcher_core::chassis
