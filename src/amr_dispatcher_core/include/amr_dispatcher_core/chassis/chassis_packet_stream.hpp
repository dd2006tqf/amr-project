#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "amr_dispatcher_core/chassis/chassis_packet.hpp"

namespace amr_dispatcher_core::chassis {

// 将字节流切分为文本协议数据包（每行一条，以 '\n' 结尾）。
// 内部维护粘包/半包缓冲：Append() 可任意切分追加，DrainPackets() 一次取干。
class ChassisPacketStream {
 public:
  explicit ChassisPacketStream(std::size_t max_buffer_bytes = 4096);

  void Append(const char* data, std::size_t size);
  void Append(const std::string& data);
  std::vector<ChassisPacket> DrainPackets();
  bool Overflowed() const;
  std::size_t InvalidLineCount() const;
  void ClearOverflow();
  void ClearInvalidLineCount();
  void Clear();

 private:
  std::size_t max_buffer_bytes_;
  std::string buffer_;
  std::vector<ChassisPacket> pending_packets_;
  bool overflowed_ = false;
  std::size_t invalid_line_count_ = 0;
};

// 将字节流切分为 Mick 二进制帧（0xAE 0xEA ... 0xEF 0xFE），带校验和。
class MickBinaryPacketStream {
 public:
  explicit MickBinaryPacketStream(std::size_t max_buffer_bytes = 4096);

  void Append(const char* data, std::size_t size);
  void Append(const std::string& data);
  std::vector<ChassisPacket> DrainPackets();
  bool Overflowed() const;
  std::size_t InvalidFrameCount() const;
  void ClearOverflow();
  void ClearInvalidFrameCount();
  void Clear();

 private:
  std::size_t max_buffer_bytes_;
  std::vector<uint8_t> buffer_;
  bool overflowed_ = false;
  std::size_t invalid_frame_count_ = 0;
};

}  // namespace amr_dispatcher_core::chassis
