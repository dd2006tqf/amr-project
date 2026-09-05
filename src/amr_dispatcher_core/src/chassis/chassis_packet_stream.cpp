#include "amr_dispatcher_core/chassis/chassis_packet_stream.hpp"

#include <algorithm>
#include <array>
#include <utility>

namespace amr_dispatcher_core::chassis {

ChassisPacketStream::ChassisPacketStream(const std::size_t max_buffer_bytes)
    : max_buffer_bytes_(max_buffer_bytes) {}

void ChassisPacketStream::Append(const char* data, const std::size_t size) {
  if (overflowed_) {
    return;
  }
  buffer_.append(data, size);
  if (buffer_.size() > max_buffer_bytes_) {
    buffer_.clear();
    overflowed_ = true;
    return;
  }
  // 同步解析完整行：每解析完一行就从 buffer 摘掉，
  // 让 InvalidLineCount() 在 Append 之后即可见（无须 DrainPackets）。
  std::size_t newline = std::string::npos;
  while ((newline = buffer_.find('\n')) != std::string::npos) {
    const std::string line = buffer_.substr(0, newline);
    buffer_.erase(0, newline + 1);
    if (auto packet = ParsePacketLine(line)) {
      pending_packets_.push_back(*packet);
    } else if (!line.empty()) {
      ++invalid_line_count_;
    }
  }
}

void ChassisPacketStream::Append(const std::string& data) { Append(data.data(), data.size()); }

std::vector<ChassisPacket> ChassisPacketStream::DrainPackets() {
  std::vector<ChassisPacket> packets;
  packets.swap(pending_packets_);
  return packets;
}

bool ChassisPacketStream::Overflowed() const { return overflowed_; }

std::size_t ChassisPacketStream::InvalidLineCount() const { return invalid_line_count_; }

void ChassisPacketStream::ClearOverflow() { overflowed_ = false; }

void ChassisPacketStream::ClearInvalidLineCount() { invalid_line_count_ = 0; }

void ChassisPacketStream::Clear() {
  buffer_.clear();
  pending_packets_.clear();
  overflowed_ = false;
  invalid_line_count_ = 0;
}

MickBinaryPacketStream::MickBinaryPacketStream(const std::size_t max_buffer_bytes)
    : max_buffer_bytes_(max_buffer_bytes) {}

void MickBinaryPacketStream::Append(const char* data, const std::size_t size) {
  buffer_.insert(buffer_.end(), data, data + size);
  if (buffer_.size() > max_buffer_bytes_) {
    buffer_.clear();
    overflowed_ = true;
  }
}

void MickBinaryPacketStream::Append(const std::string& data) { Append(data.data(), data.size()); }

std::vector<ChassisPacket> MickBinaryPacketStream::DrainPackets() {
  std::vector<ChassisPacket> packets;
  while (buffer_.size() >= 7U) {
    const std::array<uint8_t, 2> magic{0xae, 0xea};
    const auto header = std::search(buffer_.begin(), buffer_.end(), magic.begin(), magic.end());
    if (header == buffer_.end()) {
      buffer_.clear();
      return packets;
    }
    buffer_.erase(buffer_.begin(), header);
    if (buffer_.size() < 7U) {
      return packets;
    }
    const std::size_t length = buffer_[2];
    const std::size_t total_length = 2U + 1U + length + 2U;
    if (length < 2U) {
      buffer_.erase(buffer_.begin());
      ++invalid_frame_count_;
      continue;
    }
    if (buffer_.size() < total_length) {
      return packets;
    }
    if (buffer_[total_length - 2U] != 0xef || buffer_[total_length - 1U] != 0xfe) {
      buffer_.erase(buffer_.begin());
      ++invalid_frame_count_;
      continue;
    }
    uint8_t checksum = 0;
    for (std::size_t i = 2; i < total_length - 3U; ++i) {
      checksum = static_cast<uint8_t>(checksum + buffer_[i]);
    }
    if (checksum != buffer_[total_length - 3U]) {
      buffer_.erase(buffer_.begin(), buffer_.begin() + total_length);
      ++invalid_frame_count_;
      continue;
    }
    MickBinaryFrame frame;
    frame.type = buffer_[3];
    frame.payload.assign(buffer_.begin() + 4, buffer_.begin() + total_length - 3U);
    if (auto packet = DecodeMickBinaryPacket(frame)) {
      packets.push_back(*packet);
    }
    buffer_.erase(buffer_.begin(), buffer_.begin() + total_length);
  }
  return packets;
}

bool MickBinaryPacketStream::Overflowed() const { return overflowed_; }

std::size_t MickBinaryPacketStream::InvalidFrameCount() const { return invalid_frame_count_; }

void MickBinaryPacketStream::ClearOverflow() { overflowed_ = false; }

void MickBinaryPacketStream::ClearInvalidFrameCount() { invalid_frame_count_ = 0; }

void MickBinaryPacketStream::Clear() {
  buffer_.clear();
  overflowed_ = false;
  invalid_frame_count_ = 0;
}

}  // namespace amr_dispatcher_core::chassis
