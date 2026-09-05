#include "amr_dispatcher_core/chassis/chassis_packet.hpp"


#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace amr_dispatcher_core::chassis {
namespace {

std::string TrimCarriageReturn(std::string line) {
  if (!line.empty() && line.back() == '\r') {
    line.pop_back();
  }
  return line;
}

void AppendInt16(std::vector<uint8_t>* out, const int16_t value) {
  out->push_back(static_cast<uint8_t>((value >> 8) & 0xff));
  out->push_back(static_cast<uint8_t>(value & 0xff));
}

int16_t ReadInt16(const std::vector<uint8_t>& payload, const std::size_t offset) {
  return static_cast<int16_t>(
      static_cast<uint16_t>(payload.at(offset) << 8) | static_cast<uint16_t>(payload.at(offset + 1)));
}

uint16_t ReadUint16(const std::vector<uint8_t>& payload, const std::size_t offset) {
  return static_cast<uint16_t>(
      static_cast<uint16_t>(payload.at(offset) << 8) | static_cast<uint16_t>(payload.at(offset + 1)));
}

int16_t ClampToInt16(const double value) {
  const double clamped = std::clamp(value, -32768.0, 32767.0);
  return static_cast<int16_t>(std::lround(clamped));
}

}  // namespace

std::string EncodeCommand(const ChassisCommand& command) {
  std::ostringstream out;
  out.setf(std::ios::fixed);
  out << std::setprecision(4) << "CMD " << command.linear_x_mps << " "
      << command.linear_y_mps << " "
      << command.angular_z_radps << "\n";
  return out.str();
}

std::string EncodeOdometry(const ChassisOdometryPacket& odometry) {
  std::ostringstream out;
  out.setf(std::ios::fixed);
  out << std::setprecision(4) << "ODOM " << odometry.x_m << " " << odometry.y_m << " "
      << odometry.yaw_rad << " " << odometry.linear_x_mps << " " << odometry.angular_z_radps;
  if (odometry.battery_voltage.has_value()) {
    out << " " << *odometry.battery_voltage;
  }
  out << "\n";
  return out.str();
}

std::string EncodeState(const ChassisStatePacket& state) {
  std::ostringstream out;
  out.setf(std::ios::fixed);
  out << "STATE " << state.status;
  if (state.battery_voltage.has_value()) {
    out << " " << std::setprecision(4) << *state.battery_voltage;
  }
  out << "\n";
  return out.str();
}

std::optional<ChassisCommand> ParseCommandLine(const std::string& raw_line) {
  const std::string line = TrimCarriageReturn(raw_line);
  std::istringstream in(line);
  std::string tag;
  ChassisCommand command;
  if (!(in >> tag >> command.linear_x_mps) || tag != "CMD") {
    return std::nullopt;
  }
  double second = 0.0;
  if (!(in >> second)) {
    return std::nullopt;
  }
  double third = 0.0;
  if (in >> third) {
    command.linear_y_mps = second;
    command.angular_z_radps = third;
  } else {
    command.angular_z_radps = second;
  }
  std::string trailing;
  if (in >> trailing) {
    return std::nullopt;
  }
  return command;
}

std::optional<ChassisPacket> ParsePacketLine(const std::string& raw_line) {
  const std::string line = TrimCarriageReturn(raw_line);
  std::istringstream in(line);
  std::string tag;
  in >> tag;
  if (tag == "ODOM") {
    ChassisOdometryPacket odometry;
    if (!(in >> odometry.x_m >> odometry.y_m >> odometry.yaw_rad >> odometry.linear_x_mps >>
          odometry.angular_z_radps)) {
      return std::nullopt;
    }
    double battery = 0.0;
    if (in >> battery) {
      odometry.battery_voltage = battery;
    }
    std::string trailing;
    if (in >> trailing) {
      return std::nullopt;
    }
    ChassisPacket packet;
    packet.kind = ChassisPacketKind::kOdometry;
    packet.odometry = odometry;
    return packet;
  }

  if (tag == "STATE") {
    ChassisStatePacket state;
    if (!(in >> state.status)) {
      return std::nullopt;
    }
    double battery = 0.0;
    if (in >> battery) {
      state.battery_voltage = battery;
    }
    std::string trailing;
    if (in >> trailing) {
      return std::nullopt;
    }
    ChassisPacket packet;
    packet.kind = ChassisPacketKind::kState;
    packet.state = state;
    return packet;
  }

  return std::nullopt;
}

std::vector<uint8_t> EncodeMickBinaryFrame(
    const uint8_t type, const std::vector<uint8_t>& payload) {
  std::vector<uint8_t> frame;
  frame.reserve(payload.size() + 7);
  frame.push_back(0xae);
  frame.push_back(0xea);
  frame.push_back(static_cast<uint8_t>(payload.size() + 2U));
  frame.push_back(type);
  frame.insert(frame.end(), payload.begin(), payload.end());
  uint8_t checksum = 0;
  for (std::size_t i = 2; i < frame.size(); ++i) {
    checksum = static_cast<uint8_t>(checksum + frame[i]);
  }
  frame.push_back(checksum);
  frame.push_back(0xef);
  frame.push_back(0xfe);
  return frame;
}

std::vector<uint8_t> EncodeMickSpeedCommand(const ChassisCommand& command) {
  std::vector<uint8_t> payload;
  payload.reserve(6);
  AppendInt16(&payload, ClampToInt16(command.linear_x_mps * 1000.0));
  AppendInt16(&payload, ClampToInt16(command.linear_y_mps * 1000.0));
  AppendInt16(&payload, ClampToInt16(command.angular_z_radps * 1000.0));
  return EncodeMickBinaryFrame(0xf3, payload);
}

std::vector<uint8_t> EncodeMickClearOdometryCommand() {
  return EncodeMickBinaryFrame(0xe1, {0x01, 0x00, 0x00, 0x00});
}

std::optional<ChassisPacket> DecodeMickBinaryPacket(const MickBinaryFrame& frame) {
  if (frame.type == 0xa1 && frame.payload.size() >= 12U) {
    ChassisOdometryPacket odometry;
    odometry.x_m = ReadInt16(frame.payload, 0) / 1000.0;
    odometry.y_m = ReadInt16(frame.payload, 2) / 1000.0;
    odometry.yaw_rad = ReadInt16(frame.payload, 4) / 1000.0;
    odometry.linear_x_mps = ReadInt16(frame.payload, 6) / 1000.0;
    odometry.angular_z_radps = ReadInt16(frame.payload, 8) / 1000.0;
    odometry.battery_voltage = ReadUint16(frame.payload, 10) / 100.0;
    ChassisPacket packet;
    packet.kind = ChassisPacketKind::kOdometry;
    packet.odometry = odometry;
    return packet;
  }

  if (frame.type == 0xa2) {
    ChassisStatePacket state;
    state.status = frame.payload.empty() || frame.payload[0] == 0 ? "running" : "fault";
    if (frame.payload.size() >= 3U) {
      state.battery_voltage = ReadUint16(frame.payload, 1) / 100.0;
    }
    ChassisPacket packet;
    packet.kind = ChassisPacketKind::kState;
    packet.state = state;
    return packet;
  }

  return std::nullopt;
}

}  // namespace amr_dispatcher_core::chassis
