#include "amr_dispatcher_core/facility/modbus_tcp_client.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <cstring>

namespace amr_dispatcher_core::facility {

std::vector<uint8_t> ModbusPacketCodec::BuildReadCoilsRequest(
    uint16_t transaction_id, uint8_t unit_id, uint16_t start_addr, uint16_t count) {
  // MBAP (7 bytes) + PDU (Function 0x01 + 2 bytes addr + 2 bytes count) = 12 bytes
  std::vector<uint8_t> req(12);
  req[0] = static_cast<uint8_t>(transaction_id >> 8);
  req[1] = static_cast<uint8_t>(transaction_id & 0xFF);
  req[2] = 0x00;  // Protocol ID MSB
  req[3] = 0x00;  // Protocol ID LSB
  req[4] = 0x00;  // Length MSB (6 bytes follow: unit_id + func + start_addr + count)
  req[5] = 0x06;  // Length LSB
  req[6] = unit_id;
  req[7] = 0x01;  // Function Code: Read Coils
  req[8] = static_cast<uint8_t>(start_addr >> 8);
  req[9] = static_cast<uint8_t>(start_addr & 0xFF);
  req[10] = static_cast<uint8_t>(count >> 8);
  req[11] = static_cast<uint8_t>(count & 0xFF);
  return req;
}

std::vector<uint8_t> ModbusPacketCodec::BuildReadHoldingRegistersRequest(
    uint16_t transaction_id, uint8_t unit_id, uint16_t start_addr, uint16_t count) {
  std::vector<uint8_t> req(12);
  req[0] = static_cast<uint8_t>(transaction_id >> 8);
  req[1] = static_cast<uint8_t>(transaction_id & 0xFF);
  req[2] = 0x00;
  req[3] = 0x00;
  req[4] = 0x00;
  req[5] = 0x06;
  req[6] = unit_id;
  req[7] = 0x03;  // Function Code: Read Holding Registers
  req[8] = static_cast<uint8_t>(start_addr >> 8);
  req[9] = static_cast<uint8_t>(start_addr & 0xFF);
  req[10] = static_cast<uint8_t>(count >> 8);
  req[11] = static_cast<uint8_t>(count & 0xFF);
  return req;
}

std::vector<uint8_t> ModbusPacketCodec::BuildWriteSingleCoilRequest(
    uint16_t transaction_id, uint8_t unit_id, uint16_t coil_addr, bool value) {
  std::vector<uint8_t> req(12);
  req[0] = static_cast<uint8_t>(transaction_id >> 8);
  req[1] = static_cast<uint8_t>(transaction_id & 0xFF);
  req[2] = 0x00;
  req[3] = 0x00;
  req[4] = 0x00;
  req[5] = 0x06;
  req[6] = unit_id;
  req[7] = 0x05;  // Function Code: Write Single Coil
  req[8] = static_cast<uint8_t>(coil_addr >> 8);
  req[9] = static_cast<uint8_t>(coil_addr & 0xFF);
  req[10] = value ? 0xFF : 0x00;  // 0xFF00 = ON, 0x0000 = OFF
  req[11] = 0x00;
  return req;
}

std::vector<uint8_t> ModbusPacketCodec::BuildWriteSingleRegisterRequest(
    uint16_t transaction_id, uint8_t unit_id, uint16_t reg_addr, uint16_t value) {
  std::vector<uint8_t> req(12);
  req[0] = static_cast<uint8_t>(transaction_id >> 8);
  req[1] = static_cast<uint8_t>(transaction_id & 0xFF);
  req[2] = 0x00;
  req[3] = 0x00;
  req[4] = 0x00;
  req[5] = 0x06;
  req[6] = unit_id;
  req[7] = 0x06;  // Function Code: Write Single Register
  req[8] = static_cast<uint8_t>(reg_addr >> 8);
  req[9] = static_cast<uint8_t>(reg_addr & 0xFF);
  req[10] = static_cast<uint8_t>(value >> 8);
  req[11] = static_cast<uint8_t>(value & 0xFF);
  return req;
}

ModbusResponse ModbusPacketCodec::ParseResponse(
    uint16_t expected_tx_id, const uint8_t* data, std::size_t length) {
  ModbusResponse resp;
  if (data == nullptr || length < 9) {
    resp.error_message = "response too short for Modbus MBAP + PDU";
    return resp;
  }

  const uint16_t rx_tx_id = (static_cast<uint16_t>(data[0]) << 8) | data[1];
  if (rx_tx_id != expected_tx_id) {
    resp.error_message = "transaction ID mismatch";
    return resp;
  }

  const uint16_t proto_id = (static_cast<uint16_t>(data[2]) << 8) | data[3];
  if (proto_id != 0) {
    resp.error_message = "protocol ID is not Modbus (expected 0)";
    return resp;
  }

  resp.function_code = data[7];

  // 异常响应：最高位为 1 (func | 0x80)
  if (resp.function_code & 0x80) {
    resp.exception_code = data[8];
    resp.error_message = "modbus exception code: " + std::to_string(resp.exception_code);
    return resp;
  }

  // 正常响应解析
  if (resp.function_code == 0x01 || resp.function_code == 0x02) {
    // Read Coils: data[8] is byte count
    const uint8_t byte_count = data[8];
    if (length < 9U + byte_count) {
      resp.error_message = "truncated coil data";
      return resp;
    }
    for (uint8_t b = 0; b < byte_count; ++b) {
      const uint8_t byte_val = data[9 + b];
      for (int bit = 0; bit < 8; ++bit) {
        resp.coil_values.push_back((byte_val & (1 << bit)) != 0);
      }
    }
    resp.success = true;
  } else if (resp.function_code == 0x03) {
    // Read Holding Registers: data[8] is byte count
    const uint8_t byte_count = data[8];
    if (length < 9U + byte_count || byte_count % 2 != 0) {
      resp.error_message = "truncated register data";
      return resp;
    }
    for (uint8_t i = 0; i < byte_count; i += 2) {
      const uint16_t reg_val =
          (static_cast<uint16_t>(data[9 + i]) << 8) | data[10 + i];
      resp.register_values.push_back(reg_val);
    }
    resp.success = true;
  } else if (resp.function_code == 0x05 || resp.function_code == 0x06) {
    // Write Single Coil / Register: Echo response
    resp.success = true;
  } else {
    resp.error_message = "unhandled function code: " + std::to_string(resp.function_code);
  }

  return resp;
}

ModbusTcpClient::ModbusTcpClient(std::string ip, uint16_t port, uint8_t unit_id,
                                 int timeout_ms)
    : ip_(std::move(ip)),
      port_(port),
      unit_id_(unit_id),
      timeout_ms_(timeout_ms) {}

ModbusTcpClient::~ModbusTcpClient() {
  Disconnect();
}

bool ModbusTcpClient::Connect(std::string* error_msg) {
  Disconnect();

  sock_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
  if (sock_fd_ < 0) {
    if (error_msg) *error_msg = "failed to create socket";
    return false;
  }

  // 设置套接字超时
  struct timeval tv;
  tv.tv_sec = timeout_ms_ / 1000;
  tv.tv_usec = (timeout_ms_ % 1000) * 1000;
  ::setsockopt(sock_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  ::setsockopt(sock_fd_, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

  struct sockaddr_in serv_addr;
  std::memset(&serv_addr, 0, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_port = htons(port_);

  if (::inet_pton(AF_INET, ip_.c_str(), &serv_addr.sin_addr) <= 0) {
    if (error_msg) *error_msg = "invalid IP address: " + ip_;
    Disconnect();
    return false;
  }

  if (::connect(sock_fd_, reinterpret_cast<struct sockaddr*>(&serv_addr), sizeof(serv_addr)) < 0) {
    if (error_msg) *error_msg = "connect to " + ip_ + ":" + std::to_string(port_) + " failed";
    Disconnect();
    return false;
  }

  return true;
}

void ModbusTcpClient::Disconnect() {
  if (sock_fd_ >= 0) {
    ::close(sock_fd_);
    sock_fd_ = -1;
  }
}

bool ModbusTcpClient::IsConnected() const {
  return sock_fd_ >= 0;
}

ModbusResponse ModbusTcpClient::Transceive(const std::vector<uint8_t>& request, uint16_t tx_id) {
  if (!IsConnected()) {
    std::string err;
    if (!Connect(&err)) {
      ModbusResponse resp;
      resp.error_message = "disconnected and reconnect failed: " + err;
      return resp;
    }
  }

  ssize_t sent = ::send(sock_fd_, request.data(), request.size(), MSG_NOSIGNAL);
  if (sent != static_cast<ssize_t>(request.size())) {
    Disconnect();
    ModbusResponse resp;
    resp.error_message = "failed to send complete Modbus packet";
    return resp;
  }

  uint8_t rx_buffer[260];
  ssize_t n_read = ::recv(sock_fd_, rx_buffer, sizeof(rx_buffer), 0);
  if (n_read <= 0) {
    Disconnect();
    ModbusResponse resp;
    resp.error_message = "recv error or connection closed by peer";
    return resp;
  }

  return ModbusPacketCodec::ParseResponse(tx_id, rx_buffer, static_cast<std::size_t>(n_read));
}

ModbusResponse ModbusTcpClient::ReadCoils(uint16_t start_addr, uint16_t count) {
  const uint16_t tx_id = next_transaction_id_++;
  auto req = ModbusPacketCodec::BuildReadCoilsRequest(tx_id, unit_id_, start_addr, count);
  return Transceive(req, tx_id);
}

ModbusResponse ModbusTcpClient::ReadHoldingRegisters(uint16_t start_addr, uint16_t count) {
  const uint16_t tx_id = next_transaction_id_++;
  auto req = ModbusPacketCodec::BuildReadHoldingRegistersRequest(tx_id, unit_id_, start_addr, count);
  return Transceive(req, tx_id);
}

ModbusResponse ModbusTcpClient::WriteSingleCoil(uint16_t coil_addr, bool value) {
  const uint16_t tx_id = next_transaction_id_++;
  auto req = ModbusPacketCodec::BuildWriteSingleCoilRequest(tx_id, unit_id_, coil_addr, value);
  return Transceive(req, tx_id);
}

ModbusResponse ModbusTcpClient::WriteSingleRegister(uint16_t reg_addr, uint16_t value) {
  const uint16_t tx_id = next_transaction_id_++;
  auto req = ModbusPacketCodec::BuildWriteSingleRegisterRequest(tx_id, unit_id_, reg_addr, value);
  return Transceive(req, tx_id);
}

}  // namespace amr_dispatcher_core::facility
