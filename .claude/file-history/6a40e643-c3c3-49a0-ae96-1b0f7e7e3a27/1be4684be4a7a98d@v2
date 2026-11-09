#include "amr_dispatcher_core/facility/modbus_tcp_client.hpp"

#include <gtest/gtest.h>

using amr_dispatcher_core::facility::ModbusPacketCodec;
using amr_dispatcher_core::facility::ModbusResponse;

TEST(ModbusPacketCodecTest, BuildReadCoilsRequest) {
  const auto req = ModbusPacketCodec::BuildReadCoilsRequest(0x0102, 1, 0x000A, 5);
  ASSERT_EQ(req.size(), 12u);
  EXPECT_EQ(req[0], 0x01);  // Tx ID MSB
  EXPECT_EQ(req[1], 0x02);  // Tx ID LSB
  EXPECT_EQ(req[2], 0x00);  // Proto MSB
  EXPECT_EQ(req[3], 0x00);  // Proto LSB
  EXPECT_EQ(req[4], 0x00);  // Length MSB
  EXPECT_EQ(req[5], 0x06);  // Length LSB (6 bytes follow)
  EXPECT_EQ(req[6], 0x01);  // Unit ID
  EXPECT_EQ(req[7], 0x01);  // Func Code (Read Coils)
  EXPECT_EQ(req[8], 0x00);  // Start Addr MSB
  EXPECT_EQ(req[9], 0x0A);  // Start Addr LSB
  EXPECT_EQ(req[10], 0x00); // Count MSB
  EXPECT_EQ(req[11], 0x05); // Count LSB
}

TEST(ModbusPacketCodecTest, BuildWriteSingleCoilRequest) {
  const auto req = ModbusPacketCodec::BuildWriteSingleCoilRequest(0x0001, 1, 0x0010, true);
  ASSERT_EQ(req.size(), 12u);
  EXPECT_EQ(req[7], 0x05);  // Func Code
  EXPECT_EQ(req[8], 0x00);
  EXPECT_EQ(req[9], 0x10);  // Coil addr 16
  EXPECT_EQ(req[10], 0xFF); // ON = 0xFF00
  EXPECT_EQ(req[11], 0x00);
}

TEST(ModbusPacketCodecTest, ParseReadCoilsResponse) {
  // MBAP (7 bytes) + Func 0x01 + ByteCount 1 + Data (0x05: bit0=1, bit1=0, bit2=1)
  const uint8_t raw[] = {
      0x01, 0x02,             // Tx ID = 0x0102
      0x00, 0x00,             // Proto
      0x00, 0x04,             // Length = 4
      0x01,                   // Unit ID
      0x01,                   // Func Read Coils
      0x01,                   // Byte Count
      0x05                    // Data: 00000101 (coil 0 = true, coil 1 = false, coil 2 = true)
  };

  const auto resp = ModbusPacketCodec::ParseResponse(0x0102, raw, sizeof(raw));
  EXPECT_TRUE(resp.success);
  EXPECT_EQ(resp.function_code, 0x01);
  ASSERT_GE(resp.coil_values.size(), 3u);
  EXPECT_TRUE(resp.coil_values[0]);
  EXPECT_FALSE(resp.coil_values[1]);
  EXPECT_TRUE(resp.coil_values[2]);
}

TEST(ModbusPacketCodecTest, ParseReadHoldingRegistersResponse) {
  // MBAP (7 bytes) + Func 0x03 + ByteCount 2 + Reg (0x0003 -> Floor 3)
  const uint8_t raw[] = {
      0x00, 0x05,             // Tx ID = 5
      0x00, 0x00,
      0x00, 0x05,             // Length = 5
      0x01,
      0x03,                   // Func Read Holding Registers
      0x02,                   // Byte count = 2
      0x00, 0x03              // Register value = 3
  };

  const auto resp = ModbusPacketCodec::ParseResponse(5, raw, sizeof(raw));
  EXPECT_TRUE(resp.success);
  ASSERT_EQ(resp.register_values.size(), 1u);
  EXPECT_EQ(resp.register_values[0], 3);
}

TEST(ModbusPacketCodecTest, ParseExceptionResponse) {
  const uint8_t raw[] = {
      0x00, 0x02,
      0x00, 0x00,
      0x00, 0x03,
      0x01,
      0x81,                   // Error code: 0x01 | 0x80
      0x02                    // Exception: Illegal Data Address
  };

  const auto resp = ModbusPacketCodec::ParseResponse(2, raw, sizeof(raw));
  EXPECT_FALSE(resp.success);
  EXPECT_EQ(resp.exception_code, 0x02);
  EXPECT_NE(resp.error_message.find("exception code: 2"), std::string::npos);
}
