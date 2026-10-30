#include "amr_dispatcher_core/chassis/battery_voltage.hpp"

#include <gtest/gtest.h>

using amr_dispatcher_core::chassis::BatteryVoltageToPercent;

namespace {

// 12V 台架端点：满 12.6V、空 11.0V。
constexpr double kFull12V = 12.6;
constexpr double kEmpty12V = 11.0;

}  // namespace

TEST(BatteryVoltageTest, FullAndEmptyClampToEndpoints) {
  EXPECT_DOUBLE_EQ(BatteryVoltageToPercent(kFull12V, kFull12V, kEmpty12V), 100.0);
  EXPECT_DOUBLE_EQ(BatteryVoltageToPercent(kEmpty12V, kFull12V, kEmpty12V), 0.0);
}

TEST(BatteryVoltageTest, MidpointIsHalf) {
  const double mid = (kFull12V + kEmpty12V) / 2.0;
  EXPECT_NEAR(BatteryVoltageToPercent(mid, kFull12V, kEmpty12V), 50.0, 1e-9);
}

TEST(BatteryVoltageTest, AboveFullAndBelowEmptyAreClamped) {
  EXPECT_DOUBLE_EQ(BatteryVoltageToPercent(14.0, kFull12V, kEmpty12V), 100.0);
  EXPECT_DOUBLE_EQ(BatteryVoltageToPercent(9.0, kFull12V, kEmpty12V), 0.0);
}

TEST(BatteryVoltageTest, MissingVoltageReportsZero) {
  // 底盘没上报电压时不能凭空给出"满电"，否则下游 min_battery_percent 永远放行。
  EXPECT_DOUBLE_EQ(BatteryVoltageToPercent(0.0, kFull12V, kEmpty12V), 0.0);
  EXPECT_DOUBLE_EQ(BatteryVoltageToPercent(-1.0, kFull12V, kEmpty12V), 0.0);
}

TEST(BatteryVoltageTest, InvertedEndpointsDegradeToBinary) {
  // 配置写反（满 < 空）不能让结果变 NaN 或负值，退化为 0/100 两档。
  EXPECT_DOUBLE_EQ(BatteryVoltageToPercent(12.0, kEmpty12V, kFull12V), 0.0);
  EXPECT_DOUBLE_EQ(BatteryVoltageToPercent(13.0, kEmpty12V, kFull12V), 100.0);
}

TEST(BatteryVoltageTest, TwentyFourVoltFleetEndpoint) {
  // 24V 整车：25.2 / 21.0。21.0V 恰好是 dispatcher 的默认最低可用电压。
  EXPECT_DOUBLE_EQ(BatteryVoltageToPercent(25.2, 25.2, 21.0), 100.0);
  EXPECT_DOUBLE_EQ(BatteryVoltageToPercent(21.0, 25.2, 21.0), 0.0);
  EXPECT_NEAR(BatteryVoltageToPercent(23.1, 25.2, 21.0), 50.0, 1e-9);
}
