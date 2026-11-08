#include "amr_dispatcher_core/fleet/vehicle_selection_strategy.hpp"

#include <gtest/gtest.h>

using amr_dispatcher_core::fleet::kSelectionBatteryPenaltyWeight;
using amr_dispatcher_core::fleet::kSelectionBatteryReferencePercent;
using amr_dispatcher_core::fleet::ParseVehicleSelectionStrategy;
using amr_dispatcher_core::fleet::RankVehiclesForSelection;
using amr_dispatcher_core::fleet::VehicleRankEntry;
using amr_dispatcher_core::fleet::VehicleSelectionStrategy;
using amr_dispatcher_core::fleet::VehicleSelectionStrategyLabel;

constexpr auto kNearest = VehicleSelectionStrategy::kNearest;
constexpr auto kBatteryPreferred = VehicleSelectionStrategy::kBatteryPreferred;

namespace {

std::vector<VehicleRankEntry> TwoVehicles(double near_battery, double far_battery) {
  return {VehicleRankEntry{"near", 0.0, near_battery},
          VehicleRankEntry{"far", 0.0, far_battery}};
}

}  // namespace

TEST(VehicleSelectionStrategyTest, ParseAndLabelRoundTrip) {
  VehicleSelectionStrategy parsed = kNearest;
  EXPECT_TRUE(ParseVehicleSelectionStrategy("nearest", &parsed));
  EXPECT_EQ(parsed, kNearest);
  EXPECT_TRUE(ParseVehicleSelectionStrategy("battery_preferred", &parsed));
  EXPECT_EQ(parsed, kBatteryPreferred);
  EXPECT_STREQ(VehicleSelectionStrategyLabel(kNearest), "nearest");
  EXPECT_STREQ(VehicleSelectionStrategyLabel(kBatteryPreferred), "battery_preferred");
}

// 未知标签绝不猜测：调用方据此回落到默认策略并告警。
TEST(VehicleSelectionStrategyTest, UnknownLabelIsRejected) {
  VehicleSelectionStrategy parsed = kBatteryPreferred;
  EXPECT_FALSE(ParseVehicleSelectionStrategy("auction", &parsed));
  EXPECT_FALSE(ParseVehicleSelectionStrategy("", &parsed));
  EXPECT_FALSE(ParseVehicleSelectionStrategy("Nearest", &parsed));  // 大小写敏感
  EXPECT_EQ(parsed, kBatteryPreferred);  // 未被改写
  EXPECT_FALSE(ParseVehicleSelectionStrategy("nearest", nullptr));
}

TEST(VehicleSelectionStrategyTest, NearestPicksLowestCost) {
  const std::vector<VehicleRankEntry> vehicles{{"b", 0.0, 90.0}, {"a", 0.0, 10.0}};
  const std::unordered_map<std::string, double> costs{{"a", 5.0}, {"b", 2.0}};
  const auto ranked = RankVehiclesForSelection(kNearest, vehicles, costs);
  ASSERT_EQ(ranked.size(), 2u);
  EXPECT_EQ(ranked[0].vehicle_id, "b");
  EXPECT_DOUBLE_EQ(ranked[0].score, 2.0);
}

// nearest 策略**不看电量**：这是改动前的行为，必须逐字保持。
TEST(VehicleSelectionStrategyTest, NearestIgnoresBattery) {
  const auto ranked = RankVehiclesForSelection(
      kNearest, TwoVehicles(/*near_battery=*/5.0, /*far_battery=*/100.0),
      {{"near", 3.0}, {"far", 10.0}});
  EXPECT_EQ(ranked[0].vehicle_id, "near");
}

// 距离接近时，低电量软惩罚足以把顺序翻转过去。
TEST(VehicleSelectionStrategyTest, BatteryPreferredFlipsCloseRace) {
  // near: 3.0m @ 10%  -> penalty = (50-10)/100*0.25 = 0.10 -> 3.30
  // far:  3.2m @ 95%  -> 不打折                              -> 3.20
  const auto ranked = RankVehiclesForSelection(
      kBatteryPreferred, TwoVehicles(/*near_battery=*/10.0, /*far_battery=*/95.0),
      {{"near", 3.0}, {"far", 3.2}});
  ASSERT_EQ(ranked.size(), 2u);
  EXPECT_EQ(ranked[0].vehicle_id, "far");
  EXPECT_DOUBLE_EQ(ranked[0].score, 3.2);
  EXPECT_NEAR(ranked[1].score, 3.30, 1e-9);
}

// 惩罚有上限：明显更近的车不能输给远处满电车（不为了省电牺牲吞吐）。
TEST(VehicleSelectionStrategyTest, BatteryPreferredPenaltyIsBounded) {
  const auto ranked = RankVehiclesForSelection(
      kBatteryPreferred, TwoVehicles(/*near_battery=*/1.0, /*far_battery=*/100.0),
      {{"near", 3.0}, {"far", 100.0}});
  EXPECT_EQ(ranked[0].vehicle_id, "near");
  // 上界：3.0 * (1 + 0.5*0.25) = 3.375，仍远小于 100
  const double worst = 3.0 * (1.0 + (kSelectionBatteryReferencePercent - 1.0) / 100.0 *
                                       kSelectionBatteryPenaltyWeight);
  EXPECT_NEAR(ranked[0].score, worst, 1e-9);
  EXPECT_LT(ranked[0].score, 4.0);
}

// 参考电量之上不再加分：满电车之间只比距离（避免"电量 99 胜过 95"这种噪声）。
TEST(VehicleSelectionStrategyTest, AboveReferenceBatteryGetsNoBonus) {
  const std::vector<VehicleRankEntry> vehicles{{"hi", 0.0, 100.0}, {"lo", 0.0, 60.0}};
  const auto ranked = RankVehiclesForSelection(kBatteryPreferred, vehicles,
                                               {{"hi", 5.0}, {"lo", 4.0}});
  EXPECT_EQ(ranked[0].vehicle_id, "lo");  // 距离优先，两者都不打折
  EXPECT_DOUBLE_EQ(ranked[0].score, 4.0);
  EXPECT_DOUBLE_EQ(ranked[1].score, 5.0);
}

// 同分必须由 id 决定，否则容器遍历序会让每次结果不同。
TEST(VehicleSelectionStrategyTest, TiesBreakOnVehicleId) {
  const std::vector<VehicleRankEntry> vehicles{{"zeta", 0.0, 90.0}, {"alpha", 0.0, 90.0},
                                               {"mid", 0.0, 90.0}};
  const auto ranked = RankVehiclesForSelection(
      kNearest, vehicles, {{"zeta", 4.0}, {"alpha", 4.0}, {"mid", 4.0}});
  ASSERT_EQ(ranked.size(), 3u);
  EXPECT_EQ(ranked[0].vehicle_id, "alpha");
  EXPECT_EQ(ranked[1].vehicle_id, "mid");
  EXPECT_EQ(ranked[2].vehicle_id, "zeta");
}

// 成本缺失的候选不参与排序（调用方据此判定"无路可达"）。
TEST(VehicleSelectionStrategyTest, MissingCostIsDropped) {
  const std::vector<VehicleRankEntry> vehicles{{"a", 0.0, 50.0}, {"b", 0.0, 50.0}};
  const auto ranked = RankVehiclesForSelection(kBatteryPreferred, vehicles, {{"a", 1.0}});
  ASSERT_EQ(ranked.size(), 1u);
  EXPECT_EQ(ranked[0].vehicle_id, "a");
}

TEST(VehicleSelectionStrategyTest, EmptyInputYieldsEmptyRanking) {
  EXPECT_TRUE(RankVehiclesForSelection(kBatteryPreferred, {}, {}).empty());
}

TEST(VehicleSelectionStrategyTest, AllCostsMissingYieldsEmptyRanking) {
  const std::vector<VehicleRankEntry> vehicles{{"a", 0.0, 50.0}};
  EXPECT_TRUE(RankVehiclesForSelection(kNearest, vehicles, {}).empty());
}
