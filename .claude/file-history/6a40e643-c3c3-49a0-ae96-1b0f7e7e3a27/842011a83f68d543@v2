#include "amr_dispatcher_core/workflow/fleet_link_guard.hpp"

#include <gtest/gtest.h>

using namespace amr_dispatcher_core::workflow;

namespace {

VehicleLinkInput MakeVehicle(const std::string& id, bool online, bool quarantined = false,
                             bool seen_state = true, double age = 0.0) {
  VehicleLinkInput v;
  v.vehicle_id = id;
  v.online = online;
  v.quarantined = quarantined;
  v.seen_state = seen_state;
  v.state_age_sec = age;
  return v;
}

FleetLinkGuardRequest MakeRequest(std::vector<VehicleLinkInput> vehicles,
                                  double stale_sec = 5.0) {
  FleetLinkGuardRequest r;
  r.vehicles = std::move(vehicles);
  r.state_stale_sec = stale_sec;
  return r;
}

}  // namespace

TEST(FleetLinkGuardTest, AllOnlineFreshDoesNotBlock) {
  const auto decision = EvaluateFleetLinkGuard(
      MakeRequest({MakeVehicle("v1", true), MakeVehicle("v2", true)}));
  EXPECT_FALSE(decision.block_new_dispatch);
  EXPECT_TRUE(decision.retain_all_locks);  // 铁律：永不释放
  ASSERT_EQ(decision.verdicts.size(), 2u);
  for (const auto& v : decision.verdicts) {
    EXPECT_EQ(v.kind, VehicleLinkVerdict::Kind::kHealthy);
  }
}

TEST(FleetLinkGuardTest, OfflineVehicleBlocksDispatch) {
  const auto decision =
      EvaluateFleetLinkGuard(MakeRequest({MakeVehicle("v1", true), MakeVehicle("v2", false)}));
  EXPECT_TRUE(decision.block_new_dispatch);
  EXPECT_TRUE(decision.retain_all_locks);
  ASSERT_EQ(decision.verdicts.size(), 2u);
  EXPECT_EQ(decision.verdicts[0].kind, VehicleLinkVerdict::Kind::kHealthy);
  EXPECT_EQ(decision.verdicts[1].kind, VehicleLinkVerdict::Kind::kSuspendDispatch);
  // 告警文案必须带 Spec 指引（e2e 与运维手册都引用它）
  EXPECT_NE(decision.verdicts[1].message.find("Spec 5.2"), std::string::npos);
}

// 核心回归：隔离车的离线是"已知且已处置"，不得冻结全队——
// 否则人工强清之后车队永远派不出单，隔离就失去意义。
TEST(FleetLinkGuardTest, QuarantinedOfflineVehicleDoesNotBlock) {
  const auto decision = EvaluateFleetLinkGuard(
      MakeRequest({MakeVehicle("v1", true), MakeVehicle("v2", false, /*quarantined=*/true)}));
  EXPECT_FALSE(decision.block_new_dispatch);
  EXPECT_TRUE(decision.retain_all_locks);
  ASSERT_EQ(decision.verdicts.size(), 2u);
  EXPECT_EQ(decision.verdicts[1].kind, VehicleLinkVerdict::Kind::kSkip);
  EXPECT_TRUE(decision.verdicts[1].message.empty());
}

TEST(FleetLinkGuardTest, StaleStateWarnsButDoesNotBlock) {
  // 在线但 state 陈旧：陈旧不是释放依据，也不是派发依据（Spec §5.3）
  const auto decision = EvaluateFleetLinkGuard(
      MakeRequest({MakeVehicle("v1", true, false, true, /*age=*/9.0)}, /*stale_sec=*/5.0));
  EXPECT_FALSE(decision.block_new_dispatch);
  EXPECT_TRUE(decision.retain_all_locks);
  ASSERT_EQ(decision.verdicts.size(), 1u);
  EXPECT_EQ(decision.verdicts[0].kind, VehicleLinkVerdict::Kind::kStaleWarning);
  EXPECT_NE(decision.verdicts[0].message.find("Spec 5.3"), std::string::npos);
}

TEST(FleetLinkGuardTest, StaleBoundaryIsExclusive) {
  // age == threshold 不算陈旧（与原实现 age > threshold 一致）
  const auto at_boundary = EvaluateFleetLinkGuard(
      MakeRequest({MakeVehicle("v1", true, false, true, 5.0)}, 5.0));
  EXPECT_EQ(at_boundary.verdicts[0].kind, VehicleLinkVerdict::Kind::kHealthy);
  const auto past_boundary = EvaluateFleetLinkGuard(
      MakeRequest({MakeVehicle("v1", true, false, true, 5.001)}, 5.0));
  EXPECT_EQ(past_boundary.verdicts[0].kind, VehicleLinkVerdict::Kind::kStaleWarning);
}

TEST(FleetLinkGuardTest, NeverSeenStateIsNotTreatedAsStale) {
  // 从未收到 state 的在线车：没有"多久没更新"可比，不能报 stale
  const auto decision = EvaluateFleetLinkGuard(
      MakeRequest({MakeVehicle("v1", true, false, /*seen_state=*/false, /*age=*/999.0)}, 5.0));
  EXPECT_FALSE(decision.block_new_dispatch);
  EXPECT_EQ(decision.verdicts[0].kind, VehicleLinkVerdict::Kind::kHealthy);
}

TEST(FleetLinkGuardTest, FirstBlockingVehicleIsReportedAlongsideOthers) {
  // 多台离线时每台都有自己的判定（运维能看到"谁把全队拖住了"）
  const auto decision = EvaluateFleetLinkGuard(MakeRequest(
      {MakeVehicle("v1", false), MakeVehicle("v2", false), MakeVehicle("v3", true)}));
  EXPECT_TRUE(decision.block_new_dispatch);
  ASSERT_EQ(decision.verdicts.size(), 3u);
  EXPECT_EQ(decision.verdicts[0].kind, VehicleLinkVerdict::Kind::kSuspendDispatch);
  EXPECT_EQ(decision.verdicts[1].kind, VehicleLinkVerdict::Kind::kSuspendDispatch);
  EXPECT_EQ(decision.verdicts[2].kind, VehicleLinkVerdict::Kind::kHealthy);
}

TEST(FleetLinkGuardTest, EmptyFleetDoesNotBlock) {
  const auto decision = EvaluateFleetLinkGuard(MakeRequest({}));
  EXPECT_FALSE(decision.block_new_dispatch);
  EXPECT_TRUE(decision.retain_all_locks);
  EXPECT_TRUE(decision.verdicts.empty());
}

TEST(FleetLinkGuardTest, RetainAllLocksIsAlwaysTrue) {
  // 铁律回归：无论链路状态组合如何，守卫都不得产出释放动作
  const std::vector<std::vector<VehicleLinkInput>> scenarios{
      {MakeVehicle("v1", false)},
      {MakeVehicle("v1", true, false, true, 100.0)},
      {MakeVehicle("v1", false, true)},
      {MakeVehicle("v1", false), MakeVehicle("v2", true, false, true, 100.0)},
  };
  for (const auto& vehicles : scenarios) {
    const auto decision = EvaluateFleetLinkGuard(MakeRequest(vehicles));
    EXPECT_TRUE(decision.retain_all_locks);
  }
}

TEST(FleetLinkGuardTest, VerdictOrderFollowsInputOrder) {
  // 输出顺序必须与输入一致：日志顺序随哈希序抖动会让 e2e 断言变得不可靠
  const auto decision = EvaluateFleetLinkGuard(
      MakeRequest({MakeVehicle("zeta", true), MakeVehicle("alpha", false),
                   MakeVehicle("mid", true, false, true, 99.0)}));
  ASSERT_EQ(decision.verdicts.size(), 3u);
  EXPECT_EQ(decision.verdicts[0].vehicle_id, "zeta");
  EXPECT_EQ(decision.verdicts[1].vehicle_id, "alpha");
  EXPECT_EQ(decision.verdicts[2].vehicle_id, "mid");
}
