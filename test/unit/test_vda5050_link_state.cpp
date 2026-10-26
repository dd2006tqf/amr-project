#include "amr_dispatcher_core/fleet/vda5050_link_state.hpp"

#include <gtest/gtest.h>

using namespace amr_dispatcher_core::fleet;
using Clock = std::chrono::steady_clock;

namespace {

Vda5050StateObservation MakeObservation(double x, double y, double battery,
                                        bool position_initialized = true) {
  Vda5050StateObservation o;
  o.position_initialized = position_initialized;
  o.x = x;
  o.y = y;
  o.battery_percent = battery;
  return o;
}

}  // namespace

TEST(Vda5050LinkStateTest, DefaultsAreOptimisticOnline) {
  const Vda5050LinkState state;
  // 乐观初值：online=false 到达前不阻塞派发（与网关 transient_local 重放收敛）
  EXPECT_TRUE(state.online);
  EXPECT_FALSE(state.seen_state);
  EXPECT_FALSE(state.position_valid);
  EXPECT_FALSE(state.quarantined);
  EXPECT_DOUBLE_EQ(state.battery_percent, 100.0);
}

TEST(Vda5050LinkStateTest, StateObservationUpdatesAllFields) {
  Vda5050LinkState state;
  const auto now = Clock::now();
  EXPECT_EQ(ApplyStateObservation(&state, MakeObservation(1.5, -2.0, 77.5), now),
            LinkStateApplyAction::kUpdated);
  EXPECT_TRUE(state.seen_state);
  EXPECT_EQ(state.last_state_time, now);
  EXPECT_TRUE(state.position_valid);
  EXPECT_DOUBLE_EQ(state.position_x, 1.5);
  EXPECT_DOUBLE_EQ(state.position_y, -2.0);
  EXPECT_DOUBLE_EQ(state.battery_percent, 77.5);
}

TEST(Vda5050LinkStateTest, PositionUninitializedMarksInvalid) {
  Vda5050LinkState state;
  ApplyStateObservation(&state, MakeObservation(0.0, 0.0, 50.0,
                                                /*position_initialized=*/false),
                        Clock::now());
  EXPECT_FALSE(state.position_valid);  // 位置不可用 -> 选车不合格
}

// 核心回归（对应 e2e Stage 6.4 当年靠 kill -9 真车才复现的缺陷）：
// 离线期间收到的 state 不得把链路回置成在线。
TEST(Vda5050LinkStateTest, StateWhileOfflineNeverFlipsOnlineBack) {
  Vda5050LinkState state;
  ApplyOnlineReport(&state, false);
  ASSERT_FALSE(state.online);

  const auto action = ApplyStateObservation(&state, MakeObservation(3.0, 4.0, 60.0),
                                            Clock::now());
  EXPECT_EQ(action, LinkStateApplyAction::kUpdatedWhileOffline);
  EXPECT_FALSE(state.online);  // 铁律：仍为离线，派发仍挂起
  // 但数据照常更新：位置/电量是重新核对物理占位的输入
  EXPECT_DOUBLE_EQ(state.position_x, 3.0);
  EXPECT_TRUE(state.seen_state);
}

TEST(Vda5050LinkStateTest, OnlineReportReportsWhetherStateFlipped) {
  Vda5050LinkState state;  // 默认 online=true
  EXPECT_TRUE(ApplyOnlineReport(&state, false));   // true -> false：翻转
  EXPECT_FALSE(state.online);
  EXPECT_FALSE(ApplyOnlineReport(&state, false));  // 重复报文：不翻转
  EXPECT_TRUE(ApplyOnlineReport(&state, true));    // 重连：翻转
  EXPECT_TRUE(state.online);
  EXPECT_FALSE(ApplyOnlineReport(&state, true));   // transient_local 重放：不翻转
}

TEST(Vda5050LinkStateTest, StateStaleOnlyWhenOnlineAndSeen) {
  Vda5050LinkState state;
  const auto now = Clock::now();
  const auto threshold = std::chrono::milliseconds(5000);

  // 从未收到 state：不算陈旧（没有"多久没更新"可比）
  EXPECT_FALSE(IsStateStale(state, now, threshold));

  ApplyStateObservation(&state, MakeObservation(0.0, 0.0, 100.0), now);
  EXPECT_FALSE(IsStateStale(state, now + std::chrono::milliseconds(4999), threshold));
  EXPECT_FALSE(IsStateStale(state, now + threshold, threshold));  // 边界不含
  EXPECT_TRUE(IsStateStale(state, now + std::chrono::milliseconds(5001), threshold));
}

TEST(Vda5050LinkStateTest, OfflineLinkIsNeverReportedStale) {
  // 离线是独立信号（走 kSuspendDispatch），不该再报 stale
  Vda5050LinkState state;
  ApplyStateObservation(&state, MakeObservation(0.0, 0.0, 100.0), Clock::now());
  ApplyOnlineReport(&state, false);
  EXPECT_FALSE(IsStateStale(state, Clock::now() + std::chrono::seconds(60),
                            std::chrono::milliseconds(5000)));
}

TEST(Vda5050LinkStateTest, NullPointerIsTolerated) {
  EXPECT_EQ(ApplyStateObservation(nullptr, MakeObservation(0.0, 0.0, 1.0), Clock::now()),
            LinkStateApplyAction::kUpdated);
  EXPECT_FALSE(ApplyOnlineReport(nullptr, false));
}

TEST(Vda5050LinkStateTest, QuarantineIsIndependentOfOnline) {
  Vda5050LinkState state;
  ApplyOnlineReport(&state, false);
  state.quarantined = true;
  // 隔离 + 离线：两者各自独立，守卫据此判定"已知且已处置"而不冻结全队
  EXPECT_TRUE(state.quarantined);
  EXPECT_FALSE(state.online);
  ApplyOnlineReport(&state, true);
  EXPECT_TRUE(state.quarantined);  // 重新上线不解除隔离（需显式恢复）
  EXPECT_TRUE(state.online);
}
