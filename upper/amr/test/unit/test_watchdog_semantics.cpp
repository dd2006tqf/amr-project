// T9：安全门心跳阈值的语义必须是"名字说的那样"。
//
// 修复前的实际行为：连续丢 2 拍心跳（5Hz 下约 400ms）就锁存急停，而
// soft_timeout_ms=2000 名义上是"软超时"却根本轮不到——因为 safety_gate
// 用 events.empty() 一刀切，丢 1 拍产生的 warn 事件也被算作"不健康"，
// FaultSupervisor 见到 !watchdog_healthy 直接给 kEmergencyStop。
//
// 台架现场后果：USB/DDS 抖一下车就假停，且是**锁存**的，必须人工干预。
//
// 这组测试锁定的契约：
//   warn（丢拍）      -> 限速 30%，**不**急停，**不**锁存
//   critical（软超时）-> 急停锁存
//   从未收到心跳      -> 急停（启动期就该发现上游不在）
#include <gtest/gtest.h>

#include <chrono>
#include <thread>

#include "amr_dispatcher_core/safety/fault_supervisor.hpp"
#include "amr_dispatcher_core/safety/watchdog.hpp"

using namespace amr_dispatcher_core::safety;
using namespace std::chrono_literals;

namespace {

// 跑够启动宽限期再上报，避免 kArming 干扰断言。
FaultSupervisor MakeSupervisor(std::chrono::milliseconds staleness = 2000ms) {
  SupervisorConfig cfg;
  cfg.startup_grace = 0ms;
  cfg.command_cooldown = 0ms;
  cfg.auto_clear = true;
  cfg.staleness_timeout = staleness;
  return FaultSupervisor(cfg);
}

}  // namespace

// 丢拍（watchdog_degraded=true，watchdog_healthy=true）只限速。
TEST(WatchdogSemanticsTest, HeartbeatDegradedClampsSpeedButDoesNotEStop) {
  auto supervisor = MakeSupervisor();
  MultiSourceHealthReport r;
  r.watchdog_healthy = true;     // 未到软超时
  r.watchdog_degraded = true;    // 但已经丢拍
  r.detail_message = "heartbeat degraded";
  supervisor.UpdateMultiSourceHealth(r);
  supervisor.Tick();

  EXPECT_EQ(supervisor.state(), SupervisorState::kDegraded);
  EXPECT_EQ(supervisor.recommendation(), SystemActionRecommendation::kClampSpeed);
  EXPECT_FALSE(supervisor.emergency_stop_requested());
}

// 真失联（watchdog_healthy=false）必须急停——这条是原有的安全底线，不能被放宽。
TEST(WatchdogSemanticsTest, RealLossStillRequestsEmergencyStop) {
  auto supervisor = MakeSupervisor();
  MultiSourceHealthReport r;
  r.watchdog_healthy = false;
  r.detail_message = "heartbeat soft timeout";
  supervisor.UpdateMultiSourceHealth(r);
  supervisor.Tick();

  EXPECT_EQ(supervisor.state(), SupervisorState::kFault);
  EXPECT_EQ(supervisor.recommendation(), SystemActionRecommendation::kEmergencyStop);
  EXPECT_TRUE(supervisor.emergency_stop_requested());
}

// 丢拍恢复后必须能自己走回 NORMAL，不能留下任何锁存痕迹。
// 修复前这里会卡在 FAULT，因为丢拍已经被升级成急停并锁存。
TEST(WatchdogSemanticsTest, DegradedRecoversToNormalWithoutLatch) {
  auto supervisor = MakeSupervisor();
  MultiSourceHealthReport degraded;
  degraded.watchdog_degraded = true;
  supervisor.UpdateMultiSourceHealth(degraded);
  supervisor.Tick();
  ASSERT_EQ(supervisor.state(), SupervisorState::kDegraded);
  ASSERT_FALSE(supervisor.emergency_stop_requested());

  MultiSourceHealthReport healthy;  // 全默认 = 健康
  supervisor.UpdateMultiSourceHealth(healthy);
  supervisor.Tick();
  EXPECT_EQ(supervisor.state(), SupervisorState::kNormal);
  EXPECT_EQ(supervisor.recommendation(), SystemActionRecommendation::kNone);
  EXPECT_FALSE(supervisor.emergency_stop_requested());
}

// 看门狗原始语义：超过 heartbeat_timeout 但未到 soft_timeout 只出 warn 事件，
// 超过 soft_timeout 才出 critical。这两个级别是 T9 分流的事实依据。
TEST(WatchdogSemanticsTest, InspectSeparatesWarnFromCritical) {
  SafetyWatchdog w({.heartbeat_timeout = 20ms, .soft_timeout = 200ms});
  w.FeedHeartbeat("chassis_driver");
  std::this_thread::sleep_for(60ms);  // 超过 warn 阈值，远未到 soft

  const auto events = w.Inspect({"chassis_driver"});
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events.front().severity, SafetySeverity::kWarn);
}

// 从未喂过心跳的源必须直接 critical —— 启动时上游根本不在，不能等 soft_timeout。
TEST(WatchdogSemanticsTest, NeverSeenNodeIsCriticalNotWarn) {
  SafetyWatchdog w({.heartbeat_timeout = 20ms, .soft_timeout = 2000ms});
  const auto events = w.Inspect({"never_started"});
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events.front().severity, SafetySeverity::kCritical);
}
