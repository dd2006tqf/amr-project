#include "amr_dispatcher_core/dispatcher/mission_cost_estimator.hpp"
#include "amr_dispatcher_core/dispatcher/dock_catalog.hpp"
#include "amr_dispatcher_core/path_tracking/geometry_utils.hpp"
#include "amr_dispatcher_core/safety/fault_supervisor.hpp"

#include <gtest/gtest.h>
#include <thread>

using namespace amr_dispatcher_core::dispatcher;
using namespace amr_dispatcher_core::path_tracking;
using namespace amr_dispatcher_core::safety;
using namespace std::chrono_literals;

// 1. 几何与角度测试
TEST(GeometryUtilsTest, NormalizesAngleWrapAround) {
  EXPECT_NEAR(NormalizeAngle(0.0), 0.0, 1e-6);
  EXPECT_NEAR(NormalizeAngle(3.0 * M_PI), M_PI, 1e-6);
  EXPECT_NEAR(NormalizeAngle(-3.0 * M_PI), -M_PI, 1e-6);
  EXPECT_NEAR(AngleDifference(0.1, -0.1), 0.2, 1e-6);
}

TEST(GeometryUtilsTest, Quaternion2DYawRoundTrip) {
  double yaw = 1.25;
  auto q = QuaternionFromYaw(yaw);
  double recovered = YawFromQuaternion(q.z, q.w);
  EXPECT_NEAR(recovered, yaw, 1e-6);
}

// 2. 成本与电量估算测试
TEST(MissionCostEstimatorTest, CalculatesTravelTimeAndBatteryDrop) {
  CostEstimatorConfig cfg;
  cfg.nominal_linear_speed_mps = 1.0;
  cfg.battery_drop_per_meter_v = 0.01;
  cfg.minimum_safe_voltage_v = 22.0;
  cfg.station_stop_penalty_sec = 2.0;

  MissionCostEstimator estimator(cfg);

  // 100 米，3 个停靠点，当前电压 24V
  auto est = estimator.Estimate(100.0, 3, 24.0);
  EXPECT_DOUBLE_EQ(est.distance_m, 100.0);
  EXPECT_DOUBLE_EQ(est.eta_seconds, 100.0 + (3 - 1) * 2.0);  // 104 秒
  EXPECT_DOUBLE_EQ(est.estimated_battery_drop_v, 1.0);       // 100 * 0.01 = 1V
  EXPECT_DOUBLE_EQ(est.projected_battery_voltage, 23.0);
  EXPECT_TRUE(est.battery_sufficient);

  // 极低电量测试
  auto low_est = estimator.Estimate(100.0, 1, 22.5);
  EXPECT_FALSE(low_est.battery_sufficient);  // 22.5 - 1.0 = 21.5V < 22.0V
}

// 3. 充电桩管理测试
TEST(DockCatalogTest, ManagesChargingStations) {
  DockCatalog catalog;
  ChargingDock d1{"dock_01", "st_charge_1", "st_approach_1", "lock_chg_1", true, false, ""};
  catalog.AddDock(d1);

  ASSERT_NE(catalog.FindDock("dock_01"), nullptr);
  EXPECT_EQ(catalog.FindAvailableDock()->id, "dock_01");

  EXPECT_TRUE(catalog.OccupyDock("dock_01", "robot_01"));
  EXPECT_EQ(catalog.FindAvailableDock(), nullptr);  // 已被占用，无可用

  EXPECT_TRUE(catalog.ReleaseDock("dock_01"));
  EXPECT_NE(catalog.FindAvailableDock(), nullptr);  // 释放后恢复可用
}

// 4. 故障监督状态机测试
TEST(FaultSupervisorTest, TransitionsThroughGraceAndFault) {
  SupervisorConfig cfg;
  cfg.startup_grace = 100ms;
  cfg.command_cooldown = 10ms;
  cfg.auto_clear = true;

  FaultSupervisor supervisor(cfg);
  supervisor.Tick();
  EXPECT_EQ(supervisor.state(), SupervisorState::kWaiting);

  // 启动宽限期内出现错误 -> 处于 ARMING
  supervisor.UpdateHealth(false, "sensor warm-up");
  supervisor.Tick();
  EXPECT_EQ(supervisor.state(), SupervisorState::kArming);

  // 超出启动宽限期 -> 触发 FAULT 与急停请求
  std::this_thread::sleep_for(120ms);
  supervisor.Tick();
  EXPECT_EQ(supervisor.state(), SupervisorState::kFault);
  EXPECT_TRUE(supervisor.emergency_stop_requested());

  // 恢复健康 -> 自愈恢复并清除急停
  supervisor.UpdateHealth(true, "healthy");
  supervisor.Tick();
  EXPECT_EQ(supervisor.state(), SupervisorState::kRecovery);
  EXPECT_FALSE(supervisor.emergency_stop_requested());

  supervisor.Tick();
  EXPECT_EQ(supervisor.state(), SupervisorState::kNormal);
}

TEST(FaultSupervisorTest, MultiSourceArbitrationRecommendations) {
  SupervisorConfig cfg;
  cfg.startup_grace = 50ms;
  cfg.command_cooldown = 10ms;
  cfg.auto_clear = true;
  cfg.degraded_loss_rate_threshold = 0.15;
  cfg.critical_loss_rate_threshold = 0.35;

  FaultSupervisor supervisor(cfg);
  std::this_thread::sleep_for(60ms);

  // 1. 中度丢包率 20% -> 建议限速 ClampSpeed
  MultiSourceHealthReport r1;
  r1.chassis_healthy = false;
  r1.chassis_loss_rate = 0.20;
  supervisor.UpdateMultiSourceHealth(r1);
  supervisor.Tick();
  EXPECT_EQ(supervisor.state(), SupervisorState::kDegraded);
  EXPECT_EQ(supervisor.recommendation(), SystemActionRecommendation::kClampSpeed);

  // 2. 严重丢包率 40% -> 触发 FAULT 且建议 EmergencyStop
  MultiSourceHealthReport r2;
  r2.chassis_loss_rate = 0.40;
  supervisor.UpdateMultiSourceHealth(r2);
  supervisor.Tick();
  EXPECT_EQ(supervisor.state(), SupervisorState::kFault);
  EXPECT_EQ(supervisor.recommendation(), SystemActionRecommendation::kEmergencyStop);
  EXPECT_TRUE(supervisor.emergency_stop_requested());

  // 3. 死锁检测激活 -> 建议 PauseDispatch 暂缓接单
  MultiSourceHealthReport r3;
  r3.deadlock_detected = true;
  supervisor.UpdateMultiSourceHealth(r3);
  supervisor.Tick();
  EXPECT_EQ(supervisor.state(), SupervisorState::kDegraded);
  EXPECT_EQ(supervisor.recommendation(), SystemActionRecommendation::kPauseDispatch);
}
