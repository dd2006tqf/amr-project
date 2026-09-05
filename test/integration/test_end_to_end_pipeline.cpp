#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "amr_dispatcher_core/chassis/backend_degradation.hpp"
#include "amr_dispatcher_core/dispatcher/deadlock_detector.hpp"
#include "amr_dispatcher_core/dispatcher/mission_queue.hpp"
#include "amr_dispatcher_core/dispatcher/recovery_policy.hpp"
#include "amr_dispatcher_core/dispatcher/traffic_reservation.hpp"
#include "amr_dispatcher_core/path_tracking/pure_pursuit.hpp"
#include "amr_dispatcher_core/safety/cmd_vel_gate.hpp"

using namespace std::chrono_literals;

namespace {

// 端到端模拟测试 1：提单 -> 策略排序出队 -> 路径规划循迹 -> 安全门控裁决 -> 模拟下发与时延统计
TEST(IntegrationPipelineTest, EndToEndDispatchToMotionControl) {
  // 1. 初始化调度器（加权评分策略）
  amr_dispatcher_core::dispatcher::MissionQueue::Config q_cfg;
  q_cfg.max_size = 10;
  q_cfg.comparator = "weighted_score";
  amr_dispatcher_core::dispatcher::MissionQueue queue(q_cfg);

  amr_dispatcher_core::dispatcher::Mission m1;
  m1.id = "mission_01";
  m1.priority = 10;
  m1.expected_seconds = 30;
  m1.sequence = 1;
  ASSERT_TRUE(queue.Push(m1).accepted);

  amr_dispatcher_core::dispatcher::Mission m2;
  m2.id = "mission_02";
  m2.priority = 20;  // 更高优先级
  m2.expected_seconds = 10;
  m2.sequence = 2;
  ASSERT_TRUE(queue.Push(m2).accepted);

  // 出队：m2 应先被调度
  auto pop = queue.PopNext();
  ASSERT_TRUE(pop.success);
  ASSERT_TRUE(pop.mission.has_value());
  EXPECT_EQ(pop.mission->id, "mission_02");

  // 2. 循迹控制器生成期望速度
  amr_dispatcher_core::path_tracking::PurePursuitController::Config pp_cfg;
  pp_cfg.lookahead_distance = 0.5;
  pp_cfg.target_speed = 1.0;
  amr_dispatcher_core::path_tracking::PurePursuitController tracker(pp_cfg);

  std::vector<amr_dispatcher_core::path_tracking::Pose2D> path = {
      {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {2.0, 0.0, 0.0}};
  tracker.SetPath(path);

  amr_dispatcher_core::path_tracking::Pose2D pose{0.0, 0.0, 0.0};
  auto control = tracker.Compute(pose);
  EXPECT_GT(control.cmd.linear_x, 0.0);

  // 3. 安全门控接收速度并进行仲裁
  amr_dispatcher_core::safety::CmdVelGateConfig gate_cfg;
  gate_cfg.max_linear_x_mps = 1.2;
  gate_cfg.source_blocked = [](const std::string&) { return false; };
  amr_dispatcher_core::safety::CmdVelGate gate(gate_cfg);

  amr_dispatcher_core::safety::CmdVelCommand raw_cmd;
  raw_cmd.linear_x_mps = control.cmd.linear_x;
  raw_cmd.angular_z_radps = control.cmd.angular_z;

  auto safe_cmd = gate.Evaluate(raw_cmd);
  EXPECT_TRUE(safe_cmd.allowed);
  EXPECT_NEAR(safe_cmd.linear_x_mps, control.cmd.linear_x, 1e-6);

  // 4. 模拟底盘通信并记录时延
  amr_dispatcher_core::chassis::LinkQualityMonitor monitor;
  monitor.RecordSuccess(12.5);  // 12.5 ms RTT
  EXPECT_TRUE(monitor.IsHealthy() || monitor.CurrentTier() == amr_dispatcher_core::chassis::LinkHealthTier::kDegraded);
}

// 端到端模拟测试 2：交通锁申请 -> 死锁诱发 -> 自愈策略仲裁 -> 资源安全释放
TEST(IntegrationPipelineTest, TrafficContentionAndDeadlockRecoveryFlow) {
  amr_dispatcher_core::dispatcher::ResourceTable traffic;

  // 机器人 1 占用边 A->B
  std::string edge_id = amr_dispatcher_core::dispatcher::ResourceTable::RouteEdgeId("A", "B");
  auto res1 = traffic.Reserve("robot_1", {edge_id});
  ASSERT_TRUE(res1.success);

  // 机器人 2 尝试竞争同一边，必须被拒绝
  auto res2 = traffic.Reserve("robot_2", {edge_id});
  EXPECT_FALSE(res2.success);

  // 构造死锁快照：robot_1 长期未推进
  amr_dispatcher_core::dispatcher::DeadlockDetectorConfig det_cfg;
  det_cfg.active_idle_threshold = 50ms;
  det_cfg.reservation_hold_threshold = 50ms;
  amr_dispatcher_core::dispatcher::DeadlockDetector detector(det_cfg);

  amr_dispatcher_core::dispatcher::DeadlockSnapshot snap;
  amr_dispatcher_core::dispatcher::Mission active_m;
  active_m.id = "mission_deadlock";
  active_m.reservation_id = edge_id;
  snap.active_missions.push_back(active_m);
  snap.last_progress["mission_deadlock"] = std::chrono::steady_clock::now() - 200ms;
  snap.reservation_acquired_at[edge_id] = std::chrono::steady_clock::now() - 200ms;

  auto findings = detector.Detect(snap);
  ASSERT_FALSE(findings.empty());

  // 触发自愈策略
  amr_dispatcher_core::dispatcher::RecoveryPolicyConfig recovery_cfg;
  recovery_cfg.max_retries = 1;
  recovery_cfg.auto_cancel_on_exhausted = true;
  amr_dispatcher_core::dispatcher::RecoveryPolicy recovery(recovery_cfg);
  auto actions = recovery.Decide(findings, [](const std::string&) { return 2; });  // 已重试 2 次
  ASSERT_FALSE(actions.empty());
  EXPECT_EQ(actions[0].kind, amr_dispatcher_core::dispatcher::RecoveryAction::Kind::kCancel);

  // 释放资源，后续申请立即可用
  auto rel = traffic.Release("robot_1");
  EXPECT_TRUE(rel.released);
  auto res2_retry = traffic.Reserve("robot_2", {edge_id});
  EXPECT_TRUE(res2_retry.success);
}

}  // namespace
