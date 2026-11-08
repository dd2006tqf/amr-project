#include "amr_dispatcher_tools/alerting.hpp"

#include <gtest/gtest.h>

namespace amr_dispatcher_tools {
namespace {

AlertObservation Observation(std::int64_t now, bool ready, bool safety_critical = false,
                             std::vector<std::string> missing = {},
                             std::string details = "{}") {
  AlertObservation observation;
  observation.now_unix = now;
  observation.ready = ready;
  observation.safety_critical = safety_critical;
  observation.missing_components = std::move(missing);
  observation.details_json = std::move(details);
  return observation;
}

AlertType OnlyType(const std::vector<AlertEvent>& events) {
  EXPECT_EQ(events.size(), 1u);
  return events.empty() ? AlertType::kReadinessNotReady : events.front().type;
}

// ------------------------------------------------------- 就绪告警：连续 N 次才升级

TEST(AlertingTest, SingleNotReadyObservationDoesNotFire) {
  AlertStateMachine sm{AlertPolicy{3, 300, 30}};
  EXPECT_TRUE(sm.Observe(Observation(100, false)).empty());
  EXPECT_TRUE(sm.Observe(Observation(101, false)).empty());
  EXPECT_FALSE(sm.Firing("readiness_not_ready"));
}

TEST(AlertingTest, FiresOnThirdConsecutiveFailure) {
  AlertStateMachine sm{AlertPolicy{3, 300, 30}};
  sm.Observe(Observation(100, false));
  sm.Observe(Observation(101, false));
  const auto events = sm.Observe(Observation(102, false));
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events.front().type, AlertType::kReadinessNotReady);
  EXPECT_EQ(events.front().state, "firing");
  EXPECT_EQ(events.front().severity, "critical");
  EXPECT_EQ(events.front().dedup_key, "readiness_not_ready");
  EXPECT_EQ(events.front().consecutive_observations, 3u);
  EXPECT_TRUE(sm.Firing("readiness_not_ready"));
}

// 中间有一次就绪就清零计数 —— 抖动不该累积成告警。
TEST(AlertingTest, RecoveryResetsConsecutiveCounter) {
  AlertStateMachine sm{AlertPolicy{3, 300, 30}};
  sm.Observe(Observation(100, false));
  sm.Observe(Observation(101, false));
  EXPECT_TRUE(sm.Observe(Observation(102, true)).empty());  // 从未 firing，不发恢复
  EXPECT_TRUE(sm.Observe(Observation(103, false)).empty());
  EXPECT_TRUE(sm.Observe(Observation(104, false)).empty());
  EXPECT_EQ(sm.Observe(Observation(105, false)).size(), 1u);
}

TEST(AlertingTest, FiringSuppressesRepeatsWithinInterval) {
  AlertStateMachine sm{AlertPolicy{2, 300, 30}};
  sm.Observe(Observation(100, false));
  EXPECT_EQ(sm.Observe(Observation(101, false)).size(), 1u);
  // 限流窗口内：只累计 consecutive，不再发
  EXPECT_TRUE(sm.Observe(Observation(200, false)).empty());
  EXPECT_TRUE(sm.Observe(Observation(300, false)).empty());
  // 越过窗口：发一次"仍然不就绪"
  const auto repeat = sm.Observe(Observation(401, false));
  ASSERT_EQ(repeat.size(), 1u);
  EXPECT_EQ(repeat.front().type, AlertType::kReadinessNotReady);
  EXPECT_EQ(repeat.front().state, "firing");
}

// "好了"必须准时到达，不受限流影响；且只发一次，之后完全静默。
TEST(AlertingTest, RecoveryIsEmittedExactlyOnceAndNotThrottled) {
  AlertStateMachine sm{AlertPolicy{2, 300, 30}};
  sm.Observe(Observation(100, false));
  sm.Observe(Observation(101, false));
  const auto recovered = sm.Observe(Observation(102, true));  // 远未到限流窗口
  ASSERT_EQ(recovered.size(), 1u);
  EXPECT_EQ(recovered.front().type, AlertType::kReadinessRecovered);
  EXPECT_EQ(recovered.front().state, "resolved");
  EXPECT_EQ(recovered.front().severity, "info");
  EXPECT_EQ(recovered.front().fired_at_unix, 101);     // 保留最初触发时刻，便于算持续时长
  EXPECT_EQ(recovered.front().resolved_at_unix, 102);
  EXPECT_FALSE(sm.Firing("readiness_not_ready"));
  EXPECT_TRUE(sm.Observe(Observation(103, true)).empty());
  EXPECT_TRUE(sm.Observe(Observation(104, true)).empty());
}

// ------------------------------------------------------- 安全门 CRITICAL 持续时长

TEST(AlertingTest, SafetyCriticalMustBeSustainedBeforeFiring) {
  AlertStateMachine sm{AlertPolicy{3, 300, 30}};
  // 第 1 秒出现 CRITICAL：这是常态抖动，不该报警
  EXPECT_TRUE(sm.Observe(Observation(100, true, true)).empty());
  EXPECT_TRUE(sm.Observe(Observation(110, true, true)).empty());
  EXPECT_TRUE(sm.Observe(Observation(129, true, true)).empty());
  // 越过 30 秒持续阈值
  const auto events = sm.Observe(Observation(130, true, true));
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events.front().type, AlertType::kSafetyCritical);
  EXPECT_EQ(events.front().fired_at_unix, 100);  // 记的是首次触发时刻，不是报警时刻
}

TEST(AlertingTest, BriefSafetyCriticalBlipNeverFires) {
  AlertStateMachine sm{AlertPolicy{3, 300, 30}};
  sm.Observe(Observation(100, true, true));
  sm.Observe(Observation(105, true, true));
  EXPECT_TRUE(sm.Observe(Observation(110, true, false)).empty());  // 解除
  EXPECT_FALSE(sm.Firing("safety_critical"));
  // 再次出现时持续时长重新计时（不会把前一次的 10 秒算进来）
  sm.Observe(Observation(120, true, true));
  EXPECT_TRUE(sm.Observe(Observation(140, true, true)).empty());
  EXPECT_EQ(sm.Observe(Observation(150, true, true)).size(), 1u);
}

TEST(AlertingTest, SafetyRecoveryIsEmitted) {
  AlertStateMachine sm{AlertPolicy{2, 300, 5}};
  sm.Observe(Observation(100, true, true));
  EXPECT_EQ(sm.Observe(Observation(106, true, true)).size(), 1u);
  const auto recovered = sm.Observe(Observation(107, true, false));
  ASSERT_EQ(recovered.size(), 1u);
  EXPECT_EQ(recovered.front().type, AlertType::kSafetyRecovered);
  EXPECT_EQ(recovered.front().state, "resolved");
}

// ------------------------------------------------------- 单组件缺失

// 整体不就绪与组件缺失是两条独立告警：同一拍里各发一条，互不吞并。
// （组件缺失的 dedup_key 与整体开关是两个独立 slot，值班人员分别路由。）
TEST(AlertingTest, NotReadyAndMissingComponentFireTogether) {
  AlertStateMachine sm{AlertPolicy{2, 300, 30}};
  sm.Observe(Observation(100, false, false, {"dispatcher"}));
  const auto events = sm.Observe(Observation(101, false, false, {"dispatcher"}));
  ASSERT_EQ(events.size(), 2u);
  EXPECT_EQ(events[0].dedup_key, "readiness_not_ready");
  EXPECT_EQ(events[1].dedup_key, "component_missing:dispatcher");
  EXPECT_EQ(events[1].type, AlertType::kComponentMissing);
  EXPECT_EQ(events[1].severity, "warning");
  EXPECT_TRUE(sm.Firing("component_missing:dispatcher"));
}

TEST(AlertingTest, ComponentRecoveryIsEmittedAndKeyCleared) {
  AlertStateMachine sm{AlertPolicy{1, 300, 30}};
  sm.Observe(Observation(100, true, false, {"safety"}));
  const auto recovered = sm.Observe(Observation(101, true, false, {}));
  ASSERT_EQ(recovered.size(), 1u);
  EXPECT_EQ(recovered.front().type, AlertType::kComponentRecovered);
  EXPECT_EQ(recovered.front().dedup_key, "component_missing:safety");
  EXPECT_EQ(recovered.front().state, "resolved");
  EXPECT_FALSE(sm.Firing("component_missing:safety"));
}

// 两个组件同时坏：各自独立告警，恢复时也各自恢复（值班人员要能分别路由）。
TEST(AlertingTest, TwoComponentsTrackIndependently) {
  AlertStateMachine sm{AlertPolicy{1, 300, 30}};
  auto first = sm.Observe(Observation(100, true, false, {"safety", "chassis"}));
  EXPECT_EQ(first.size(), 2u);
  // 只剩 chassis 还坏着
  auto second = sm.Observe(Observation(101, true, false, {"chassis"}));
  ASSERT_EQ(second.size(), 1u);
  EXPECT_EQ(second.front().type, AlertType::kComponentRecovered);
  EXPECT_EQ(second.front().dedup_key, "component_missing:safety");
  EXPECT_TRUE(sm.Firing("component_missing:chassis"));
}

// ------------------------------------------------------- 一次性事件

TEST(AlertingTest, NotifyOnceEmitsWithoutDedupState) {
  AlertStateMachine sm;
  const auto events = sm.NotifyOnce(AlertType::kTopologyOverlayInvalidated, "info",
                                    "topology_overlay_invalidated",
                                    "overlay invalidated by new map", Observation(500, true));
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events.front().type, AlertType::kTopologyOverlayInvalidated);
  // 不进状态机：不产生"恢复"，也不占 dedup slot
  EXPECT_FALSE(sm.Firing("topology_overlay_invalidated"));
}

// ------------------------------------------------------- 载荷

TEST(AlertingTest, PayloadIsWellFormedJsonWithSchema) {
  AlertStateMachine sm{AlertPolicy{1, 300, 30}};
  const auto events = sm.Observe(Observation(1700000000, false, false,
                                             {"dispatcher"},
                                             "{\"dispatcher_state_age_ms\":12000}"));
  ASSERT_FALSE(events.empty());
  const std::string payload =
      RenderAlertPayload(events.front(), "amr-server", 8080, 42, 1234);
  EXPECT_TRUE(payload.find("\"schema\":\"amr.alert.v1\"") != std::string::npos);
  EXPECT_TRUE(payload.find("\"type\":\"readiness_not_ready\"") != std::string::npos);
  EXPECT_TRUE(payload.find("\"state\":\"firing\"") != std::string::npos);
  EXPECT_TRUE(payload.find("\"instance\":{\"host\":\"amr-server\",\"pid\":1234,") !=
              std::string::npos);
  EXPECT_TRUE(payload.find("\"ros_domain_id\":42") != std::string::npos);
  EXPECT_TRUE(payload.find("\"gateway_port\":8080") != std::string::npos);
  // 时间戳是 ISO-8601 UTC，不是裸 unix 秒
  EXPECT_TRUE(payload.find("\"fired_at\":\"2023-11-14T22:13:20Z\"") != std::string::npos);
  EXPECT_TRUE(payload.find("\"resolved_at\":null") != std::string::npos);
  // details 原样嵌入（已是合法 JSON 对象）
  EXPECT_TRUE(payload.find("\"details\":{\"dispatcher_state_age_ms\":12000}") !=
              std::string::npos);
}

TEST(AlertingTest, PayloadEscapesHostAndSummary) {
  AlertEvent event;
  event.type = AlertType::kReadinessNotReady;
  event.summary = "quoted \"summary\"\nwith newline";
  event.fired_at_unix = 1700000000;
  const std::string payload = RenderAlertPayload(event, "host\"name", 8080, 42, 1);
  EXPECT_TRUE(payload.find("quoted \\\"summary\\\"\\nwith newline") != std::string::npos);
  EXPECT_TRUE(payload.find("\"host\":\"host\\\"name\"") != std::string::npos);
}

TEST(AlertingTest, ResolvedPayloadCarriesBothTimestamps) {
  // 只让"整体不就绪"触发，不附带缺失组件（否则会多出一条 component_missing 事件）
  AlertStateMachine sm{AlertPolicy{1, 300, 30}};
  sm.Observe(Observation(1700000000, false));
  const auto recovered = sm.Observe(Observation(1700000060, true));
  ASSERT_EQ(recovered.size(), 1u);
  const std::string payload = RenderAlertPayload(recovered.front(), "h", 8080, 42, 1);
  EXPECT_TRUE(payload.find("\"state\":\"resolved\"") != std::string::npos);
  EXPECT_TRUE(payload.find("\"fired_at\":\"2023-11-14T22:13:20Z\"") != std::string::npos);
  EXPECT_TRUE(payload.find("\"resolved_at\":\"2023-11-14T22:14:20Z\"") != std::string::npos);
}

TEST(AlertingTest, FiringCountTracksActiveAlerts) {
  AlertStateMachine sm{AlertPolicy{1, 0, 30}};
  EXPECT_EQ(sm.FiringCount(), 0u);
  sm.Observe(Observation(100, false, false, {"safety"}));
  EXPECT_GE(sm.FiringCount(), 2u);  // readiness_not_ready + component_missing:safety
  sm.Observe(Observation(101, true, false, {}));
  EXPECT_EQ(sm.FiringCount(), 0u);
}

}  // namespace
}  // namespace amr_dispatcher_tools
