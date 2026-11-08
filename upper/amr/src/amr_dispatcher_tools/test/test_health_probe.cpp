#include "amr_dispatcher_tools/health_probe.hpp"

#include <gtest/gtest.h>

namespace amr_dispatcher_tools {
namespace {

ComponentProbe FreshOk(const std::string& name, double age = 0.5) {
  ComponentProbe probe;
  probe.name = name;
  probe.seen = true;
  probe.semantic_ok = true;
  probe.age_seconds = age;
  probe.stale_after_seconds = 3.0;
  return probe;
}

ComponentProbe FreshBad(const std::string& name, const std::string& detail = "CRITICAL") {
  auto probe = FreshOk(name);
  probe.semantic_ok = false;
  probe.detail = detail;
  return probe;
}

// ---------------------------------------------------------------- 单组件判定

TEST(HealthProbeTest, FreshAndHealthyIsOk) {
  EXPECT_EQ(EvaluateComponent(FreshOk("dispatcher")), ComponentStatus::kOk);
}

TEST(HealthProbeTest, AgeAtThresholdIsStillFresh) {
  auto probe = FreshOk("dispatcher", 3.0);  // age == stale_after
  EXPECT_EQ(EvaluateComponent(probe), ComponentStatus::kOk);
}

TEST(HealthProbeTest, AgeJustOverThresholdIsStale) {
  auto probe = FreshOk("dispatcher", 3.001);
  EXPECT_EQ(EvaluateComponent(probe), ComponentStatus::kStale);
}

// 从未收到 = 不就绪。这是 fail-closed 的底线：进程活着但不出声比死掉更危险。
TEST(HealthProbeTest, NeverSeenIsNotReadyAndNotTreatedAsHealthy) {
  ComponentProbe probe;
  probe.name = "safety";
  probe.age_seconds = -1.0;
  EXPECT_EQ(EvaluateComponent(probe), ComponentStatus::kNeverSeen);

  const auto verdict = EvaluateReadiness({probe});
  EXPECT_FALSE(verdict.ready);
  EXPECT_EQ(verdict.state, "INITIALIZING");
}

TEST(HealthProbeTest, FreshButSemanticallyBadIsUnhealthy) {
  EXPECT_EQ(EvaluateComponent(FreshBad("safety")), ComponentStatus::kUnhealthy);
}

// ---------------------------------------------------------------- 汇总判定

TEST(HealthProbeTest, AllComponentsOkIsReady) {
  const auto verdict = EvaluateReadiness(
      {FreshOk("dispatcher"), FreshOk("safety"), FreshOk("chassis"), FreshOk("coordinator")});
  EXPECT_TRUE(verdict.ready);
  EXPECT_EQ(verdict.state, "READY");
  EXPECT_TRUE(verdict.message.find("healthy") != std::string::npos);
}

TEST(HealthProbeTest, StaleComponentDegradesInsteadOfInitializing) {
  auto stale = FreshOk("dispatcher", 10.0);
  const auto verdict = EvaluateReadiness({stale, FreshOk("safety")});
  EXPECT_FALSE(verdict.ready);
  EXPECT_EQ(verdict.state, "DEGRADED");
  EXPECT_TRUE(verdict.message.find("dispatcher") != std::string::npos);
}

// 混合场景：见过但坏 + 从未出声 -> 必须先报降级（已经确定坏了），
// 不能因为还有一个没出声就把它降格成"初始化中"。
TEST(HealthProbeTest, SeenButBadWinsOverNeverSeen) {
  ComponentProbe never;
  never.name = "chassis";
  never.age_seconds = -1.0;
  const auto verdict = EvaluateReadiness({FreshBad("safety"), never});
  EXPECT_FALSE(verdict.ready);
  EXPECT_EQ(verdict.state, "DEGRADED");
}

// 未监控 ≠ 健康：它不参与结论，但也不许被当成"已通过"来掩盖其他问题。
TEST(HealthProbeTest, NotMonitoredDoesNotFailReadiness) {
  ComponentProbe unmonitored;
  unmonitored.name = "chassis";
  unmonitored.monitored = false;
  unmonitored.age_seconds = -1.0;
  const auto verdict = EvaluateReadiness({FreshOk("dispatcher"), unmonitored});
  EXPECT_TRUE(verdict.ready);
  EXPECT_EQ(EvaluateComponent(unmonitored), ComponentStatus::kNotMonitored);
}

TEST(HealthProbeTest, NotMonitoredAloneIsNotEnoughToBeReady) {
  ComponentProbe unmonitored;
  unmonitored.name = "chassis";
  unmonitored.monitored = false;
  const auto verdict = EvaluateReadiness({unmonitored});
  // 没有任何受监控组件 -> 空集全过在逻辑上会让"什么都没监控"看起来就绪，
  // 这是不可接受的：至少一个受监控组件且全 ok 才算就绪。
  EXPECT_FALSE(verdict.ready);
}

TEST(HealthProbeTest, EmptyComponentListIsNotReady) {
  const auto verdict = EvaluateReadiness({});
  EXPECT_FALSE(verdict.ready);
  EXPECT_EQ(verdict.state, "INITIALIZING");
  EXPECT_EQ(verdict.message, "no monitored components");
}

// ---------------------------------------------------------------- JSON 契约

TEST(HealthProbeTest, JsonCarriesPerComponentDiagnostics) {
  const auto verdict = EvaluateReadiness({FreshOk("dispatcher"), FreshBad("safety")});
  const std::string json = RenderReadinessJson(verdict);
  EXPECT_TRUE(json.find("\"ready\":false") != std::string::npos);
  EXPECT_TRUE(json.find("\"state\":\"DEGRADED\"") != std::string::npos);
  EXPECT_TRUE(json.find("\"component\":\"dispatcher\"") != std::string::npos);
  EXPECT_TRUE(json.find("\"reason\":\"ok\"") != std::string::npos);
  EXPECT_TRUE(json.find("\"component\":\"safety\"") != std::string::npos);
  EXPECT_TRUE(json.find("\"reason\":\"unhealthy\"") != std::string::npos);
  // 每个组件都必须带 age，ops 要靠它判断"多久没说话了"
  EXPECT_TRUE(json.find("\"age_seconds\":") != std::string::npos);
}

TEST(HealthProbeTest, JsonEscapesComponentDetail) {
  auto probe = FreshBad("safety", "CRITICAL \"quote\" and \\ backslash");
  const std::string json = RenderReadinessJson(EvaluateReadiness({probe}));
  // 未转义的引号会让整份 JSON 非法，前端 JSON.parse 直接抛
  EXPECT_TRUE(json.find("\"detail\":\"CRITICAL \\\"quote\\\" and \\\\ backslash\"") !=
              std::string::npos);
}

TEST(HealthProbeTest, JsonReportsNeverSeenWithNegativeAge) {
  ComponentProbe probe;
  probe.name = "coordinator";
  probe.age_seconds = -1.0;
  const std::string json = RenderReadinessJson(EvaluateReadiness({probe}));
  EXPECT_TRUE(json.find("\"reason\":\"never_seen\"") != std::string::npos);
  // age 是 JSON 数值（不是字符串）：-1 表示"从未收到"，抓取端/脚本可直接比较
  EXPECT_TRUE(json.find("\"age_seconds\":-1.000") != std::string::npos);
}

TEST(HealthProbeTest, LivenessJsonIsStableShape) {
  const std::string json = RenderLivenessJson(12.5);
  EXPECT_EQ(json, "{\"status\":\"ok\",\"uptime_seconds\":12.500}");
}

}  // namespace
}  // namespace amr_dispatcher_tools
