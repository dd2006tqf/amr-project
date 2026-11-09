#include "amr_dispatcher_ros/report_freshness.hpp"

#include <gtest/gtest.h>

#include <chrono>

using namespace amr_dispatcher_ros;
using namespace std::chrono_literals;

namespace {
using Clock = std::chrono::steady_clock;
using ArrivalMap = std::unordered_map<std::string, Clock::time_point>;
}  // namespace

// 数据在容忍窗口内到达 -> 新鲜
TEST(ReportFreshness, RecentArrivalIsFresh) {
  const auto now = Clock::now();
  ArrivalMap m{{"chassis", now - 100ms}, {"watchdog", now - 200ms}};
  const auto fresh = MakeReportFreshness(m, now, 1000ms);
  EXPECT_TRUE(fresh.chassis);
  EXPECT_TRUE(fresh.watchdog);
}

// 超出容忍窗口 -> 陈旧（上游发布者死亡的检测信号）
TEST(ReportFreshness, StaleArrivalIsNotFresh) {
  const auto now = Clock::now();
  ArrivalMap m{{"chassis", now - 3000ms}, {"watchdog", now - 100ms}};
  const auto fresh = MakeReportFreshness(m, now, 1000ms);
  EXPECT_FALSE(fresh.chassis);
  EXPECT_TRUE(fresh.watchdog);
}

// 从未到达过的源不判新鲜——由"从未上报不参与时效"语义兜底
TEST(ReportFreshness, NeverArrivedIsNotFresh) {
  const auto now = Clock::now();
  ArrivalMap m{{"watchdog", now - 100ms}};
  const auto fresh = MakeReportFreshness(m, now, 1000ms);
  EXPECT_FALSE(fresh.chassis);
  EXPECT_TRUE(fresh.watchdog);
}

// 恰好等于容忍窗口不算过期（严格超出才陈旧）
TEST(ReportFreshness, BoundaryWindowIsStillFresh) {
  const auto now = Clock::now();
  ArrivalMap m{{"chassis", now - 1000ms}};
  const auto fresh = MakeReportFreshness(m, now, 1000ms);
  EXPECT_TRUE(fresh.chassis);
}
