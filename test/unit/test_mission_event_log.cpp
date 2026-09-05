#include "amr_dispatcher_core/dispatcher/mission_event_log.hpp"

#include <gtest/gtest.h>

using namespace amr_dispatcher_core::dispatcher;

namespace {

MissionEvent Evt(const std::string& id, const std::string& ev) {
  MissionEvent e;
  e.stamp = "2026-09-04T12:00:00Z";
  e.mission_id = id;
  e.event = ev;
  return e;
}

}  // namespace

TEST(MissionEventLogTest, AppendAndRecent) {
  MissionEventLog log(4);
  log.Append(Evt("m1", "queued"));
  log.Append(Evt("m1", "active"));
  log.Append(Evt("m2", "queued"));
  const auto recent = log.Recent(2);
  ASSERT_EQ(recent.size(), 2u);
  // Recent() returns new→old; last Append was m2/"queued", before that m1/"active".
  EXPECT_EQ(recent[0].event, "queued");
  EXPECT_EQ(recent[0].mission_id, "m2");
  EXPECT_EQ(recent[1].event, "active");
  EXPECT_EQ(recent[1].mission_id, "m1");
}

TEST(MissionEventLogTest, OverflowEvictsOldest) {
  MissionEventLog log(3);
  log.Append(Evt("m1", "queued"));
  log.Append(Evt("m1", "active"));
  log.Append(Evt("m1", "finished"));
  log.Append(Evt("m1", "audit"));  // 挤出 queued
  const auto snap = log.Snapshot();
  ASSERT_EQ(snap.size(), 3u);
  EXPECT_EQ(snap[0].event, "active");
  EXPECT_EQ(snap[2].event, "audit");
  EXPECT_EQ(log.total_appended(), 4u);
}

TEST(MissionEventLogTest, FilterByEvent) {
  MissionEventLog log(8);
  log.Append(Evt("m1", "queued"));
  log.Append(Evt("m1", "active"));
  log.Append(Evt("m2", "queued"));
  const auto r = log.Recent(8, "queued");
  ASSERT_EQ(r.size(), 2u);
  for (const auto& e : r) {
    EXPECT_EQ(e.event, "queued");
  }
}

TEST(MissionEventLogTest, FilterByMission) {
  MissionEventLog log(8);
  log.Append(Evt("m1", "queued"));
  log.Append(Evt("m2", "queued"));
  const auto r = log.Recent(8, "", "m1");
  ASSERT_EQ(r.size(), 1u);
  EXPECT_EQ(r[0].mission_id, "m1");
}

TEST(MissionEventLogTest, NormalizeLimit) {
  EXPECT_EQ(MissionEventLog::NormalizeLimit(0, 100, 1000), 100u);
  EXPECT_EQ(MissionEventLog::NormalizeLimit(50, 100, 1000), 50u);
  EXPECT_EQ(MissionEventLog::NormalizeLimit(2000, 100, 1000), 1000u);
}

TEST(MissionEventLogTest, Clear) {
  MissionEventLog log(4);
  log.Append(Evt("m1", "queued"));
  log.Clear();
  EXPECT_EQ(log.size(), 0u);
}
