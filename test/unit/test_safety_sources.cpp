#include "amr_dispatcher_core/safety/safety_sources.hpp"

#include <gtest/gtest.h>

using namespace amr_dispatcher_core::safety;

TEST(SafetySourcesTest, RegisterListContains) {
  SafetySourceRegistry r;
  r.Register({"bumper", "front bumper", {}});
  r.Register({"battery", "BMS", {}});
  EXPECT_EQ(r.List().size(), 2u);
  EXPECT_TRUE(r.Contains("bumper"));
  EXPECT_FALSE(r.Contains("none"));
}

TEST(SafetySourcesTest, UnregisterRemoves) {
  SafetySourceRegistry r;
  r.Register({"a", "", {}});
  r.Unregister("a");
  EXPECT_FALSE(r.Contains("a"));
}

TEST(SafetySourcesTest, SnapshotCallsSample) {
  SafetySourceRegistry r;
  r.Register({"x", "desc", [] { return SafetyEvent{"x", SafetySeverity::kWarn, "snap"}; }});
  auto evs = r.Snapshot();
  ASSERT_EQ(evs.size(), 1u);
  EXPECT_EQ(evs[0].source, "x");
  EXPECT_EQ(evs[0].severity, SafetySeverity::kWarn);
}

TEST(SafetySourcesTest, EmptySourceDefaultsToName) {
  SafetySourceRegistry r;
  r.Register({"y", "desc", [] { return SafetyEvent{"", SafetySeverity::kInfo, "m"}; }});
  auto evs = r.Snapshot();
  ASSERT_EQ(evs.size(), 1u);
  EXPECT_EQ(evs[0].source, "y");
}
