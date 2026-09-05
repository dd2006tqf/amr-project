#include "amr_dispatcher_core/dispatcher/recovery_policy.hpp"

#include <gtest/gtest.h>

using namespace amr_dispatcher_core::dispatcher;

namespace {

DeadlockFinding Make(DeadlockKind kind, std::vector<std::string> ids,
                     std::string desc = "x") {
  DeadlockFinding f;
  f.kind = kind;
  f.mission_ids = std::move(ids);
  f.description = std::move(desc);
  return f;
}

}  // namespace

TEST(RecoveryPolicyTest, IdleActiveWithRetriesLeftPicksRetry) {
  RecoveryPolicy p({.max_retries = 3, .auto_cancel_on_exhausted = true});
  std::vector<DeadlockFinding> fs{Make(DeadlockKind::kIdleActive, {"a"})};
  auto acts = p.Decide(fs, [](const std::string&) { return 0; });
  ASSERT_EQ(acts.size(), 1u);
  EXPECT_EQ(acts[0].kind, RecoveryAction::Kind::kRetry);
}

TEST(RecoveryPolicyTest, IdleActiveExhaustedCancels) {
  RecoveryPolicy p({.max_retries = 1, .auto_cancel_on_exhausted = true});
  std::vector<DeadlockFinding> fs{Make(DeadlockKind::kIdleActive, {"a"})};
  auto acts = p.Decide(fs, [](const std::string&) { return 5; });
  ASSERT_EQ(acts.size(), 1u);
  EXPECT_EQ(acts[0].kind, RecoveryAction::Kind::kCancel);
}

TEST(RecoveryPolicyTest, ExhaustedWithoutAutoCancelPauses) {
  RecoveryPolicy p({.max_retries = 1, .auto_cancel_on_exhausted = false});
  std::vector<DeadlockFinding> fs{Make(DeadlockKind::kIdleActive, {"a"})};
  auto acts = p.Decide(fs, [](const std::string&) { return 5; });
  EXPECT_EQ(acts[0].kind, RecoveryAction::Kind::kPause);
}

TEST(RecoveryPolicyTest, MutualBlockAlwaysPauses) {
  RecoveryPolicy p;
  std::vector<DeadlockFinding> fs{Make(DeadlockKind::kMutualBlock, {"a", "b"})};
  auto acts = p.Decide(fs, [](const std::string&) { return 0; });
  ASSERT_EQ(acts.size(), 2u);
  for (const auto& a : acts) {
    EXPECT_EQ(a.kind, RecoveryAction::Kind::kPause);
  }
}

TEST(RecoveryPolicyTest, DedupAcrossFindings) {
  RecoveryPolicy p;
  std::vector<DeadlockFinding> fs{
      Make(DeadlockKind::kIdleActive, {"a"}),
      Make(DeadlockKind::kLongReservationHold, {"a"}),
  };
  auto acts = p.Decide(fs, [](const std::string&) { return 0; });
  EXPECT_EQ(acts.size(), 1u);
}

TEST(RecoveryPolicyTest, LongHoldRetryWhenBudgetLeft) {
  RecoveryPolicy p({.max_retries = 2});
  std::vector<DeadlockFinding> fs{Make(DeadlockKind::kLongReservationHold, {"a"})};
  auto acts = p.Decide(fs, [](const std::string&) { return 1; });
  ASSERT_EQ(acts.size(), 1u);
  EXPECT_EQ(acts[0].kind, RecoveryAction::Kind::kRetry);
}
