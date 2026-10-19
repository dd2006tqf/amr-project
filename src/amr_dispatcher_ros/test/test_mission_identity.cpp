#include "amr_dispatcher_ros/mission_identity.hpp"

#include <gtest/gtest.h>

using namespace amr_dispatcher_ros;

// B4：id 与 sequence 必须取自同一次自增的 n，修复 id=n+1 / sequence=n 错位
TEST(MissionIdentity, IdMatchesSequence) {
  std::uint64_t next = 1;
  amr_dispatcher_core::dispatcher::Mission m;
  StampMissionIdentity(m, next);
  EXPECT_EQ(m.id, "m_1");
  EXPECT_EQ(m.sequence, 1u);
  EXPECT_EQ(next, 2u);
}

// B4：连续两次戳记单调递增
TEST(MissionIdentity, SequenceIncrementsMonotonically) {
  std::uint64_t next = 1;
  amr_dispatcher_core::dispatcher::Mission a, b;
  StampMissionIdentity(a, next);
  StampMissionIdentity(b, next);
  EXPECT_EQ(a.sequence + 1, b.sequence);
  EXPECT_EQ(a.id, "m_" + std::to_string(a.sequence));
  EXPECT_EQ(b.id, "m_" + std::to_string(b.sequence));
}

// B4：id 前缀可定制（preempt_/dock 场景仍走同一戳记路径）
TEST(MissionIdentity, CustomPrefix) {
  std::uint64_t next = 7;
  amr_dispatcher_core::dispatcher::Mission m;
  StampMissionIdentity(m, next, "preempt_");
  EXPECT_EQ(m.id, "preempt_7");
  EXPECT_EQ(m.sequence, 7u);
}
