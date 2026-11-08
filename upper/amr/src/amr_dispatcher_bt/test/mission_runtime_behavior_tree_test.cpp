
#include "amr_dispatcher_core/workflow/docking_workflow.hpp"
#include "amr_dispatcher_core/workflow/mission_recovery_policy.hpp"
#include "amr_dispatcher_core/workflow/mission_result_workflow.hpp"
#include "amr_dispatcher_core/workflow/mission_runtime_workflow.hpp"
#include "amr_dispatcher_core/workflow/queued_mission.hpp"
#include "amr_dispatcher_core/workflow/station_sequence_workflow.hpp"
#include "amr_dispatcher_core/workflow/submit_order_router.hpp"
#include "amr_dispatcher_core/workflow/mission_resource_cleanup.hpp"
#include "amr_dispatcher_core/facility/facility_workflow.hpp"
#include "amr_dispatcher_core/facility/facility_reservation.hpp"
#include "amr_dispatcher_core/facility/facility_catalog.hpp"
#include "amr_dispatcher_core/catalog/station_catalog.hpp"

#include "amr_dispatcher_bt/mission_runtime_behavior_tree.hpp"

#include <gtest/gtest.h>

namespace amr_dispatcher_bt {

using namespace amr_dispatcher_core::workflow;
using namespace amr_dispatcher_core::facility;
using namespace amr_dispatcher_core::catalog;

namespace {

MissionRuntimeBehaviorTreeInput MakeInput(
    const bool autostart, const bool autostart_sent, const bool mission_active,
    const bool queue_start_requested) {
  MissionRuntimeBehaviorTreeInput input;
  input.tick.autostart = autostart;
  input.tick.autostart_sent = autostart_sent;
  input.tick.mission_active = mission_active;
  input.tick.queue_start_requested = queue_start_requested;
  return input;
}

}  // namespace

TEST(MissionRuntimeBehaviorTreeTest, AutostartIdle_SelectsAutostartBranch) {
  const auto result = TickMissionRuntimeBehaviorTree(MakeInput(true, false, false, false));

  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.branch, "autostart");
  EXPECT_TRUE(result.decision.start_default_mission);
  EXPECT_FALSE(result.decision.start_queued_mission);
}

TEST(MissionRuntimeBehaviorTreeTest, QueueStartIdle_SelectsQueueStartBranch) {
  const auto result = TickMissionRuntimeBehaviorTree(MakeInput(false, false, false, true));

  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.branch, "queue_start");
  EXPECT_FALSE(result.decision.start_default_mission);
  EXPECT_TRUE(result.decision.start_queued_mission);
}

TEST(MissionRuntimeBehaviorTreeTest, AutostartAndQueueRequested_SelectsCombinedBranch) {
  const auto result = TickMissionRuntimeBehaviorTree(MakeInput(true, false, false, true));

  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.branch, "autostart_then_queue_if_still_idle");
  EXPECT_TRUE(result.decision.start_default_mission);
  EXPECT_TRUE(result.decision.start_queued_mission);
}

TEST(MissionRuntimeBehaviorTreeTest, NoRuntimeStartNeeded_SelectsPublishOnlyBranch) {
  const auto result = TickMissionRuntimeBehaviorTree(MakeInput(true, true, false, false));

  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.branch, "publish_only");
  EXPECT_FALSE(result.decision.start_default_mission);
  EXPECT_FALSE(result.decision.start_queued_mission);
}

}  // namespace amr_dispatcher_bt
