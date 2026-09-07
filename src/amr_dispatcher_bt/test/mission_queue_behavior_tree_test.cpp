
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

#include "amr_dispatcher_bt/mission_queue_behavior_tree.hpp"

#include <gtest/gtest.h>

namespace amr_dispatcher_bt {

using namespace amr_dispatcher_core::workflow;
using namespace amr_dispatcher_core::facility;
using namespace amr_dispatcher_core::catalog;

namespace {

QueuedMission MakeQueuedMission(
    const std::string& mission_id, const int priority, const std::uint64_t sequence) {
  QueuedMission mission;
  mission.profile.mission_id = mission_id;
  mission.mission_file = mission_id + ".yaml";
  mission.priority = priority;
  mission.sequence = sequence;
  return mission;
}

}  // namespace

TEST(MissionQueueBehaviorTreeTest, RunnableQueue_PopsFirstRunnableMission) {
  MissionQueueBehaviorTreeInput input;
  input.queue = {
      MakeQueuedMission("station_order_fleet_paused_order_step_0", 10, 0),
      MakeQueuedMission("delivery_b", 2, 1),
  };
  input.paused_order_ids = {"paused_order"};

  const auto result = TickMissionQueueBehaviorTree(input);

  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.branch, "mission_queue_dispatch");
  ASSERT_TRUE(result.dispatch.mission.has_value());
  EXPECT_EQ(result.dispatch.mission->profile.mission_id, "delivery_b");
  ASSERT_EQ(result.dispatch.remaining_queue.size(), 1U);
  EXPECT_EQ(
      result.dispatch.remaining_queue[0].profile.mission_id,
      "station_order_fleet_paused_order_step_0");
}

TEST(MissionQueueBehaviorTreeTest, EmptyQueue_ReturnsQueueEmptyFailure) {
  const auto result = TickMissionQueueBehaviorTree(MissionQueueBehaviorTreeInput{});

  EXPECT_FALSE(result.success);
  EXPECT_TRUE(result.dispatch.queue_empty);
  EXPECT_EQ(result.dispatch.message, "mission queue is empty");
  EXPECT_TRUE(result.dispatch.remaining_queue.empty());
}

TEST(MissionQueueBehaviorTreeTest, AllPausedQueue_ReturnsAllPausedFailure) {
  MissionQueueBehaviorTreeInput input;
  input.queue = {MakeQueuedMission("station_sequence_paused_order_step_0", 10, 0)};
  input.paused_order_ids = {"paused_order"};

  const auto result = TickMissionQueueBehaviorTree(input);

  EXPECT_FALSE(result.success);
  EXPECT_TRUE(result.dispatch.all_paused);
  EXPECT_EQ(result.dispatch.message, "all queued workflow missions are paused");
  ASSERT_EQ(result.dispatch.remaining_queue.size(), 1U);
  EXPECT_EQ(result.dispatch.remaining_queue[0].profile.mission_id, input.queue[0].profile.mission_id);
}

}  // namespace amr_dispatcher_bt
