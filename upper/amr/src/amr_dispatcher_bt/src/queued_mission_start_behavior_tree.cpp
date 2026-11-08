
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

#include "amr_dispatcher_bt/queued_mission_start_behavior_tree.hpp"

#include "behaviortree_cpp/bt_factory.h"

namespace amr_dispatcher_bt {

using namespace amr_dispatcher_core::workflow;
using namespace amr_dispatcher_core::facility;
using namespace amr_dispatcher_core::catalog;

namespace {

struct QueuedMissionStartBtContext {
  QueuedMissionStartBehaviorTreeInput input;
  QueuedMissionStartBehaviorTreeResult result;
};

bool ShouldStartIdle(const QueuedMissionStartDecision& decision) {
  return decision.run_queue && decision.request_queue_start && !decision.cancel_active;
}

}  // namespace

std::string DefaultQueuedMissionStartBehaviorTreeXml() {
  return R"(
<root BTCPP_format="4" main_tree_to_execute="QueuedMissionStart">
  <BehaviorTree ID="QueuedMissionStart">
    <Sequence>
      <DecideQueuedMissionStart/>
      <Fallback>
        <Sequence>
          <ShouldPreemptActiveMission/>
          <StageActivePreempt/>
        </Sequence>
        <Sequence>
          <ShouldStartIdleMission/>
          <StageIdleStart/>
        </Sequence>
        <StageKeepQueued/>
      </Fallback>
    </Sequence>
  </BehaviorTree>
</root>
)";
}

QueuedMissionStartBehaviorTreeResult TickQueuedMissionStartBehaviorTree(
    const QueuedMissionStartBehaviorTreeInput& input) {
  QueuedMissionStartBtContext context;
  context.input = input;

  BT::BehaviorTreeFactory factory;
  factory.registerSimpleAction(
      "DecideQueuedMissionStart", [&context](BT::TreeNode&) {
        context.result.decision = DecideQueuedMissionStart(context.input.request);
        return BT::NodeStatus::SUCCESS;
      });
  factory.registerSimpleCondition(
      "ShouldPreemptActiveMission", [&context](BT::TreeNode&) {
        return context.result.decision.cancel_active ? BT::NodeStatus::SUCCESS
                                                     : BT::NodeStatus::FAILURE;
      });
  factory.registerSimpleAction(
      "StageActivePreempt", [&context](BT::TreeNode&) {
        context.result.branch = "preempt_active";
        return BT::NodeStatus::SUCCESS;
      });
  factory.registerSimpleCondition(
      "ShouldStartIdleMission", [&context](BT::TreeNode&) {
        return ShouldStartIdle(context.result.decision) ? BT::NodeStatus::SUCCESS
                                                        : BT::NodeStatus::FAILURE;
      });
  factory.registerSimpleAction(
      "StageIdleStart", [&context](BT::TreeNode&) {
        context.result.branch = "start_idle";
        return BT::NodeStatus::SUCCESS;
      });
  factory.registerSimpleAction(
      "StageKeepQueued", [&context](BT::TreeNode&) {
        context.result.branch = "keep_queued";
        return BT::NodeStatus::SUCCESS;
      });

  auto tree = factory.createTreeFromText(DefaultQueuedMissionStartBehaviorTreeXml());
  context.result.success = tree.tickOnce() == BT::NodeStatus::SUCCESS;
  return context.result;
}

}  // namespace amr_dispatcher_bt
