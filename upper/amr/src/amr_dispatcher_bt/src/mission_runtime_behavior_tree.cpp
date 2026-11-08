
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

#include "behaviortree_cpp/bt_factory.h"

namespace amr_dispatcher_bt {

using namespace amr_dispatcher_core::workflow;
using namespace amr_dispatcher_core::facility;
using namespace amr_dispatcher_core::catalog;

namespace {

struct MissionRuntimeBtContext {
  MissionRuntimeBehaviorTreeInput input;
  MissionRuntimeBehaviorTreeResult result;
};

}  // namespace

std::string DefaultMissionRuntimeBehaviorTreeXml() {
  return R"(
<root BTCPP_format="4" main_tree_to_execute="MissionRuntimeTick">
  <BehaviorTree ID="MissionRuntimeTick">
    <Sequence>
      <DecideMissionRuntimeTick/>
      <Fallback>
        <Sequence>
          <ShouldAutostartMission/>
          <ShouldStartQueuedMission/>
          <StageAutostartThenQueue/>
        </Sequence>
        <Sequence>
          <ShouldAutostartMission/>
          <StageAutostart/>
        </Sequence>
        <Sequence>
          <ShouldStartQueuedMission/>
          <StageQueueStart/>
        </Sequence>
        <StagePublishOnly/>
      </Fallback>
    </Sequence>
  </BehaviorTree>
</root>
)";
}

MissionRuntimeBehaviorTreeResult TickMissionRuntimeBehaviorTree(
    const MissionRuntimeBehaviorTreeInput& input) {
  MissionRuntimeBtContext context;
  context.input = input;

  BT::BehaviorTreeFactory factory;
  factory.registerSimpleAction(
      "DecideMissionRuntimeTick", [&context](BT::TreeNode&) {
        context.result.decision = DecideMissionRuntimeTick(context.input.tick);
        return BT::NodeStatus::SUCCESS;
      });
  factory.registerSimpleCondition(
      "ShouldAutostartMission", [&context](BT::TreeNode&) {
        return context.result.decision.start_default_mission ? BT::NodeStatus::SUCCESS
                                                             : BT::NodeStatus::FAILURE;
      });
  factory.registerSimpleCondition(
      "ShouldStartQueuedMission", [&context](BT::TreeNode&) {
        return context.result.decision.start_queued_mission ? BT::NodeStatus::SUCCESS
                                                            : BT::NodeStatus::FAILURE;
      });
  factory.registerSimpleAction(
      "StageAutostartThenQueue", [&context](BT::TreeNode&) {
        context.result.branch = "autostart_then_queue_if_still_idle";
        return BT::NodeStatus::SUCCESS;
      });
  factory.registerSimpleAction(
      "StageAutostart", [&context](BT::TreeNode&) {
        context.result.branch = "autostart";
        return BT::NodeStatus::SUCCESS;
      });
  factory.registerSimpleAction(
      "StageQueueStart", [&context](BT::TreeNode&) {
        context.result.branch = "queue_start";
        return BT::NodeStatus::SUCCESS;
      });
  factory.registerSimpleAction(
      "StagePublishOnly", [&context](BT::TreeNode&) {
        context.result.branch = "publish_only";
        return BT::NodeStatus::SUCCESS;
      });

  auto tree = factory.createTreeFromText(DefaultMissionRuntimeBehaviorTreeXml());
  context.result.success = tree.tickOnce() == BT::NodeStatus::SUCCESS;
  return context.result;
}

}  // namespace amr_dispatcher_bt
