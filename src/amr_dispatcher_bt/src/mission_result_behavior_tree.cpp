
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

#include "amr_dispatcher_bt/mission_result_behavior_tree.hpp"

#include "behaviortree_cpp/bt_factory.h"

namespace amr_dispatcher_bt {

using namespace amr_dispatcher_core::workflow;
using namespace amr_dispatcher_core::facility;
using namespace amr_dispatcher_core::catalog;

namespace {

struct MissionResultBtContext {
  MissionResultWorkflowInput input;
  MissionResultBehaviorTreeResult result;
};

bool IsCanceled(const MissionResultWorkflowInput& input) {
  return input.code == MissionActionResultCode::kCanceled || input.cancellation_requested;
}

bool IsSucceeded(const MissionResultWorkflowInput& input) {
  return input.code == MissionActionResultCode::kSucceeded && input.result_success;
}

BT::NodeStatus PlanBranch(MissionResultBtContext& context, const std::string& branch) {
  context.result.decision = PlanMissionResultWorkflow(context.input);
  context.result.branch = context.result.decision.branch;
  return context.result.branch == branch ? BT::NodeStatus::SUCCESS
                                         : BT::NodeStatus::FAILURE;
}

}  // namespace

std::string DefaultMissionResultBehaviorTreeXml() {
  return R"(
<root BTCPP_format="4" main_tree_to_execute="MissionResult">
  <BehaviorTree ID="MissionResult">
    <Fallback>
      <Sequence>
        <MissionWasCanceled/>
        <PlanCanceledMission/>
      </Sequence>
      <Sequence>
        <MissionSucceeded/>
        <PlanSucceededMission/>
      </Sequence>
      <PlanFailedMission/>
    </Fallback>
  </BehaviorTree>
</root>
)";
}

MissionResultBehaviorTreeResult TickMissionResultBehaviorTree(
    const MissionResultWorkflowInput& input) {
  MissionResultBtContext context;
  context.input = input;

  BT::BehaviorTreeFactory factory;
  factory.registerSimpleCondition(
      "MissionWasCanceled", [&context](BT::TreeNode&) {
        return IsCanceled(context.input) ? BT::NodeStatus::SUCCESS
                                         : BT::NodeStatus::FAILURE;
      });
  factory.registerSimpleAction(
      "PlanCanceledMission", [&context](BT::TreeNode&) {
        return PlanBranch(context, "canceled");
      });
  factory.registerSimpleCondition(
      "MissionSucceeded", [&context](BT::TreeNode&) {
        return IsSucceeded(context.input) ? BT::NodeStatus::SUCCESS
                                          : BT::NodeStatus::FAILURE;
      });
  factory.registerSimpleAction(
      "PlanSucceededMission", [&context](BT::TreeNode&) {
        return PlanBranch(context, "succeeded");
      });
  factory.registerSimpleAction(
      "PlanFailedMission", [&context](BT::TreeNode&) {
        return PlanBranch(context, "failed");
      });

  auto tree = factory.createTreeFromText(DefaultMissionResultBehaviorTreeXml());
  context.result.success = tree.tickOnce() == BT::NodeStatus::SUCCESS;
  return context.result;
}

}  // namespace amr_dispatcher_bt
