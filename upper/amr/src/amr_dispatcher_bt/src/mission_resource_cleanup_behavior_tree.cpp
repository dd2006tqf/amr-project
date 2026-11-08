
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

#include "amr_dispatcher_bt/mission_resource_cleanup_behavior_tree.hpp"

#include "behaviortree_cpp/bt_factory.h"

namespace amr_dispatcher_bt {

using namespace amr_dispatcher_core::workflow;
using namespace amr_dispatcher_core::facility;
using namespace amr_dispatcher_core::catalog;

namespace {

struct MissionResourceCleanupBtContext {
  MissionResourceCleanupBehaviorTreeInput input;
  MissionResourceCleanupBehaviorTreeResult result;
};

}  // namespace

std::string DefaultMissionResourceCleanupBehaviorTreeXml() {
  return R"(
<root BTCPP_format="4" main_tree_to_execute="MissionResourceCleanup">
  <BehaviorTree ID="MissionResourceCleanup">
    <Sequence>
      <ReleaseMissionResources/>
      <Fallback>
        <Sequence>
          <AnyResourceReleased/>
          <StageReleased/>
        </Sequence>
        <StageNoop/>
      </Fallback>
    </Sequence>
  </BehaviorTree>
</root>
)";
}

MissionResourceCleanupBehaviorTreeResult TickMissionResourceCleanupBehaviorTree(
    MissionResourceCleanupBehaviorTreeInput input) {
  MissionResourceCleanupBtContext context{input, MissionResourceCleanupBehaviorTreeResult{}};

  BT::BehaviorTreeFactory factory;
  factory.registerSimpleAction(
      "ReleaseMissionResources", [&context](BT::TreeNode&) {
        context.result.cleanup = ReleaseMissionResources(
            context.input.facility_reservations, context.input.facility_resource_by_mission,
            context.input.route_locks, context.input.route_locks_by_mission,
            context.input.mission_id);
        return BT::NodeStatus::SUCCESS;
      });
  factory.registerSimpleCondition(
      "AnyResourceReleased", [&context](BT::TreeNode&) {
        return (context.result.cleanup.facility_resource_released ||
                context.result.cleanup.route_locks_released)
                   ? BT::NodeStatus::SUCCESS
                   : BT::NodeStatus::FAILURE;
      });
  factory.registerSimpleAction(
      "StageReleased", [&context](BT::TreeNode&) {
        context.result.branch = "released";
        return BT::NodeStatus::SUCCESS;
      });
  factory.registerSimpleAction(
      "StageNoop", [&context](BT::TreeNode&) {
        context.result.branch = "noop";
        return BT::NodeStatus::SUCCESS;
      });

  auto tree = factory.createTreeFromText(DefaultMissionResourceCleanupBehaviorTreeXml());
  context.result.success = tree.tickOnce() == BT::NodeStatus::SUCCESS;
  return context.result;
}

}  // namespace amr_dispatcher_bt
