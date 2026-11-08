
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

#include "amr_dispatcher_bt/station_sequence_behavior_tree.hpp"

#include <cstddef>
#include <string>

#include "behaviortree_cpp/bt_factory.h"

namespace amr_dispatcher_bt {

using namespace amr_dispatcher_core::workflow;
using namespace amr_dispatcher_core::facility;
using namespace amr_dispatcher_core::catalog;

namespace {

struct StationSequenceBtContext {
  StationSequenceBehaviorTreeInput input;
  StationSequenceBehaviorTreeResult result;
};

StationSequenceBtContext MakeStationSequenceContext(
    const StationSequenceBehaviorTreeInput& input) {
  StationSequenceBtContext context;
  context.input = input;
  context.result.message = "station sequence accepted";
  return context;
}

MissionPreflightResult DefaultAllowedPreflight() {
  MissionPreflightResult preflight;
  preflight.allowed = true;
  preflight.message = "allowed";
  return preflight;
}

}  // namespace

std::string DefaultStationSequenceBehaviorTreeXml() {
  return R"(
<root BTCPP_format="4" main_tree_to_execute="StationSequenceSubflow">
  <BehaviorTree ID="StationSequenceSubflow">
    <Sequence>
      <CheckStationSequenceConflicts/>
      <CheckStationSequencePreflight/>
      <BuildStationSequenceQueue/>
    </Sequence>
  </BehaviorTree>
</root>
)";
}

StationSequenceBehaviorTreeResult TickStationSequenceBehaviorTree(
    const StationSequenceBehaviorTreeInput& input) {
  auto context = MakeStationSequenceContext(input);

  BT::BehaviorTreeFactory factory;
  factory.registerSimpleCondition(
      "CheckStationSequenceConflicts", [&context](BT::TreeNode&) {
        const auto result = CheckStationSequenceConflicts(
            context.input.legs, context.input.active_or_queued_mission_ids);
        context.result.message = result.message;
        return result.accepted ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
      });
  factory.registerSimpleCondition(
      "CheckStationSequencePreflight", [&context](BT::TreeNode&) {
        for (std::size_t index = 0; index < context.input.legs.size(); ++index) {
          const auto preflight = index < context.input.preflights.size()
                                     ? context.input.preflights[index]
                                     : DefaultAllowedPreflight();
          const auto result =
              CheckStationSequenceLegPreflight(context.input.legs[index], preflight);
          if (!result.accepted) {
            context.result.message = result.message;
            return BT::NodeStatus::FAILURE;
          }
        }
        return BT::NodeStatus::SUCCESS;
      });
  factory.registerSimpleAction(
      "BuildStationSequenceQueue", [&context](BT::TreeNode&) {
        context.result.mission_ids = StationSequenceMissionIds(context.input.legs);
        context.result.queued_missions = BuildStationSequenceQueuedMissions(
            context.input.legs, context.input.priority, context.input.first_sequence);
        context.result.branch = "station_sequence_queue";
        context.result.message =
            BuildStationSequenceQueuedMessage(context.result.queued_missions.size());
        return BT::NodeStatus::SUCCESS;
      });

  auto tree = factory.createTreeFromText(DefaultStationSequenceBehaviorTreeXml());
  context.result.success = tree.tickOnce() == BT::NodeStatus::SUCCESS;
  return context.result;
}

}  // namespace amr_dispatcher_bt
