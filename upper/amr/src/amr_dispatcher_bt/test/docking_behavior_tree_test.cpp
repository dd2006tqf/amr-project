
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

#include "amr_dispatcher_bt/docking_behavior_tree.hpp"

#include <gtest/gtest.h>

namespace amr_dispatcher_bt {

using namespace amr_dispatcher_core::workflow;
using namespace amr_dispatcher_core::facility;
using namespace amr_dispatcher_core::catalog;


TEST(DockingBehaviorTreeTest, SuccessfulContact_CompletesDockingThroughBtAction) {
  DockingBehaviorTreeInput input;
  input.dock_state = DockingRuntimeState{"dock_a", "mission_a", "APPROACHING", true, 0};
  input.resource_id = "charger_a";

  const auto result = TickDockingBehaviorTree(input);

  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.branch, "complete_docking");
  EXPECT_EQ(result.transition.event_state, "CHARGING");
  EXPECT_EQ(result.dock_state.state, "CHARGING");
  EXPECT_EQ(result.reservation.status, "CHARGING");
}

TEST(DockingBehaviorTreeTest, FirstContactFailure_StagesRetryThroughBtBranch) {
  DockingBehaviorTreeInput input;
  input.dock_state = DockingRuntimeState{"dock_a", "mission_a", "APPROACHING", false, 0};
  input.resource_id = "charger_a";

  const auto result = TickDockingBehaviorTree(input);

  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.branch, "contact_retry");
  EXPECT_EQ(result.transition.event_state, "DOCKING_CONTACT_FAILED");
  EXPECT_EQ(result.dock_state.state, "FAILED");
  EXPECT_EQ(result.dock_state.contact_attempts, 1);
  EXPECT_TRUE(result.dock_state.simulate_contact_success);
  EXPECT_EQ(result.reservation.status, "DOCKING_CONTACT_FAILED");
}

}  // namespace amr_dispatcher_bt
