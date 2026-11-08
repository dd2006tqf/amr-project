#include "amr_dispatcher_core/workflow/fleet_link_guard.hpp"

#include "amr_dispatcher_core/workflow/dynamic_unlocking.hpp"

namespace amr_dispatcher_core::workflow {

FleetLinkGuardDecision EvaluateFleetLinkGuard(const FleetLinkGuardRequest& request) {
  FleetLinkGuardDecision decision;
  decision.verdicts.reserve(request.vehicles.size());

  for (const auto& vehicle : request.vehicles) {
    VehicleLinkVerdict verdict;
    verdict.vehicle_id = vehicle.vehicle_id;

    if (vehicle.quarantined) {
      // 隔离态车辆已被移出调度池：它的离线是**已知且已处置**的状态，
      // 不再挂起全队派发（否则人工强清后车队仍然冻结，隔离就失去意义）。
      // 恢复需显式调用 RestoreVda5050Link（并核实物理位置）。
      verdict.kind = VehicleLinkVerdict::Kind::kSkip;
      decision.verdicts.push_back(std::move(verdict));
      continue;
    }

    const bool stale = vehicle.online && vehicle.seen_state &&
                       vehicle.state_age_sec > request.state_stale_sec;
    // 复用 core 里既有的单车间守卫决策，保证"消息文案/铁律"只有一处定义
    const auto guard = EvaluateVehicleLinkGuard(vehicle.online, stale);
    verdict.message = guard.message;
    // 铁律：无论哪条分支，都不产生释放动作
    decision.retain_all_locks = decision.retain_all_locks && guard.retain_all_locks;

    if (!vehicle.online) {
      verdict.kind = VehicleLinkVerdict::Kind::kSuspendDispatch;
      // 任一受管离线车即挂起新派发（单车 /plan 执行模型）
      decision.block_new_dispatch = true;
    } else if (stale) {
      verdict.kind = VehicleLinkVerdict::Kind::kStaleWarning;
    } else {
      verdict.kind = VehicleLinkVerdict::Kind::kHealthy;
    }
    decision.verdicts.push_back(std::move(verdict));
  }
  return decision;
}

}  // namespace amr_dispatcher_core::workflow
