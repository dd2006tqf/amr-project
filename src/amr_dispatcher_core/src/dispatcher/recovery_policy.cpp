#include "amr_dispatcher_core/dispatcher/recovery_policy.hpp"

#include <unordered_set>

namespace amr_dispatcher_core::dispatcher {

RecoveryPolicy::RecoveryPolicy(RecoveryPolicyConfig config)
    : config_(config) {}

std::vector<RecoveryAction> RecoveryPolicy::Decide(
    const std::vector<DeadlockFinding>& findings,
    const std::function<std::size_t(const std::string&)>& attempt_of) const {
  std::vector<RecoveryAction> actions;
  std::unordered_set<std::string> seen;

  for (const auto& f : findings) {
    for (const auto& mid : f.mission_ids) {
      if (!seen.insert(mid).second) {
        continue;
      }
      const auto attempts = attempt_of ? attempt_of(mid) : 0;
      RecoveryAction action;
      action.mission_id = mid;
      action.reason = f.description;

      switch (f.kind) {
        case DeadlockKind::kIdleActive:
        case DeadlockKind::kLongReservationHold:
          if (attempts < config_.max_retries) {
            action.kind = RecoveryAction::Kind::kRetry;
          } else if (config_.auto_cancel_on_exhausted) {
            action.kind = RecoveryAction::Kind::kCancel;
          } else {
            action.kind = RecoveryAction::Kind::kPause;
          }
          break;
        case DeadlockKind::kMutualBlock:
          action.kind = RecoveryAction::Kind::kPause;
          break;
        case DeadlockKind::kOverCapacity:
          // A8：超容量是派发过载不是死锁 —— 不 pause 活跃任务（暂缓派发由
          // 派发侧限流负责），动作枚举中最贴近 hold 的处置是 kNone
          action.kind = RecoveryAction::Kind::kNone;
          break;
      }
      actions.push_back(std::move(action));
    }
  }
  return actions;
}

}  // namespace amr_dispatcher_core::dispatcher
