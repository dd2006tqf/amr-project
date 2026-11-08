#include "amr_dispatcher_core/fleet/vda5050_link_state.hpp"

namespace amr_dispatcher_core::fleet {

LinkStateApplyAction ApplyStateObservation(
    Vda5050LinkState* state, const Vda5050StateObservation& observation,
    std::chrono::steady_clock::time_point now) {
  if (state == nullptr) {
    return LinkStateApplyAction::kUpdated;
  }
  state->seen_state = true;
  state->last_state_time = now;
  state->position_valid = observation.position_initialized;
  state->position_x = observation.x;
  state->position_y = observation.y;
  state->battery_percent = observation.battery_percent;
  // 离线期间收到 state 绝不可回置 online（见头文件铁律）：只更新数据。
  return state->online ? LinkStateApplyAction::kUpdated
                       : LinkStateApplyAction::kUpdatedWhileOffline;
}

bool ApplyOnlineReport(Vda5050LinkState* state, bool online) {
  if (state == nullptr) {
    return false;
  }
  if (state->online == online) {
    return false;  // 状态未变（transient_local 重放/重复报文）：调用方不重复告警
  }
  state->online = online;
  return true;
}

bool IsStateStale(const Vda5050LinkState& state, std::chrono::steady_clock::time_point now,
                  std::chrono::milliseconds stale_threshold) {
  if (!state.online || !state.seen_state) {
    return false;  // 从未收到 state 的在线车没有"多久没更新"可比
  }
  return (now - state.last_state_time) > stale_threshold;
}

}  // namespace amr_dispatcher_core::fleet
