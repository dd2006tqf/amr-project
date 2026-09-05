#include "amr_dispatcher_core/dispatcher/mission_state.hpp"

#include <array>
#include <utility>

namespace amr_dispatcher_core::dispatcher {
namespace {

// 合法迁移矩阵：from → {允许 to 集合}。
// PENDING: →ACTIVE / →FINISHED(提交即取消)
// ACTIVE : →PAUSED / →FINISHED
// PAUSED : →ACTIVE(恢复) / →PENDING(撤回到队列头) / →FINISHED(暂停中取消)
// FINISHED: 终态，任何迁出都非法
constexpr std::array<std::pair<MissionState, MissionState>, 9> kAllowedTransitions{{
    {MissionState::kPending, MissionState::kActive},
    {MissionState::kPending, MissionState::kFinished},
    {MissionState::kActive, MissionState::kPaused},
    {MissionState::kActive, MissionState::kFinished},
    {MissionState::kPaused, MissionState::kActive},
    {MissionState::kPaused, MissionState::kPending},
    {MissionState::kPaused, MissionState::kFinished},
}};

}  // namespace

bool CanTransition(const MissionState from, const MissionState to) {
  if (from == to) {
    return false;
  }
  for (const auto& [src, dst] : kAllowedTransitions) {
    if (src == from && dst == to) {
      return true;
    }
  }
  return false;
}

std::optional<MissionState> ParseMissionState(const std::string_view text) {
  if (text == "PENDING") return MissionState::kPending;
  if (text == "ACTIVE") return MissionState::kActive;
  if (text == "PAUSED") return MissionState::kPaused;
  if (text == "FINISHED") return MissionState::kFinished;
  return std::nullopt;
}

std::optional<MissionResult> ParseMissionResult(const std::string_view text) {
  if (text == "NONE") return MissionResult::kNone;
  if (text == "SUCCEEDED") return MissionResult::kSucceeded;
  if (text == "FAILED") return MissionResult::kFailed;
  if (text == "CANCELED") return MissionResult::kCanceled;
  return std::nullopt;
}

std::string_view ToString(const MissionState state) {
  switch (state) {
    case MissionState::kPending: return "PENDING";
    case MissionState::kActive: return "ACTIVE";
    case MissionState::kPaused: return "PAUSED";
    case MissionState::kFinished: return "FINISHED";
  }
  return "PENDING";
}

std::string_view ToString(const MissionResult result) {
  switch (result) {
    case MissionResult::kNone: return "NONE";
    case MissionResult::kSucceeded: return "SUCCEEDED";
    case MissionResult::kFailed: return "FAILED";
    case MissionResult::kCanceled: return "CANCELED";
  }
  return "NONE";
}

MissionStateMachine::MissionStateMachine(Mission mission) : mission_(std::move(mission)) {}

bool MissionStateMachine::Transition(
    const MissionState to, const MissionResult result, std::string* reason) {
  if (!CanTransition(mission_.state, to)) {
    if (reason != nullptr) {
      *reason =
          "illegal transition " + std::string(ToString(mission_.state)) + " -> " +
          std::string(ToString(to));
    }
    return false;
  }
  if (to == MissionState::kFinished && result == MissionResult::kNone) {
    if (reason != nullptr) {
      *reason = "transition to FINISHED requires an explicit result";
    }
    return false;
  }
  if (to != MissionState::kFinished && result != MissionResult::kNone) {
    if (reason != nullptr) {
      *reason = "non-terminal transition must not carry a result";
    }
    return false;
  }
  if (mission_.state == MissionState::kFinished) {
    if (reason != nullptr) {
      *reason = "mission already finished";
    }
    return false;
  }

  last_transition_ = MissionTransition{mission_.state, to, {}};
  mission_.state = to;
  mission_.result = to == MissionState::kFinished ? result : MissionResult::kNone;
  ++transition_count_;
  return true;
}

bool MissionStateMachine::Finish(const MissionResult result, std::string* reason) {
  if (result == MissionResult::kNone) {
    if (reason != nullptr) {
      *reason = "Finish() requires a concrete result";
    }
    return false;
  }
  return Transition(MissionState::kFinished, result, reason);
}

}  // namespace amr_dispatcher_core::dispatcher
