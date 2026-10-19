#pragma once

#include <cstdint>
#include <string>

#include "amr_dispatcher_core/dispatcher/mission_state.hpp"

namespace amr_dispatcher_ros {

// 任务 id / sequence 统一戳记（B4）：
// 修复 Enqueue/Preempt/dock 三处 id 用了 n+1 而 sequence 用了 n 的错位。
// 统一为：id 与 sequence 取自同一次自增的 n，保证 m.id == "m_<m.sequence>" 恒成立。
struct MissionIdentity {
  std::string id;
  std::uint64_t sequence = 0;
};

inline MissionIdentity NextMissionIdentity(std::uint64_t& next_sequence) {
  const std::uint64_t n = next_sequence++;
  return MissionIdentity{"m_" + std::to_string(n), n};
}

// 应用到 Mission：id 前缀可定制（默认 "m_"，回坞等场景复用同一路径）
inline void StampMissionIdentity(amr_dispatcher_core::dispatcher::Mission& m,
                                 std::uint64_t& next_sequence,
                                 const std::string& id_prefix = "m_") {
  const std::uint64_t n = next_sequence++;
  m.id = id_prefix + std::to_string(n);
  m.sequence = n;
}

}  // namespace amr_dispatcher_ros
