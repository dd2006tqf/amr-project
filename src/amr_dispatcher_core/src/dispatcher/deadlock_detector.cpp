#include "amr_dispatcher_core/dispatcher/deadlock_detector.hpp"

#include <algorithm>
#include <unordered_map>

namespace amr_dispatcher_core::dispatcher {

DeadlockDetector::DeadlockDetector(DeadlockDetectorConfig config)
    : config_(config) {}

std::vector<DeadlockFinding> DeadlockDetector::Detect(
    const DeadlockSnapshot& snapshot) const {
  std::vector<DeadlockFinding> findings;
  const auto now = std::chrono::steady_clock::now();

  // 1) 活跃任务过多
  if (config_.max_active_missions > 0 &&
      snapshot.active_missions.size() > config_.max_active_missions) {
    DeadlockFinding f;
    f.kind = DeadlockKind::kMutualBlock;
    f.description = "too many active missions: " +
                    std::to_string(snapshot.active_missions.size());
    for (const auto& m : snapshot.active_missions) {
      f.mission_ids.push_back(m.id);
    }
    findings.push_back(std::move(f));
  }

  // 2) ACTIVE 但无 progress 超过阈值
  for (const auto& m : snapshot.active_missions) {
    auto it = snapshot.last_progress.find(m.id);
    if (it == snapshot.last_progress.end()) {
      continue;
    }
    if (now - it->second > config_.active_idle_threshold) {
      DeadlockFinding f;
      f.kind = DeadlockKind::kIdleActive;
      f.description = "mission idle too long: " + m.id;
      f.mission_ids.push_back(m.id);
      findings.push_back(std::move(f));
    }
  }

  // 3) 长持资源（潜在循环等待）
  std::unordered_map<std::string, std::vector<std::string>> resource_to_missions;
  for (const auto& m : snapshot.active_missions) {
    if (!m.reservation_id.empty()) {
      resource_to_missions[m.reservation_id].push_back(m.id);
    }
  }
  for (const auto& [resource, acquired_at] : snapshot.reservation_acquired_at) {
    if (now - acquired_at <= config_.reservation_hold_threshold) {
      continue;
    }
    auto it = resource_to_missions.find(resource);
    if (it == resource_to_missions.end() || it->second.empty()) {
      continue;
    }
    DeadlockFinding f;
    f.kind = DeadlockKind::kLongReservationHold;
    f.description = "resource held too long: " + resource;
    f.mission_ids = it->second;
    findings.push_back(std::move(f));
  }

  return findings;
}

}  // namespace amr_dispatcher_core::dispatcher
