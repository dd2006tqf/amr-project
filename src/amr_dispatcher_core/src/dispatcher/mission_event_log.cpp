#include "amr_dispatcher_core/dispatcher/mission_event_log.hpp"

#include <algorithm>
#include <utility>

namespace amr_dispatcher_core::dispatcher {
namespace {

std::size_t ClampCapacity(const std::size_t requested) {
  return std::clamp(requested, std::size_t{1}, MissionEventLog::kMaxCapacity);
}

}  // namespace

MissionEventLog::MissionEventLog(const std::size_t capacity)
    : capacity_(ClampCapacity(capacity)), slots_(capacity_) {}

std::size_t MissionEventLog::IndexFromOldest(const std::size_t offset) const {
  const std::size_t oldest = (head_ + capacity_ - size_) % capacity_;
  return (oldest + offset) % capacity_;
}

void MissionEventLog::Append(MissionEvent event) {
  slots_[head_] = std::move(event);
  head_ = (head_ + 1) % capacity_;
  if (size_ < capacity_) {
    ++size_;
  }
  ++total_appended_;
}

std::vector<MissionEvent> MissionEventLog::Recent(
    const std::size_t limit, const std::string& event_filter,
    const std::string& mission_id_filter) const {
  std::vector<MissionEvent> selected;
  if (limit == 0) {
    return selected;
  }
  selected.reserve(std::min(limit, size_));
  for (std::size_t offset = size_; offset > 0 && selected.size() < limit; --offset) {
    const MissionEvent& event = slots_[IndexFromOldest(offset - 1)];
    if (!event_filter.empty() && event.event != event_filter) {
      continue;
    }
    if (!mission_id_filter.empty() && event.mission_id != mission_id_filter) {
      continue;
    }
    selected.push_back(event);
  }
  return selected;
}

std::vector<MissionEvent> MissionEventLog::Snapshot() const {
  std::vector<MissionEvent> snapshot;
  snapshot.reserve(size_);
  for (std::size_t offset = 0; offset < size_; ++offset) {
    snapshot.push_back(slots_[IndexFromOldest(offset)]);
  }
  return snapshot;
}

void MissionEventLog::Clear() {
  head_ = 0;
  size_ = 0;
}

std::size_t MissionEventLog::NormalizeLimit(
    const int requested_limit, const std::size_t default_limit, const std::size_t max_limit) {
  const auto effective =
      requested_limit > 0 ? static_cast<std::size_t>(requested_limit) : default_limit;
  return std::min(effective, ClampCapacity(max_limit));
}

}  // namespace amr_dispatcher_core::dispatcher
