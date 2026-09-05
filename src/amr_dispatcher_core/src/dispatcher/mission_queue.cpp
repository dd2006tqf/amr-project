#include "amr_dispatcher_core/dispatcher/mission_queue.hpp"

#include <algorithm>
#include <utility>

namespace amr_dispatcher_core::dispatcher {
namespace {

std::unique_ptr<MissionComparator> DefaultComparator(const std::string& name) {
  auto comparator = CreateMissionComparator(name);
  if (comparator == nullptr) {
    comparator = std::make_unique<PriorityFifoComparator>();
  }
  return comparator;
}

}  // namespace

MissionQueue::MissionQueue(Config config)
    : config_(config), comparator_(DefaultComparator(config.comparator)) {
  config_.max_size = std::max<std::size_t>(1, config_.max_size);
}

void MissionQueue::SetComparator(std::unique_ptr<MissionComparator> comparator) {
  if (comparator == nullptr) {
    return;
  }
  comparator_ = std::move(comparator);
  dirty_ = true;
}

void MissionQueue::Sort() const {
  if (!dirty_) {
    return;
  }
  auto& items = const_cast<std::vector<Mission>&>(items_);
  const MissionComparator& cmp = *comparator_;
  std::stable_sort(items.begin(), items.end(),
                   [&cmp](const Mission& a, const Mission& b) { return cmp.Before(a, b); });
  dirty_ = false;
}

MissionQueue::PushResult MissionQueue::Push(Mission mission) {
  PushResult result;
  if (mission.id.empty()) {
    result.message = "mission id must not be empty";
    return result;
  }
  if (Contains(mission.id)) {
    result.message = "duplicate mission id: " + mission.id;
    return result;
  }
  if (items_.size() >= config_.max_size) {
    result.message = "mission queue is full (" + std::to_string(config_.max_size) + ")";
    return result;
  }
  mission.state = MissionState::kPending;
  mission.result = MissionResult::kNone;
  items_.push_back(std::move(mission));
  dirty_ = true;
  result.accepted = true;
  result.message = "queued " + items_.back().id;
  return result;
}

MissionQueue::PopResult MissionQueue::PopNext() {
  PopResult result;
  Sort();
  if (items_.empty()) {
    result.queue_empty = true;
    result.message = "mission queue is empty";
    return result;
  }
  const auto it = std::find_if(items_.begin(), items_.end(), [this](const Mission& m) {
    return !IsOrderPaused(m.order_id);
  });
  if (it == items_.end()) {
    result.all_paused = true;
    result.message = "all queued missions are paused";
    return result;
  }
  result.mission = *it;
  result.mission->state = MissionState::kActive;
  result.success = true;
  result.message = "dispatched " + it->id;
  items_.erase(it);
  return result;
}

const Mission* MissionQueue::Peek() const {
  Sort();
  const auto it = std::find_if(items_.begin(), items_.end(), [this](const Mission& m) {
    return !IsOrderPaused(m.order_id);
  });
  return it == items_.end() ? nullptr : &*it;
}

bool MissionQueue::Cancel(const std::string& mission_id) {
  const auto it = std::find_if(items_.begin(), items_.end(),
                               [&mission_id](const Mission& m) { return m.id == mission_id; });
  if (it == items_.end()) {
    return false;
  }
  const std::string order_id = it->order_id;
  items_.erase(it);
  if (!order_id.empty() && IsOrderPaused(order_id)) {
    // 该订单已无排队任务时清掉暂停标记，避免残留。
    const bool still_queued =
        std::any_of(items_.begin(), items_.end(),
                    [&order_id](const Mission& m) { return m.order_id == order_id; });
    if (!still_queued) {
      paused_order_ids_.erase(order_id);
    }
  }
  return true;
}

bool MissionQueue::Reprioritize(const std::string& mission_id, const int priority) {
  const auto it = std::find_if(items_.begin(), items_.end(),
                               [&mission_id](const Mission& m) { return m.id == mission_id; });
  if (it == items_.end()) {
    return false;
  }
  it->priority = priority;
  dirty_ = true;
  return true;
}

void MissionQueue::PauseOrder(const std::string& order_id) {
  if (!order_id.empty()) {
    paused_order_ids_.insert(order_id);
  }
}

void MissionQueue::ResumeOrder(const std::string& order_id) { paused_order_ids_.erase(order_id); }

bool MissionQueue::IsOrderPaused(const std::string& order_id) const {
  return !order_id.empty() && paused_order_ids_.count(order_id) > 0;
}

std::size_t MissionQueue::CancelOrder(const std::string& order_id) {
  if (order_id.empty()) {
    return 0;
  }
  const auto before = items_.size();
  items_.erase(
      std::remove_if(items_.begin(), items_.end(),
                     [&order_id](const Mission& m) { return m.order_id == order_id; }),
      items_.end());
  paused_order_ids_.erase(order_id);
  return before - items_.size();
}

bool MissionQueue::Contains(const std::string& mission_id) const {
  return std::any_of(items_.begin(), items_.end(),
                     [&mission_id](const Mission& m) { return m.id == mission_id; });
}

std::vector<Mission> MissionQueue::Snapshot() const {
  Sort();
  std::vector<Mission> snapshot;
  snapshot.reserve(items_.size());
  for (const auto& item : items_) {
    Mission copy = item;
    copy.state = IsOrderPaused(item.order_id) ? MissionState::kPaused : MissionState::kPending;
    snapshot.push_back(std::move(copy));
  }
  return snapshot;
}

}  // namespace amr_dispatcher_core::dispatcher
