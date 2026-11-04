#include "amr_dispatcher_core/safety/watchdog.hpp"

#include <utility>

namespace amr_dispatcher_core::safety {

SafetyWatchdog::SafetyWatchdog(WatchdogConfig config) : config_(std::move(config)) {}

void SafetyWatchdog::FeedHeartbeat(const std::string& node) {
  last_seen_[node] = std::chrono::steady_clock::now();
}

void SafetyWatchdog::Forget(const std::string& node) {
  last_seen_.erase(node);
}

std::vector<SafetyEvent> SafetyWatchdog::Inspect(
    const std::vector<std::string>& watched_nodes) const {
  std::vector<SafetyEvent> events;
  const auto now = std::chrono::steady_clock::now();
  for (const auto& node : watched_nodes) {
    auto it = last_seen_.find(node);
    if (it == last_seen_.end()) {
      SafetyEvent e;
      e.source = "watchdog/" + node;
      e.severity = SafetySeverity::kCritical;
      e.message = "no heartbeat ever seen";
      e.stamp = now;
      events.push_back(std::move(e));
      continue;
    }
    const auto since = std::chrono::duration_cast<std::chrono::milliseconds>(now - it->second);
    SafetyEvent e;
    e.source = "watchdog/" + node;
    e.stamp = now;
    if (since >= config_.soft_timeout) {
      e.severity = SafetySeverity::kCritical;
      e.message = "heartbeat soft timeout " + std::to_string(since.count()) + "ms (>= " +
                  std::to_string(config_.soft_timeout.count()) + "ms)";
    } else if (since >= config_.heartbeat_timeout) {
      // T9：这条只是**丢拍**（限速），不是失联。日志里必须能一眼看出区别，
      // 否则现场看到 "heartbeat_timeout" 会以为安全门已经急停了。
      e.severity = SafetySeverity::kWarn;
      e.message = "heartbeat missed " + std::to_string(since.count()) + "ms (warn at " +
                  std::to_string(config_.heartbeat_timeout.count()) + "ms, soft at " +
                  std::to_string(config_.soft_timeout.count()) + "ms)";
    } else {
      continue;  // 健康：不产生事件
    }
    events.push_back(std::move(e));
  }
  return events;
}

}  // namespace amr_dispatcher_core::safety
