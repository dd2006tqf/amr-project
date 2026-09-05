#pragma once

#include <chrono>
#include <cstddef>
#include <string>
#include <unordered_map>

#include "amr_dispatcher_core/safety/safety_sources.hpp"

namespace amr_dispatcher_core::safety {

// 看门狗配置
struct WatchdogConfig {
  std::chrono::milliseconds heartbeat_timeout{std::chrono::milliseconds(500)};
  std::chrono::milliseconds soft_timeout{std::chrono::milliseconds(2000)};
};

// 看门狗：跟踪一组节点的心跳。Inspect 返回 warn/critical 事件列表。
class SafetyWatchdog {
 public:
  explicit SafetyWatchdog(WatchdogConfig config = {});

  void FeedHeartbeat(const std::string& node);
  void Forget(const std::string& node);

  std::vector<SafetyEvent> Inspect(
      const std::vector<std::string>& watched_nodes) const;

 private:
  WatchdogConfig config_;
  std::unordered_map<std::string, std::chrono::steady_clock::time_point> last_seen_;
};

}  // namespace amr_dispatcher_core::safety
