// 主动告警的状态机与载荷渲染（纯逻辑，零 IO、零线程、零时钟）。
//
// 为什么必须有"状态机"而不是"条件满足就发一次"：
//   探针每秒跑一次。`/readyz` 连续 3 秒不就绪是事件，连续 8 小时不就绪是同一个
//   事件——但它会把 webhook 通道刷爆，把真正的新故障淹掉。所以这里做三件事：
//     1. **连续 N 次才升级**（过滤单拍抖动，DDS 重连、一轮 GC 都会造成单拍延迟）；
//     2. **firing 期间限流重复提醒**（`min_interval_sec`）；
//     3. **恢复只发一次且不受限流**（"好了"必须准时到达，否则值班人员会去现场白跑）。
//
// 载荷 schema 固定为 `amr.alert.v1`：接收端（企业微信/钉钉/自建通知中心）可以按
// `type` / `state` / `severity` 三个字段做路由，不必解析自由文本。
#pragma once

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <map>
#include <string>
#include <vector>

#include "amr_dispatcher_tools/json_escape.hpp"

namespace amr_dispatcher_tools {

enum class AlertType {
  kReadinessNotReady,
  kReadinessRecovered,
  kSafetyCritical,
  kSafetyRecovered,
  kComponentMissing,
  kComponentRecovered,
  kTopologyOverlayInvalidated,
};

inline const char* AlertTypeName(AlertType type) {
  switch (type) {
    case AlertType::kReadinessNotReady:
      return "readiness_not_ready";
    case AlertType::kReadinessRecovered:
      return "readiness_recovered";
    case AlertType::kSafetyCritical:
      return "safety_critical";
    case AlertType::kSafetyRecovered:
      return "safety_recovered";
    case AlertType::kComponentMissing:
      return "component_missing";
    case AlertType::kComponentRecovered:
      return "component_recovered";
    case AlertType::kTopologyOverlayInvalidated:
      return "topology_overlay_invalidated";
  }
  return "unknown";
}

struct AlertEvent {
  AlertType type = AlertType::kReadinessNotReady;
  std::string severity = "warning";  // critical | warning | info
  std::string state = "firing";      // firing | resolved
  std::string dedup_key;
  std::string summary;
  std::int64_t fired_at_unix = 0;
  std::int64_t resolved_at_unix = 0;
  std::uint32_t consecutive_observations = 0;
  std::string details_json;  // 已渲染好的 JSON 对象片段（"{}" 或 "{\"k\":\"v\"}"）
};

struct AlertPolicy {
  // 连续多少次不就绪才告警。探针 1Hz -> 3 次 ≈ 3 秒，足以滤掉单拍抖动。
  std::uint32_t fire_after_consecutive_failures = 3;
  // firing 期间的重复提醒间隔（秒）。0 = 只提醒一次。
  std::int64_t min_interval_sec = 300;
  // 安全门 CRITICAL 持续多久才告警（秒）。急停是常驻状态，瞬时出现不该报警。
  std::int64_t safety_critical_sec = 30;
};

// 一次探针巡检的观测量（由网关组装，本层不读任何全局状态）。
struct AlertObservation {
  std::int64_t now_unix = 0;
  bool ready = true;
  bool safety_critical = false;
  // 语义健康的组件名缺失列表（stale / never_seen / unhealthy）。
  // 只用于生成 component_missing:<name> 的 dedup_key，不参与聚合计时。
  std::vector<std::string> missing_components;
  std::string details_json = "{}";
};

class AlertStateMachine {
 public:
  explicit AlertStateMachine(AlertPolicy policy = {}) : policy_(policy) {}

  // 吃一次观测，产出本次要真正发出去的事件（0..N 条，顺序稳定）。
  std::vector<AlertEvent> Observe(const AlertObservation& observation) {
    std::vector<AlertEvent> out;

    // ---- 1. 整体就绪 ----
    if (!observation.ready) {
      auto& slot = slots_["readiness_not_ready"];
      ++slot.consecutive;
      if (!slot.firing && slot.consecutive >= policy_.fire_after_consecutive_failures) {
        slot.firing = true;
        slot.fired_at = observation.now_unix;
        slot.last_sent = observation.now_unix;
        out.push_back(MakeEvent(AlertType::kReadinessNotReady, "critical", "firing",
                                "readiness_not_ready",
                                "AMR dispatcher not ready: " + observation.details_json,
                                observation, slot.consecutive));
      } else if (slot.firing && policy_.min_interval_sec > 0 &&
                 observation.now_unix - slot.last_sent >= policy_.min_interval_sec) {
        slot.last_sent = observation.now_unix;
        out.push_back(MakeEvent(AlertType::kReadinessNotReady, "critical", "firing",
                                "readiness_not_ready",
                                "AMR dispatcher still not ready: " + observation.details_json,
                                observation, slot.consecutive));
      }
    } else {
      auto it = slots_.find("readiness_not_ready");
      if (it != slots_.end()) {
        if (it->second.firing) {
          AlertEvent event = MakeEvent(AlertType::kReadinessRecovered, "info", "resolved",
                                       "readiness_not_ready", "AMR dispatcher is ready again",
                                       observation, it->second.consecutive);
          event.fired_at_unix = it->second.fired_at;
          out.push_back(event);
        }
        // 未 firing 也要清 slot：否则"两次失败 + 一次成功 + 一次失败"会立刻越过
        // 连续失败阈值报警。抖动的计数必须真的归零。
        slots_.erase(it);
      }
    }

    // ---- 2. 安全门 CRITICAL 持续 ----
    if (observation.safety_critical) {
      auto& slot = slots_["safety_critical"];
      if (slot.consecutive == 0) {
        slot.first_trigger = observation.now_unix;
      }
      ++slot.consecutive;
      const bool sustained =
          observation.now_unix - slot.first_trigger >= policy_.safety_critical_sec;
      if (!slot.firing && sustained) {
        slot.firing = true;
        slot.last_sent = observation.now_unix;
        AlertEvent event =
            MakeEvent(AlertType::kSafetyCritical, "critical", "firing", "safety_critical",
                      "Safety gate latched CRITICAL for " +
                          std::to_string(observation.now_unix - slot.first_trigger) + "s: " +
                          observation.details_json,
                      observation, slot.consecutive);
        // 记首次出现 CRITICAL 的时刻（而不是报警时刻）：持续时长才是这条告警的含义
        event.fired_at_unix = slot.first_trigger;
        slot.fired_at = slot.first_trigger;
        out.push_back(event);
      } else if (slot.firing && policy_.min_interval_sec > 0 &&
                 observation.now_unix - slot.last_sent >= policy_.min_interval_sec) {
        slot.last_sent = observation.now_unix;
        out.push_back(MakeEvent(AlertType::kSafetyCritical, "critical", "firing",
                                "safety_critical",
                                "Safety gate still CRITICAL: " + observation.details_json,
                                observation, slot.consecutive));
      }
    } else {
      auto it = slots_.find("safety_critical");
      if (it != slots_.end() && it->second.firing) {
        AlertEvent event = MakeEvent(AlertType::kSafetyRecovered, "info", "resolved",
                                     "safety_critical", "Safety gate recovered",
                                     observation, it->second.consecutive);
        event.fired_at_unix = it->second.fired_at;
        out.push_back(event);
      }
      slots_.erase("safety_critical");
    }

    // ---- 3. 单组件缺失 ----
    // 每个组件一个独立 slot：调度器死了与底盘失联是两个不同故障，值班人员要能分别路由。
    for (const auto& name : observation.missing_components) {
      const std::string key = "component_missing:" + name;
      auto& slot = slots_[key];
      ++slot.consecutive;
      if (!slot.firing && slot.consecutive >= policy_.fire_after_consecutive_failures) {
        slot.firing = true;
        slot.fired_at = observation.now_unix;
        slot.last_sent = observation.now_unix;
        out.push_back(MakeEvent(AlertType::kComponentMissing, "warning", "firing", key,
                                "Component not healthy: " + name, observation,
                                slot.consecutive));
      } else if (slot.firing && policy_.min_interval_sec > 0 &&
                 observation.now_unix - slot.last_sent >= policy_.min_interval_sec) {
        slot.last_sent = observation.now_unix;
        out.push_back(MakeEvent(AlertType::kComponentMissing, "warning", "firing", key,
                                "Component still not healthy: " + name, observation,
                                slot.consecutive));
      }
    }
    // 恢复：所有 component_missing:* slot 里本拍不再缺失的
    {
      std::vector<std::string> recovered_keys;
      for (const auto& [key, slot] : slots_) {
        if (key.rfind("component_missing:", 0) != 0) {
          continue;
        }
        const std::string name = key.substr(std::string("component_missing:").size());
        const bool still_missing =
            std::find(observation.missing_components.begin(),
                      observation.missing_components.end(),
                      name) != observation.missing_components.end();
        if (still_missing) {
          continue;
        }
        if (slot.firing) {
          AlertEvent event = MakeEvent(AlertType::kComponentRecovered, "info", "resolved", key,
                                       "Component healthy again: " + name, observation,
                                       slot.consecutive);
          event.fired_at_unix = slot.fired_at;
          out.push_back(event);
        }
        recovered_keys.push_back(key);
      }
      for (const auto& key : recovered_keys) {
        slots_.erase(key);
      }
    }

    return out;
  }

  // 一次性事件（如叠加层被新地图作废）：不进状态机，直接产出。
  std::vector<AlertEvent> NotifyOnce(AlertType type, const std::string& severity,
                                     const std::string& dedup_key, const std::string& summary,
                                     const AlertObservation& observation) {
    std::vector<AlertEvent> out;
    out.push_back(MakeEvent(type, severity, "firing", dedup_key, summary, observation, 1));
    return out;
  }

  bool Firing(const std::string& dedup_key) const {
    const auto it = slots_.find(dedup_key);
    return it != slots_.end() && it->second.firing;
  }

  // 只读策略（启动日志要把它打出来，现场据此判断"为什么这次没收到告警"）
  const AlertPolicy& policy() const { return policy_; }

  std::size_t FiringCount() const {
    std::size_t count = 0;
    for (const auto& [_, slot] : slots_) {
      if (slot.firing) ++count;
    }
    return count;
  }

 private:
  struct Slot {
    bool firing = false;
    std::uint32_t consecutive = 0;
    std::int64_t first_trigger = 0;  // 连续计数起点（安全门持续时长判定用）
    std::int64_t last_sent = 0;
    std::int64_t fired_at = 0;
  };

  AlertEvent MakeEvent(AlertType type, const std::string& severity, const std::string& state,
                       const std::string& dedup_key, const std::string& summary,
                       const AlertObservation& observation,
                       std::uint32_t consecutive) const {
    AlertEvent event;
    event.type = type;
    event.severity = severity;
    event.state = state;
    event.dedup_key = dedup_key;
    event.summary = summary;
    event.consecutive_observations = consecutive;
    event.details_json = observation.details_json.empty() ? "{}" : observation.details_json;
    if (state == "resolved") {
      event.resolved_at_unix = observation.now_unix;
    } else {
      event.fired_at_unix = observation.now_unix;
    }
    return event;
  }

  AlertPolicy policy_;
  std::map<std::string, Slot> slots_;
};

// ISO-8601 UTC（秒精度）。告警接收入口普遍用它做时间轴，不用 unix 秒。
inline std::string FormatAlertTimestamp(std::int64_t unix_sec) {
  if (unix_sec <= 0) {
    return "";
  }
  const std::time_t seconds = static_cast<std::time_t>(unix_sec);
  std::tm utc{};
  gmtime_r(&seconds, &utc);
  char buf[32];
  std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &utc);
  return std::string(buf);
}

// 告警载荷（schema amr.alert.v1）。details 已是 JSON 片段，这里只做包裹与转义。
inline std::string RenderAlertPayload(const AlertEvent& event, const std::string& instance_host,
                                      int gateway_port, int ros_domain_id, long pid) {
  std::string out = "{";
  out += JsonStringPair("schema", "amr.alert.v1") + ",";
  out += JsonStringPair("type", AlertTypeName(event.type)) + ",";
  out += JsonStringPair("state", event.state) + ",";
  out += JsonStringPair("severity", event.severity) + ",";
  out += JsonStringPair("dedup_key", event.dedup_key) + ",";
  out += JsonStringPair("summary", event.summary) + ",";
  const std::string fired = FormatAlertTimestamp(event.fired_at_unix);
  out += "\"fired_at\":" +
         (fired.empty() ? std::string("null") : "\"" + JsonEscape(fired) + "\"") + ",";
  const std::string resolved = FormatAlertTimestamp(event.resolved_at_unix);
  out += "\"resolved_at\":" +
         (resolved.empty() ? std::string("null") : "\"" + JsonEscape(resolved) + "\"") + ",";
  out += "\"consecutive_observations\":" + std::to_string(event.consecutive_observations) + ",";
  out += "\"instance\":{";
  out += JsonStringPair("host", instance_host) + ",";
  out += "\"pid\":" + std::to_string(pid) + ",";
  out += "\"ros_domain_id\":" + std::to_string(ros_domain_id) + ",";
  out += "\"gateway_port\":" + std::to_string(gateway_port);
  out += "},";
  out += "\"details\":" + (event.details_json.empty() ? std::string("{}") : event.details_json);
  out += "}";
  return out;
}

}  // namespace amr_dispatcher_tools
