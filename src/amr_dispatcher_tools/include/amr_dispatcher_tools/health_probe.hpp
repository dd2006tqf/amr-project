// 就绪探针的纯逻辑层（零 ROS、零时钟）。
//
// 为什么单列一个头文件：`/healthz` 与 `/readyz` 的输出是**对编排系统的契约**
// （Docker HEALTHCHECK / k8s probe / 现场监控脚本都会按字段名与状态码判断），
// 一旦被依赖，改判据就是破坏性变更。所以：
//   * 判据（新鲜度阈值 + 语义健康）在这里以纯函数表达，由宿主机/CI 单测锁定；
//   * 时间的读取留在调用方（网关），本层只接收已算好的 `age_seconds`。
//
// 两条不可让渡的语义：
//   1. **从未收到 = 不就绪**（fail closed）。进程活着但不出声，比进程死掉更危险。
//   2. **未监控 ≠ 健康**。多车模式下网关订阅不到 per-vehicle 话题，此时组件标
//      `monitored=false` 并**不**计入失败，但 JSON 里如实标注 —— 不假装在监控。
#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "amr_dispatcher_tools/json_escape.hpp"

namespace amr_dispatcher_tools {

// 单个组件的观测结果。
struct ComponentProbe {
  std::string name;      // "dispatcher" / "safety" / "chassis" / "coordinator"
  bool monitored = true; // false = 该部署形态下无法观测（如多车模式下的 /chassis/link_health）
  bool seen = false;     // 是否至少收到过一次消息
  // 语义健康：dispatcher 在发状态 / safety.motion_allowed / chassis.is_healthy / ready_flag
  bool semantic_ok = false;
  double age_seconds = -1.0;         // <0 = 从未收到
  double stale_after_seconds = 3.0;  // 超过即视为失联
  std::string detail;                // 人读摘要（"CRITICAL" / "state_stale" / 空）
};

enum class ComponentStatus {
  kOk,           // 新鲜且语义健康
  kNeverSeen,    // 从未收到过消息
  kStale,        // 收到过但已超过阈值
  kUnhealthy,    // 新鲜但语义判负（CRITICAL / 未就绪 / 链路不健康）
  kNotMonitored  // 本部署形态无法观测
};

inline const char* ComponentStatusName(ComponentStatus status) {
  switch (status) {
    case ComponentStatus::kOk:
      return "ok";
    case ComponentStatus::kNeverSeen:
      return "never_seen";
    case ComponentStatus::kStale:
      return "stale";
    case ComponentStatus::kUnhealthy:
      return "unhealthy";
    case ComponentStatus::kNotMonitored:
      return "not_monitored";
  }
  return "unknown";
}

// 单个组件的判定。阈值取组件自带的 stale_after_seconds。
inline ComponentStatus EvaluateComponent(const ComponentProbe& probe) {
  if (!probe.monitored) {
    return ComponentStatus::kNotMonitored;
  }
  if (!probe.seen || probe.age_seconds < 0.0) {
    return ComponentStatus::kNeverSeen;
  }
  if (probe.age_seconds > probe.stale_after_seconds) {
    return ComponentStatus::kStale;
  }
  return probe.semantic_ok ? ComponentStatus::kOk : ComponentStatus::kUnhealthy;
}

struct ReadinessVerdict {
  bool ready = false;
  // READY / DEGRADED / INITIALIZING（沿用 lifecycle_coordinator 的词汇，
  // 便于现场同时看 /system/ready 与 /readyz 时不会出现两套说法）
  std::string state = "INITIALIZING";
  std::vector<ComponentProbe> components;
  std::string message;
};

// 汇总判定：
//   * 没有任何受监控组件    -> INITIALIZING（"什么都没监控"不能算就绪）
//   * 任一受监控组件 kNeverSeen  -> INITIALIZING（刚启动 / 消息从未到达）
//   * 全部受监控组件 kOk         -> READY
//   * 其余（stale / unhealthy）  -> DEGRADED
// 未监控组件永远不影响结论。
inline ReadinessVerdict EvaluateReadiness(const std::vector<ComponentProbe>& components) {
  ReadinessVerdict verdict;
  verdict.components = components;

  bool all_ok = true;
  bool any_seen_but_bad = false;
  bool any_monitored = false;
  std::vector<std::string> never_seen;
  std::vector<std::string> bad;

  for (const auto& probe : components) {
    if (probe.monitored) {
      any_monitored = true;
    }
    switch (EvaluateComponent(probe)) {
      case ComponentStatus::kOk:
        break;
      case ComponentStatus::kNotMonitored:
        break;
      case ComponentStatus::kNeverSeen:
        all_ok = false;
        never_seen.push_back(probe.name);
        break;
      case ComponentStatus::kStale:
      case ComponentStatus::kUnhealthy:
        all_ok = false;
        any_seen_but_bad = true;
        bad.push_back(probe.name);
        break;
    }
  }

  verdict.ready = all_ok && any_monitored;
  if (verdict.ready) {
    verdict.state = "READY";
    verdict.message = "all monitored components healthy";
    return verdict;
  }
  if (!any_monitored) {
    // 空集全过会让"什么都没监控"看起来就绪 —— 明确判负。
    verdict.state = "INITIALIZING";
    verdict.message = "no monitored components";
    return verdict;
  }
  // 只要有一个组件"见过但坏了"，就不是初始化中，而是明确降级；
  // 否则（有组件从未出声）按初始化中处理。
  verdict.state = any_seen_but_bad ? "DEGRADED" : "INITIALIZING";
  verdict.message = any_seen_but_bad ? "degraded: " : "waiting for: ";
  const auto& names = any_seen_but_bad ? bad : never_seen;
  for (std::size_t i = 0; i < names.size(); ++i) {
    if (i) verdict.message += ",";
    verdict.message += names[i];
  }
  return verdict;
}

// 组件名（作为 JSON 键 "component" 的取值）必须稳定且有限，否则指标标签基数会爆。
inline std::string RenderReadinessJson(const ReadinessVerdict& verdict) {
  std::string out = "{";
  out += "\"ready\":" + std::string(verdict.ready ? "true" : "false");
  out += "," + JsonStringPair("state", verdict.state);
  out += "," + JsonStringPair("message", verdict.message);
  out += ",\"components\":[";
  for (std::size_t i = 0; i < verdict.components.size(); ++i) {
    const auto& probe = verdict.components[i];
    const ComponentStatus status = EvaluateComponent(probe);
    if (i) out += ",";
    out += "{";
    out += JsonStringPair("component", probe.name) + ",";
    out += "\"ok\":" + std::string(status == ComponentStatus::kOk ? "true" : "false") + ",";
    out += "\"monitored\":" + std::string(probe.monitored ? "true" : "false") + ",";
    out += std::string("\"reason\":\"") + ComponentStatusName(status) + "\",";
    // age 用固定 3 位小数：探针输出要能直接 diff，避免 %g 的抖动
    char age[32];
    std::snprintf(age, sizeof(age), "%.3f", probe.age_seconds);
    out += std::string("\"age_seconds\":") + age + ",";
    char stale[32];
    std::snprintf(stale, sizeof(stale), "%.3f", probe.stale_after_seconds);
    out += std::string("\"stale_after_seconds\":") + stale + ",";
    out += JsonStringPair("detail", probe.detail);
    out += "}";
  }
  out += "]}";
  return out;
}

// liveness 响应体：恒定形态，只带进程自身信息（不含任何心跳/业务状态）。
inline std::string RenderLivenessJson(double uptime_sec) {
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.3f", uptime_sec);
  return std::string("{\"status\":\"ok\",\"uptime_seconds\":") + buf + "}";
}

}  // namespace amr_dispatcher_tools
