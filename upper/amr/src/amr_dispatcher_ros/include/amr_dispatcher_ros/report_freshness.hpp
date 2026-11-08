#pragma once

#include <chrono>
#include <string>
#include <unordered_map>

namespace amr_dispatcher_ros {

// I1：多源健康报告的新鲜度判定（纯函数，可脱离 rclcpp 单测）。
// 时效打点必须发生在**真实数据到达点**（各订阅回调），而不是巡检调用周期——
// 否则上游发布者死亡后，调用周期会无限"续命"，失联检测完全失效。
// 只对周期性数据源做时效判定；estop 属一次性锁存信号（latched QoS 语义），
// 不参与——对锁存源打点会把"不重复发送"误判为过期。
struct ReportFreshness {
  bool chassis = false;  // /chassis/link_health 本轮是否有新数据
  bool watchdog = false; // 心跳（chassis_driver / path_tracker）本轮是否有新数据
};

// last_arrival：各数据源最后一次真实到达时刻；tolerance：容忍窗口
//（应覆盖发布周期的数倍，避免抖动误判为陈旧）。
// 从未到达的源保持 false —— 由 FaultSupervisor 的"从未上报不参与时效"语义兜底。
inline ReportFreshness MakeReportFreshness(
    const std::unordered_map<std::string, std::chrono::steady_clock::time_point>& last_arrival,
    std::chrono::steady_clock::time_point now,
    std::chrono::steady_clock::duration tolerance) {
  ReportFreshness fresh;
  const auto is_fresh = [&](const char* key) {
    const auto it = last_arrival.find(key);
    if (it == last_arrival.end()) return false;
    return (now - it->second) <= tolerance;
  };
  fresh.chassis = is_fresh("chassis");
  fresh.watchdog = is_fresh("watchdog");
  return fresh;
}

}  // namespace amr_dispatcher_ros
