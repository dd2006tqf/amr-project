#pragma once

#include <string>
#include <utility>
#include <vector>

// P0.6 多车并发的「话题与 TF 隔离」命名契约（A4）。
//
// 兼容性红线（不可破）：vehicle_id 为空（默认）时，本文件解析出的每一个话题名
// 都必须与改动前**逐字一致**（/plan、/cmd_vel_raw、/cmd_vel_safe、/odom、
// /tracking/error、/safety/heartbeat/*、/safety/estop、/safety/bumper、
// /safety/manual_takeover、/chassis/link_health、/safety/state）。
//
// 派生规则（统一形状，suffix 拼接）：
//   有效名 = 显式覆盖参数 非空 ? 显式覆盖参数
//                              : (vehicle_id 非空 ? legacy_name + "/" + vehicle_id
//                                                 : legacy_name)
// 例：legacy "/plan" + vid "amr_01" -> "/plan/amr_01"
//     legacy "/safety/heartbeat/path_tracker" + vid "amr_01"
//                                  -> "/safety/heartbeat/path_tracker/amr_01"
namespace amr_dispatcher_ros {

// vehicle_id 必须符合 ROS 2 名称规范 [A-Za-z_][A-Za-z0-9_]*
// （外部来源的 id，如 VDA 5050 serialNumber，不保证合法）。
inline bool IsValidVehicleId(const std::string& vehicle_id) {
  if (vehicle_id.empty()) {
    return false;
  }
  const char first = vehicle_id.front();
  const bool first_ok = (first >= 'A' && first <= 'Z') || (first >= 'a' && first <= 'z') ||
                        first == '_';
  if (!first_ok) {
    return false;
  }
  for (const char c : vehicle_id) {
    const bool ok = (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
                    (c >= '0' && c <= '9') || c == '_';
    if (!ok) {
      return false;
    }
  }
  return true;
}

// 话题名解析：显式覆盖 > per-vehicle 派生 > legacy 全局名（单车兼容红线）。
inline std::string ResolveVehicleTopic(const std::string& override_topic,
                                       const std::string& legacy_topic,
                                       const std::string& vehicle_id) {
  if (!override_topic.empty()) {
    return override_topic;
  }
  if (vehicle_id.empty()) {
    return legacy_topic;
  }
  return legacy_topic + "/" + vehicle_id;
}

// 逗号分隔 vehicle id 列表解析：裁剪空白、保序去重。
inline std::vector<std::string> SplitVehicleIds(const std::string& csv) {
  std::vector<std::string> out;
  std::size_t start = 0;
  while (start <= csv.size()) {
    const std::size_t end = csv.find(',', start);
    const std::string raw = csv.substr(
        start, end == std::string::npos ? std::string::npos : end - start);
    std::string token;
    for (const char c : raw) {
      if (c == ' ' || c == '\t' || c == '\n' || c == '\r') {
        continue;
      }
      token.push_back(c);
    }
    if (!token.empty()) {
      bool seen = false;
      for (const auto& existing : out) {
        if (existing == token) {
          seen = true;
          break;
        }
      }
      if (!seen) {
        out.push_back(token);
      }
    }
    if (end == std::string::npos) {
      break;
    }
    start = end + 1;
  }
  return out;
}

}  // namespace amr_dispatcher_ros
