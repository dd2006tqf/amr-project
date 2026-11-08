#include "amr_dispatcher_core/dispatcher/dispatch_state_snapshot.hpp"

#include <yaml-cpp/yaml.h>

#include <fstream>
#include <string_view>

namespace amr_dispatcher_core::dispatcher {
namespace {

// 状态/结果用稳定字符串落盘：枚举数值一旦重排会静默读错，字符串不会。
std::string StateToText(MissionState state) { return std::string(ToString(state)); }

std::string ResultToText(MissionResult result) { return std::string(ToString(result)); }

YAML::Node MissionToNode(const Mission& m) {
  YAML::Node node(YAML::NodeType::Map);
  node["id"] = m.id;
  node["order_id"] = m.order_id;
  node["fleet_id"] = m.fleet_id;
  node["assigned_vehicle_id"] = m.assigned_vehicle_id;
  node["type"] = m.type;
  node["pickup_station"] = m.pickup_station;
  node["dropoff_station"] = m.dropoff_station;
  node["priority"] = m.priority;
  node["sequence"] = m.sequence;
  node["deadline_unix_ms"] = m.deadline_unix_ms;
  node["expected_seconds"] = m.expected_seconds;
  node["state"] = StateToText(m.state);
  node["result"] = ResultToText(m.result);
  node["attempt"] = m.attempt;
  node["reservation_id"] = m.reservation_id;
  return node;
}

// 宽松读取：缺字段用默认值；字段类型明显错误（如 id 是序列）则视为损坏。
bool ReadString(const YAML::Node& node, const char* key, std::string* out, std::string* err) {
  if (!node[key]) {
    out->clear();
    return true;
  }
  try {
    *out = node[key].as<std::string>();
    return true;
  } catch (const YAML::Exception& e) {
    if (err != nullptr) *err = std::string("field '") + key + "' is not a string: " + e.what();
    return false;
  }
}

bool ReadInt(const YAML::Node& node, const char* key, int* out, std::string* err) {
  if (!node[key]) {
    *out = 0;
    return true;
  }
  try {
    *out = node[key].as<int>();
    return true;
  } catch (const YAML::Exception& e) {
    if (err != nullptr) *err = std::string("field '") + key + "' is not an integer: " + e.what();
    return false;
  }
}

bool ReadInt64(const YAML::Node& node, const char* key, std::int64_t* out, std::string* err) {
  if (!node[key]) {
    *out = 0;
    return true;
  }
  try {
    *out = node[key].as<std::int64_t>();
    return true;
  } catch (const YAML::Exception& e) {
    if (err != nullptr) *err = std::string("field '") + key + "' is not an integer: " + e.what();
    return false;
  }
}

bool ReadUint64(const YAML::Node& node, const char* key, std::uint64_t* out, std::string* err) {
  if (!node[key]) {
    *out = 0;
    return true;
  }
  try {
    *out = node[key].as<std::uint64_t>();
    return true;
  } catch (const YAML::Exception& e) {
    if (err != nullptr) *err = std::string("field '") + key + "' is not an integer: " + e.what();
    return false;
  }
}

bool MissionFromNode(const YAML::Node& node, Mission* mission, std::string* err) {
  if (!node.IsMap()) {
    if (err != nullptr) *err = "mission entry is not a mapping";
    return false;
  }
  Mission m;
  if (!ReadString(node, "id", &m.id, err) || !ReadString(node, "order_id", &m.order_id, err) ||
      !ReadString(node, "fleet_id", &m.fleet_id, err) ||
      !ReadString(node, "assigned_vehicle_id", &m.assigned_vehicle_id, err) ||
      !ReadString(node, "type", &m.type, err) ||
      !ReadString(node, "pickup_station", &m.pickup_station, err) ||
      !ReadString(node, "dropoff_station", &m.dropoff_station, err) ||
      !ReadInt(node, "priority", &m.priority, err) ||
      !ReadUint64(node, "sequence", &m.sequence, err) ||
      !ReadInt64(node, "deadline_unix_ms", &m.deadline_unix_ms, err) ||
      !ReadInt(node, "expected_seconds", &m.expected_seconds, err) ||
      !ReadInt(node, "attempt", &m.attempt, err) ||
      !ReadString(node, "reservation_id", &m.reservation_id, err)) {
    return false;
  }
  if (m.id.empty()) {
    if (err != nullptr) *err = "mission entry without id";
    return false;
  }
  if (m.type.empty()) {
    m.type = "transport";
  }
  // 状态/结果按字符串解析；解析不了说明文件被改坏，拒绝而不是猜
  std::string state_text;
  std::string result_text;
  if (!ReadString(node, "state", &state_text, err) ||
      !ReadString(node, "result", &result_text, err)) {
    return false;
  }
  if (!state_text.empty()) {
    const auto parsed = ParseMissionState(state_text);
    if (!parsed.has_value()) {
      if (err != nullptr) *err = "unknown mission state '" + state_text + "' for " + m.id;
      return false;
    }
    m.state = *parsed;
  }
  if (!result_text.empty()) {
    const auto parsed = ParseMissionResult(result_text);
    if (!parsed.has_value()) {
      if (err != nullptr) *err = "unknown mission result '" + result_text + "' for " + m.id;
      return false;
    }
    m.result = *parsed;
  }
  *mission = std::move(m);
  return true;
}

bool ReadStringList(const YAML::Node& parent, const char* key, std::vector<std::string>* out,
                    std::string* err) {
  out->clear();
  const YAML::Node node = parent[key];
  if (!node) {
    return true;
  }
  if (!node.IsSequence()) {
    if (err != nullptr) *err = std::string("field '") + key + "' is not a sequence";
    return false;
  }
  for (const auto& item : node) {
    try {
      out->push_back(item.as<std::string>());
    } catch (const YAML::Exception& e) {
      if (err != nullptr) {
        *err = std::string("field '") + key + "' contains a non-string: " + e.what();
      }
      return false;
    }
  }
  return true;
}

bool ReadMissionList(const YAML::Node& parent, const char* key, std::vector<Mission>* out,
                     std::string* err) {
  out->clear();
  const YAML::Node node = parent[key];
  if (!node) {
    return true;
  }
  if (!node.IsSequence()) {
    if (err != nullptr) *err = std::string("field '") + key + "' is not a sequence";
    return false;
  }
  for (const auto& item : node) {
    Mission m;
    if (!MissionFromNode(item, &m, err)) {
      return false;
    }
    out->push_back(std::move(m));
  }
  return true;
}

}  // namespace

std::string SerializeDispatchState(const DispatchStateSnapshot& snapshot) {
  YAML::Node root(YAML::NodeType::Map);
  root["format_version"] = snapshot.format_version;
  root["map_hash"] = snapshot.map_hash;
  root["next_sequence"] = snapshot.next_sequence;

  YAML::Node queued(YAML::NodeType::Sequence);
  for (const auto& m : snapshot.queued_missions) {
    queued.push_back(MissionToNode(m));
  }
  root["queued_missions"] = queued;

  YAML::Node paused(YAML::NodeType::Sequence);
  for (const auto& id : snapshot.paused_order_ids) {
    paused.push_back(id);
  }
  root["paused_order_ids"] = paused;

  YAML::Node active(YAML::NodeType::Sequence);
  for (const auto& m : snapshot.active_missions) {
    active.push_back(MissionToNode(m));
  }
  root["active_missions"] = active;

  YAML::Node quarantined(YAML::NodeType::Sequence);
  for (const auto& id : snapshot.quarantined_vehicle_ids) {
    quarantined.push_back(id);
  }
  root["quarantined_vehicle_ids"] = quarantined;

  YAML::Emitter out;
  out << root;
  return std::string(out.c_str()) + "\n";
}

std::optional<DispatchStateSnapshot> DeserializeDispatchState(const std::string& text,
                                                             std::string* message) {
  if (message != nullptr) message->clear();
  if (text.empty()) {
    if (message != nullptr) *message = "empty snapshot text";
    return std::nullopt;
  }
  YAML::Node root;
  try {
    root = YAML::Load(text);
  } catch (const YAML::Exception& e) {
    if (message != nullptr) *message = std::string("yaml parse error: ") + e.what();
    return std::nullopt;
  }
  if (!root.IsMap()) {
    if (message != nullptr) *message = "snapshot root is not a mapping";
    return std::nullopt;
  }

  std::string err;
  DispatchStateSnapshot snapshot;
  std::uint64_t version_value = 0;
  if (!ReadUint64(root, "format_version", &version_value, &err)) {
    if (message != nullptr) *message = err;
    return std::nullopt;
  }
  if (version_value != kDispatchStateFormatVersion) {
    if (message != nullptr) {
      *message = "unsupported format_version " + std::to_string(version_value);
    }
    return std::nullopt;
  }
  snapshot.format_version = static_cast<std::uint32_t>(version_value);
  if (!ReadString(root, "map_hash", &snapshot.map_hash, &err) ||
      !ReadUint64(root, "next_sequence", &snapshot.next_sequence, &err) ||
      !ReadMissionList(root, "queued_missions", &snapshot.queued_missions, &err) ||
      !ReadStringList(root, "paused_order_ids", &snapshot.paused_order_ids, &err) ||
      !ReadMissionList(root, "active_missions", &snapshot.active_missions, &err) ||
      !ReadStringList(root, "quarantined_vehicle_ids", &snapshot.quarantined_vehicle_ids, &err)) {
    if (message != nullptr) *message = err;
    return std::nullopt;
  }

  // 序号游标必须能覆盖所有已见 sequence，否则恢复后新任务会与旧任务撞号
  std::uint64_t max_sequence = 0;
  for (const auto& m : snapshot.queued_missions) {
    if (m.sequence > max_sequence) max_sequence = m.sequence;
  }
  for (const auto& m : snapshot.active_missions) {
    if (m.sequence > max_sequence) max_sequence = m.sequence;
  }
  if (snapshot.next_sequence <= max_sequence) {
    snapshot.next_sequence = max_sequence + 1;
  }
  return snapshot;
}

bool WriteDispatchStateAtomic(const std::filesystem::path& path,
                              const DispatchStateSnapshot& snapshot, std::string* message) {
  if (message != nullptr) message->clear();
  std::error_code ec;
  if (path.has_parent_path()) {
    std::filesystem::create_directories(path.parent_path(), ec);
    if (ec) {
      if (message != nullptr) *message = "cannot create directory: " + ec.message();
      return false;
    }
  }
  const std::filesystem::path tmp_path = path.string() + ".tmp";
  const std::string text = SerializeDispatchState(snapshot);
  {
    std::ofstream out(tmp_path, std::ios::trunc);
    if (!out) {
      if (message != nullptr) *message = "cannot open temp file " + tmp_path.string();
      return false;
    }
    out << text;
    out.flush();
    if (!out) {
      out.close();
      std::filesystem::remove(tmp_path, ec);
      if (message != nullptr) *message = "failed writing temp file " + tmp_path.string();
      return false;
    }
  }
  // 回读校验：磁盘上的字节必须能被同一套解析器读回来（防半截/编码问题）。
  std::ifstream in(tmp_path);
  if (!in) {
    std::filesystem::remove(tmp_path, ec);
    if (message != nullptr) *message = "cannot reread temp file " + tmp_path.string();
    return false;
  }
  const std::string round_trip((std::istreambuf_iterator<char>(in)),
                               std::istreambuf_iterator<char>());
  std::string verify_message;
  if (!DeserializeDispatchState(round_trip, &verify_message).has_value()) {
    std::filesystem::remove(tmp_path, ec);
    if (message != nullptr) {
      *message = "round-trip validation failed: " + verify_message;
    }
    return false;
  }
  std::filesystem::rename(tmp_path, path, ec);
  if (ec) {
    std::filesystem::remove(tmp_path, ec);
    if (message != nullptr) {
      *message = "atomic rename failed: " + ec.message();
    }
    return false;
  }
  return true;
}

std::optional<DispatchStateSnapshot> LoadDispatchState(const std::filesystem::path& path,
                                                       std::string* message) {
  if (message != nullptr) message->clear();
  std::error_code ec;
  if (!std::filesystem::exists(path, ec)) {
    if (message != nullptr) *message = "not_found";
    return std::nullopt;
  }
  std::ifstream in(path);
  if (!in) {
    if (message != nullptr) *message = "cannot open " + path.string();
    return std::nullopt;
  }
  const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  return DeserializeDispatchState(text, message);
}

}  // namespace amr_dispatcher_core::dispatcher
