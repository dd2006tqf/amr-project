#include "amr_dispatcher_core/fleet/vda5050_serializer.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

namespace amr_dispatcher_core::fleet {

namespace {

// 获取当前 UTC 时间 ISO 8601 字符串，例如 2026-10-20T12:00:00.000Z
std::string CurrentIsoTimestamp() {
  const auto now = std::chrono::system_clock::now();
  const auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(
                          now.time_since_epoch()) %
                      1000;
  const std::time_t timer = std::chrono::system_clock::to_time_t(now);
  std::tm tm_buf{};
#if defined(_WIN32)
  gmtime_s(&tm_buf, &timer);
#else
  gmtime_r(&timer, &tm_buf);
#endif
  std::ostringstream ss;
  ss << std::put_time(&tm_buf, "%Y-%m-%dT%H:%M:%S") << '.'
     << std::setfill('0') << std::setw(3) << millis.count() << 'Z';
  return ss.str();
}

std::string EscapeJsonString(const std::string& input) {
  std::string out;
  out.reserve(input.size() + 8);
  for (const char c : input) {
    switch (c) {
      case '"':
        out += "\\\"";
        break;
      case '\\':
        out += "\\\\";
        break;
      case '\b':
        out += "\\b";
        break;
      case '\f':
        out += "\\f";
        break;
      case '\n':
        out += "\\n";
        break;
      case '\r':
        out += "\\r";
        break;
      case '\t':
        out += "\\t";
        break;
      default:
        out += c;
        break;
    }
  }
  return out;
}

std::string FormatDouble(double v) {
  if (std::isnan(v) || std::isinf(v)) {
    return "0.0";
  }
  std::ostringstream ss;
  ss << std::setprecision(8) << v;
  std::string s = ss.str();
  return s;
}

// -------------------------------------------------------------
// 极轻量但健壮的递归下降 JSON 抽象节点树
// -------------------------------------------------------------
enum class JsonType { Null, Bool, Number, String, Array, Object };

struct JsonValue {
  JsonType type = JsonType::Null;
  bool bool_val = false;
  double num_val = 0.0;
  std::string str_val;
  std::vector<JsonValue> arr_val;
  std::unordered_map<std::string, JsonValue> obj_val;

  bool IsObject() const { return type == JsonType::Object; }
  bool IsArray() const { return type == JsonType::Array; }
  bool IsString() const { return type == JsonType::String; }
  bool IsNumber() const { return type == JsonType::Number; }
  bool IsBool() const { return type == JsonType::Bool; }

  const JsonValue* Find(const std::string& key) const {
    if (type != JsonType::Object) return nullptr;
    auto it = obj_val.find(key);
    if (it != obj_val.end()) {
      return &it->second;
    }
    return nullptr;
  }
};

class MiniJsonParser {
 public:
  explicit MiniJsonParser(std::string_view text) : text_(text), pos_(0) {}

  std::optional<JsonValue> Parse() {
    SkipWhitespace();
    if (pos_ >= text_.size()) {
      return std::nullopt;
    }
    auto val = ParseValue();
    if (!val.has_value()) {
      return std::nullopt;
    }
    SkipWhitespace();
    if (pos_ != text_.size()) {
      return std::nullopt;  // 尾部有多余非空白字符
    }
    return val;
  }

 private:
  char Peek() const {
    if (pos_ < text_.size()) return text_[pos_];
    return '\0';
  }

  char Next() {
    if (pos_ < text_.size()) return text_[pos_++];
    return '\0';
  }

  void SkipWhitespace() {
    while (pos_ < text_.size()) {
      const char c = text_[pos_];
      if (c == ' ' || c == '\t' || c == '\r' || c == '\n') {
        ++pos_;
      } else {
        break;
      }
    }
  }

  std::optional<JsonValue> ParseValue() {
    SkipWhitespace();
    if (pos_ >= text_.size()) return std::nullopt;
    char c = Peek();
    if (c == '{') return ParseObject();
    if (c == '[') return ParseArray();
    if (c == '"') return ParseString();
    if (c == 't' || c == 'f') return ParseBool();
    if (c == 'n') return ParseNull();
    if (c == '-' || (c >= '0' && c <= '9')) return ParseNumber();
    return std::nullopt;
  }

  std::optional<JsonValue> ParseObject() {
    if (Next() != '{') return std::nullopt;
    JsonValue val;
    val.type = JsonType::Object;
    SkipWhitespace();
    if (Peek() == '}') {
      Next();
      return val;
    }
    while (pos_ < text_.size()) {
      SkipWhitespace();
      if (Peek() != '"') return std::nullopt;
      auto key_val = ParseString();
      if (!key_val.has_value()) return std::nullopt;
      std::string key = key_val->str_val;

      SkipWhitespace();
      if (Next() != ':') return std::nullopt;

      auto member_val = ParseValue();
      if (!member_val.has_value()) return std::nullopt;

      val.obj_val.emplace(std::move(key), std::move(*member_val));

      SkipWhitespace();
      char sep = Peek();
      if (sep == ',') {
        Next();
      } else if (sep == '}') {
        Next();
        return val;
      } else {
        return std::nullopt;
      }
    }
    return std::nullopt;
  }

  std::optional<JsonValue> ParseArray() {
    if (Next() != '[') return std::nullopt;
    JsonValue val;
    val.type = JsonType::Array;
    SkipWhitespace();
    if (Peek() == ']') {
      Next();
      return val;
    }
    while (pos_ < text_.size()) {
      auto item = ParseValue();
      if (!item.has_value()) return std::nullopt;
      val.arr_val.push_back(std::move(*item));

      SkipWhitespace();
      char sep = Peek();
      if (sep == ',') {
        Next();
      } else if (sep == ']') {
        Next();
        return val;
      } else {
        return std::nullopt;
      }
    }
    return std::nullopt;
  }

  std::optional<JsonValue> ParseString() {
    if (Next() != '"') return std::nullopt;
    std::string s;
    while (pos_ < text_.size()) {
      char c = Next();
      if (c == '"') {
        JsonValue v;
        v.type = JsonType::String;
        v.str_val = std::move(s);
        return v;
      }
      if (c == '\\') {
        if (pos_ >= text_.size()) return std::nullopt;
        char esc = Next();
        switch (esc) {
          case '"':
            s += '"';
            break;
          case '\\':
            s += '\\';
            break;
          case '/':
            s += '/';
            break;
          case 'b':
            s += '\b';
            break;
          case 'f':
            s += '\f';
            break;
          case 'n':
            s += '\n';
            break;
          case 'r':
            s += '\r';
            break;
          case 't':
            s += '\t';
            break;
          case 'u': {
            // 跳过 4 位 hex，简化处理
            if (pos_ + 4 > text_.size()) return std::nullopt;
            pos_ += 4;
            s += '?';
            break;
          }
          default:
            s += esc;
            break;
        }
      } else {
        s += c;
      }
    }
    return std::nullopt;
  }

  std::optional<JsonValue> ParseNumber() {
    size_t start = pos_;
    if (Peek() == '-') Next();
    while (pos_ < text_.size() && (std::isdigit(static_cast<unsigned char>(text_[pos_])) ||
                                   text_[pos_] == '.' || text_[pos_] == 'e' ||
                                   text_[pos_] == 'E' || text_[pos_] == '+' ||
                                   text_[pos_] == '-')) {
      Next();
    }
    std::string sub(text_.substr(start, pos_ - start));
    try {
      size_t processed = 0;
      double d = std::stod(sub, &processed);
      if (processed != sub.size() || std::isnan(d) || std::isinf(d)) {
        return std::nullopt;
      }
      JsonValue v;
      v.type = JsonType::Number;
      v.num_val = d;
      return v;
    } catch (...) {
      return std::nullopt;
    }
  }

  std::optional<JsonValue> ParseBool() {
    if (text_.substr(pos_, 4) == "true") {
      pos_ += 4;
      JsonValue v;
      v.type = JsonType::Bool;
      v.bool_val = true;
      return v;
    }
    if (text_.substr(pos_, 5) == "false") {
      pos_ += 5;
      JsonValue v;
      v.type = JsonType::Bool;
      v.bool_val = false;
      return v;
    }
    return std::nullopt;
  }

  std::optional<JsonValue> ParseNull() {
    if (text_.substr(pos_, 4) == "null") {
      pos_ += 4;
      JsonValue v;
      v.type = JsonType::Null;
      return v;
    }
    return std::nullopt;
  }

  std::string_view text_;
  size_t pos_;
};

}  // namespace

std::string SerializeVda5050Order(const VehicleOrder& order, const std::string& version) {
  std::ostringstream ss;
  const std::string vda_version = (version == "v3") ? "3.0.0" : "2.0.0";
  const std::string timestamp = CurrentIsoTimestamp();

  ss << "{";
  ss << "\"headerId\":0,";
  ss << "\"timestamp\":\"" << timestamp << "\",";
  ss << "\"version\":\"" << vda_version << "\",";
  ss << "\"manufacturer\":\"RobotManufacturer\",";
  ss << "\"serialNumber\":\"" << EscapeJsonString(order.vehicle_id) << "\",";
  ss << "\"orderId\":\"" << EscapeJsonString(order.order_id) << "\",";
  ss << "\"orderUpdateId\":" << order.order_update_id << ",";

  // nodes
  ss << "\"nodes\":[";
  for (size_t i = 0; i < order.nodes.size(); ++i) {
    if (i > 0) ss << ",";
    const auto& node = order.nodes[i];
    ss << "{";
    ss << "\"nodeId\":\"" << EscapeJsonString(node.node_id) << "\",";
    ss << "\"sequenceId\":" << node.sequence_id << ",";
    ss << "\"released\":" << (node.released ? "true" : "false") << ",";
    ss << "\"nodePosition\":{";
    ss << "\"x\":" << FormatDouble(node.x) << ",";
    ss << "\"y\":" << FormatDouble(node.y) << ",";
    ss << "\"theta\":" << FormatDouble(node.yaw) << ",";
    ss << "\"mapId\":\"map\"";
    ss << "},";
    ss << "\"actions\":[";
    for (size_t a = 0; a < node.actions.size(); ++a) {
      if (a > 0) ss << ",";
      const auto& act = node.actions[a];
      ss << "{";
      ss << "\"actionType\":\"" << EscapeJsonString(act.action_type) << "\",";
      ss << "\"actionId\":\"" << EscapeJsonString(act.action_id) << "\",";
      ss << "\"blockingType\":\"HARD\",";
      ss << "\"actionParameters\":[";
      size_t p_idx = 0;
      for (const auto& [k, v] : act.parameters) {
        if (p_idx > 0) ss << ",";
        ss << "{\"key\":\"" << EscapeJsonString(k) << "\",\"value\":\""
           << EscapeJsonString(v) << "\"}";
        ++p_idx;
      }
      ss << "]";
      ss << "}";
    }
    ss << "]";
    ss << "}";
  }
  ss << "],";

  // edges
  ss << "\"edges\":[";
  for (size_t i = 0; i < order.edges.size(); ++i) {
    if (i > 0) ss << ",";
    const auto& edge = order.edges[i];
    ss << "{";
    ss << "\"edgeId\":\"" << EscapeJsonString(edge.edge_id) << "\",";
    ss << "\"sequenceId\":" << (i * 2 + 2) << ",";  // edge sequence id
    ss << "\"startNodeId\":\"" << EscapeJsonString(edge.start_node_id) << "\",";
    ss << "\"endNodeId\":\"" << EscapeJsonString(edge.end_node_id) << "\",";
    ss << "\"released\":" << (edge.released ? "true" : "false") << ",";
    ss << "\"maxSpeed\":" << FormatDouble(edge.max_speed) << ",";

    // trajectory
    ss << "\"trajectory\":{";
    ss << "\"degree\":1,";
    ss << "\"controlPoints\":[";
    for (size_t p = 0; p < edge.polyline.size(); ++p) {
      if (p > 0) ss << ",";
      ss << "{";
      ss << "\"x\":" << FormatDouble(edge.polyline[p].x) << ",";
      ss << "\"y\":" << FormatDouble(edge.polyline[p].y) << ",";
      ss << "\"theta\":" << FormatDouble(edge.polyline[p].yaw);
      ss << "}";
    }
    ss << "]";
    ss << "},";

    // actions
    ss << "\"actions\":[";
    for (size_t a = 0; a < edge.actions.size(); ++a) {
      if (a > 0) ss << ",";
      const auto& act = edge.actions[a];
      ss << "{";
      ss << "\"actionType\":\"" << EscapeJsonString(act.action_type) << "\",";
      ss << "\"actionId\":\"" << EscapeJsonString(act.action_id) << "\",";
      ss << "\"blockingType\":\"HARD\",";
      ss << "\"actionParameters\":[";
      size_t p_idx = 0;
      for (const auto& [k, v] : act.parameters) {
        if (p_idx > 0) ss << ",";
        ss << "{\"key\":\"" << EscapeJsonString(k) << "\",\"value\":\""
           << EscapeJsonString(v) << "\"}";
        ++p_idx;
      }
      ss << "]";
      ss << "}";
    }
    ss << "]";
    ss << "}";
  }
  ss << "]";

  ss << "}";
  return ss.str();
}

std::string SerializeVda5050InstantActions(const std::string& vehicle_id,
                                          const std::vector<VehicleAction>& actions,
                                          const std::string& version) {
  std::ostringstream ss;
  const std::string vda_version = (version == "v3") ? "3.0.0" : "2.0.0";
  const std::string timestamp = CurrentIsoTimestamp();

  ss << "{";
  ss << "\"headerId\":0,";
  ss << "\"timestamp\":\"" << timestamp << "\",";
  ss << "\"version\":\"" << vda_version << "\",";
  ss << "\"manufacturer\":\"RobotManufacturer\",";
  ss << "\"serialNumber\":\"" << EscapeJsonString(vehicle_id) << "\",";
  ss << "\"actions\":[";
  for (size_t i = 0; i < actions.size(); ++i) {
    if (i > 0) ss << ",";
    const auto& act = actions[i];
    ss << "{";
    ss << "\"actionType\":\"" << EscapeJsonString(act.action_type) << "\",";
    ss << "\"actionId\":\"" << EscapeJsonString(act.action_id) << "\",";
    ss << "\"blockingType\":\"HARD\",";
    ss << "\"actionParameters\":[";
    size_t p_idx = 0;
    for (const auto& [k, v] : act.parameters) {
      if (p_idx > 0) ss << ",";
      ss << "{\"key\":\"" << EscapeJsonString(k) << "\",\"value\":\""
         << EscapeJsonString(v) << "\"}";
      ++p_idx;
    }
    ss << "]";
    ss << "}";
  }
  ss << "]";
  ss << "}";
  return ss.str();
}

std::optional<VehicleStateReport> DeserializeVda5050State(const std::string& json_str) {
  if (json_str.empty()) {
    return std::nullopt;
  }
  MiniJsonParser parser(json_str);
  auto root = parser.Parse();
  if (!root.has_value() || !root->IsObject()) {
    return std::nullopt;
  }

  // 必须字段检查: serialNumber
  const auto* serial_val = root->Find("serialNumber");
  if (!serial_val || !serial_val->IsString() || serial_val->str_val.empty()) {
    return std::nullopt;
  }

  // agvPosition 必须存在且合法
  const auto* pos_val = root->Find("agvPosition");
  if (!pos_val || !pos_val->IsObject()) {
    return std::nullopt;
  }
  const auto* x_val = pos_val->Find("x");
  const auto* y_val = pos_val->Find("y");
  const auto* theta_val = pos_val->Find("theta");
  if (!x_val || !x_val->IsNumber() || !y_val || !y_val->IsNumber() ||
      !theta_val || !theta_val->IsNumber()) {
    return std::nullopt;
  }

  VehicleStateReport report;
  report.vehicle_id = serial_val->str_val;
  report.x = x_val->num_val;
  report.y = y_val->num_val;
  report.yaw = theta_val->num_val;

  // orderId
  const auto* order_id_val = root->Find("orderId");
  if (order_id_val && order_id_val->IsString()) {
    report.order_id = order_id_val->str_val;
  }

  // lastNodeId
  const auto* last_node_val = root->Find("lastNodeId");
  if (last_node_val && last_node_val->IsString()) {
    report.last_node_id = last_node_val->str_val;
  }

  // lastNodeSequenceId
  const auto* last_seq_val = root->Find("lastNodeSequenceId");
  if (last_seq_val && last_seq_val->IsNumber()) {
    report.last_node_sequence_id = static_cast<uint32_t>(last_seq_val->num_val);
  }

  // batteryState
  const auto* battery_val = root->Find("batteryState");
  if (battery_val && battery_val->IsObject()) {
    const auto* charge_val = battery_val->Find("batteryCharge");
    if (charge_val && charge_val->IsNumber()) {
      report.battery_percent = charge_val->num_val;
    }
    const auto* charging_val = battery_val->Find("charging");
    if (charging_val && charging_val->IsBool()) {
      report.is_charging = charging_val->bool_val;
    }
  }

  // safetyState
  const auto* safety_val = root->Find("safetyState");
  if (safety_val && safety_val->IsObject()) {
    const auto* estop_val = safety_val->Find("eStop");
    if (estop_val) {
      if (estop_val->IsBool()) {
        report.estop_active = estop_val->bool_val;
      } else if (estop_val->IsString()) {
        // VDA 5050 常见: "none", "autoAck", "manual", "remote"
        report.estop_active = (estop_val->str_val != "none" && !estop_val->str_val.empty());
      }
    }
  }

  // operatingMode
  const auto* mode_val = root->Find("operatingMode");
  if (mode_val && mode_val->IsString() && !mode_val->str_val.empty()) {
    report.operating_mode = mode_val->str_val;
  }

  // errors
  const auto* errors_val = root->Find("errors");
  if (errors_val && errors_val->IsArray()) {
    for (const auto& err_item : errors_val->arr_val) {
      if (err_item.IsObject()) {
        std::string err_type;
        std::string err_desc;
        const auto* t = err_item.Find("errorType");
        if (t && t->IsString()) err_type = t->str_val;
        const auto* d = err_item.Find("errorDescription");
        if (d && d->IsString()) err_desc = d->str_val;

        if (!err_type.empty() && !err_desc.empty()) {
          report.active_errors.push_back(err_type + ": " + err_desc);
        } else if (!err_type.empty()) {
          report.active_errors.push_back(err_type);
        } else if (!err_desc.empty()) {
          report.active_errors.push_back(err_desc);
        }
      }
    }
  }

  report.online = true;
  report.last_report_time = std::chrono::steady_clock::now();
  return report;
}

std::optional<std::string> DeserializeVda5050Connection(const std::string& json_str) {
  if (json_str.empty()) {
    return std::nullopt;
  }
  MiniJsonParser parser(json_str);
  auto root = parser.Parse();
  if (!root.has_value() || !root->IsObject()) {
    return std::nullopt;
  }

  const auto* conn_val = root->Find("connectionState");
  if (!conn_val || !conn_val->IsString() || conn_val->str_val.empty()) {
    return std::nullopt;
  }

  return conn_val->str_val;
}

}  // namespace amr_dispatcher_core::fleet
