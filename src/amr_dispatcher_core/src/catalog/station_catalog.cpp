#include "amr_dispatcher_core/catalog/station_catalog.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

namespace amr_dispatcher_core::catalog {
namespace {

std::string Trim(const std::string& value) {
  auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return "";
  }
  auto last = value.find_last_not_of(" \t\r\n");
  std::string s = value.substr(first, last - first + 1);
  // 去除 YAML 引号包裹：例如 "station_pickup_A" 或 'station_pickup_A'
  if (s.size() >= 2 && ((s.front() == '"' && s.back() == '"') || (s.front() == '\'' && s.back() == '\''))) {
    s = s.substr(1, s.size() - 2);
  }
  return s;
}

std::optional<std::string> ValueAfterColon(const std::string& line, const std::string& key) {
  const std::string prefix = key + ":";
  if (line.rfind(prefix, 0) != 0) {
    return std::nullopt;
  }
  return Trim(line.substr(prefix.size()));
}

std::optional<double> ParseDouble(const std::string& value) {
  std::istringstream parser(value);
  double result = 0.0;
  parser >> result;
  if (parser.fail() || !std::isfinite(result)) {
    return std::nullopt;
  }
  return result;
}

bool ParseBool(const std::string& value) {
  return value == "true" || value == "True" || value == "1";
}

// 解析 "- [1.5, 2.0]" 形式的 flow 折线点（取前两个分量）
std::optional<std::pair<double, double>> ParseFlowPoint(const std::string& content) {
  if (content.size() < 2 || content.front() != '[' || content.back() != ']') {
    return std::nullopt;
  }
  const std::string inner = content.substr(1, content.size() - 2);
  const auto comma = inner.find(',');
  if (comma == std::string::npos) {
    return std::nullopt;
  }
  const auto x = ParseDouble(Trim(inner.substr(0, comma)));
  const auto y = ParseDouble(Trim(inner.substr(comma + 1)));
  if (!x || !y) {
    return std::nullopt;
  }
  return std::pair<double, double>{*x, *y};
}

// edge 自身字段键（用于识别折线块何时结束）
bool IsEdgeFieldKey(const std::string& line) {
  return ValueAfterColon(line, "from").has_value() ||
         ValueAfterColon(line, "to").has_value() ||
         ValueAfterColon(line, "distance_m").has_value() ||
         ValueAfterColon(line, "distance").has_value() ||
         ValueAfterColon(line, "bidirectional").has_value() ||
         ValueAfterColon(line, "enabled").has_value();
}

}  // namespace

std::optional<StationCatalog> LoadStationCatalog(const std::filesystem::path& path) {
  std::ifstream input(path);
  if (!input) {
    return std::nullopt;
  }

  StationCatalog catalog;
  Station current_station;
  RoadNode current_road_node;
  CatalogStationEdge current_edge;
  std::string default_frame_id = "map";
  std::string section;
  bool in_station = false;
  bool in_road_node = false;
  bool in_edge = false;
  bool in_polyline = false;
  bool has_id = false;
  bool has_x = false;
  bool has_y = false;
  bool has_node_id = false;
  bool has_node_x = false;
  bool has_node_y = false;
  bool has_from = false;
  bool has_to = false;
  bool has_distance = false;
  // map 风格折线点（- x: ... / y: ...）的暂存分量
  std::pair<double, double> pending_point{0.0, 0.0};
  bool point_has_x = false;
  bool point_has_y = false;

  auto flush_polyline_point = [&]() {
    if (point_has_x && point_has_y) {
      current_edge.polyline.push_back(pending_point);
    }
    pending_point = std::pair<double, double>{0.0, 0.0};
    point_has_x = false;
    point_has_y = false;
  };

  auto flush_station = [&]() {
    if (in_station && has_id && has_x && has_y) {
      if (current_station.frame_id.empty()) {
        current_station.frame_id = default_frame_id;
      }
      catalog.stations.push_back(current_station);
    }
    current_station = Station{};
    current_station.frame_id = default_frame_id;
    in_station = false;
    has_id = false;
    has_x = false;
    has_y = false;
  };

  auto flush_road_node = [&]() {
    if (in_road_node && has_node_id && has_node_x && has_node_y) {
      if (current_road_node.frame_id.empty()) {
        current_road_node.frame_id = default_frame_id;
      }
      catalog.road_nodes.push_back(current_road_node);
    }
    current_road_node = RoadNode{};
    current_road_node.frame_id = default_frame_id;
    in_road_node = false;
    has_node_id = false;
    has_node_x = false;
    has_node_y = false;
  };

  auto flush_edge = [&]() {
    flush_polyline_point();
    if (in_edge && has_from && has_to && has_distance) {
      catalog.edges.push_back(current_edge);
    }
    current_edge = CatalogStationEdge{};
    in_edge = false;
    in_polyline = false;
    has_from = false;
    has_to = false;
    has_distance = false;
  };

  std::string line;
  while (std::getline(input, line)) {
    line = Trim(line);
    if (line.empty() || line.front() == '#') {
      continue;
    }

    if (line == "stations:" || line == "edges:" || line == "routes:" || line == "road_nodes:" ||
        line == "version:") {
      flush_station();
      flush_road_node();
      flush_edge();
      section = line.substr(0, line.size() - 1);
      continue;
    }

    if (!in_station && !in_edge && !in_road_node) {
      if (auto value = ValueAfterColon(line, "frame_id")) {
        default_frame_id = value->empty() ? "map" : *value;
        current_station.frame_id = default_frame_id;
        current_road_node.frame_id = default_frame_id;
        continue;
      }
      if (auto value = ValueAfterColon(line, "map_hash")) {
        catalog.map_hash = *value;
        continue;
      }
      if (auto value = ValueAfterColon(line, "graph_version")) {
        catalog.graph_version = *value;
        continue;
      }
    }

    if (line.rfind("- ", 0) == 0) {
      const std::string content = Trim(line.substr(2));
      if (in_polyline && !content.empty() && content.front() == '[') {
        // flow 风格折线点：- [x, y]
        if (auto point = ParseFlowPoint(content)) {
          flush_polyline_point();
          current_edge.polyline.push_back(*point);
        }
        continue;
      }
      if (in_polyline && (ValueAfterColon(content, "x") || ValueAfterColon(content, "y"))) {
        // map 风格折线点起始：- x: ... / - y: ...
        flush_polyline_point();
        if (auto value = ValueAfterColon(content, "x")) {
          if (auto parsed = ParseDouble(*value)) {
            pending_point.first = *parsed;
            point_has_x = true;
          }
        } else if (auto value = ValueAfterColon(content, "y")) {
          if (auto parsed = ParseDouble(*value)) {
            pending_point.second = *parsed;
            point_has_y = true;
          }
        }
        continue;
      }
      if (in_polyline) {
        // 折线块内的其他 "- " 条目：结束折线并开始下一条边
        flush_edge();
        in_edge = true;
      } else if (section == "edges" || section == "routes") {
        flush_edge();
        in_edge = true;
      } else if (section == "road_nodes") {
        flush_road_node();
        in_road_node = true;
        current_road_node.frame_id = default_frame_id;
      } else {
        flush_station();
        in_station = true;
        current_station.frame_id = default_frame_id;
      }
      line = content;
    }

    if (in_edge) {
      if (ValueAfterColon(line, "polyline")) {
        flush_polyline_point();
        in_polyline = true;
        continue;
      }
      if (in_polyline) {
        if (IsEdgeFieldKey(line)) {
          flush_polyline_point();
          in_polyline = false;  // 折线块结束，继续按 edge 字段解析本行
        } else if (auto value = ValueAfterColon(line, "x")) {
          flush_polyline_point();
          if (auto parsed = ParseDouble(*value)) {
            pending_point.first = *parsed;
            point_has_x = true;
          }
          continue;
        } else if (auto value = ValueAfterColon(line, "y")) {
          if (auto parsed = ParseDouble(*value)) {
            pending_point.second = *parsed;
            point_has_y = true;
          }
          continue;
        } else {
          continue;  // 折线块内的其余行忽略
        }
      }
      if (auto value = ValueAfterColon(line, "from")) {
        current_edge.from = *value;
        has_from = !current_edge.from.empty();
        continue;
      }
      if (auto value = ValueAfterColon(line, "to")) {
        current_edge.to = *value;
        has_to = !current_edge.to.empty();
        continue;
      }
      if (auto value = ValueAfterColon(line, "distance_m")) {
        if (auto parsed = ParseDouble(*value)) {
          current_edge.distance_m = *parsed;
          has_distance = true;
        }
        continue;
      }
      if (auto value = ValueAfterColon(line, "distance")) {
        if (auto parsed = ParseDouble(*value)) {
          current_edge.distance_m = *parsed;
          has_distance = true;
        }
        continue;
      }
      if (auto value = ValueAfterColon(line, "bidirectional")) {
        current_edge.bidirectional = ParseBool(*value);
        continue;
      }
      if (auto value = ValueAfterColon(line, "enabled")) {
        current_edge.enabled = ParseBool(*value);
        continue;
      }
      continue;
    }

    if (in_road_node) {
      if (auto value = ValueAfterColon(line, "id")) {
        current_road_node.id = *value;
        has_node_id = !current_road_node.id.empty();
        continue;
      }
      if (auto value = ValueAfterColon(line, "frame_id")) {
        current_road_node.frame_id = value->empty() ? default_frame_id : *value;
        continue;
      }
      if (auto value = ValueAfterColon(line, "x")) {
        if (auto parsed = ParseDouble(*value)) {
          current_road_node.x = *parsed;
          has_node_x = true;
        }
        continue;
      }
      if (auto value = ValueAfterColon(line, "y")) {
        if (auto parsed = ParseDouble(*value)) {
          current_road_node.y = *parsed;
          has_node_y = true;
        }
        continue;
      }
      if (auto value = ValueAfterColon(line, "yaw")) {
        if (auto parsed = ParseDouble(*value)) {
          current_road_node.yaw = *parsed;
        }
        continue;
      }
      continue;  // road_nodes 条目内的其他字段（如 type）忽略
    }

    if (auto value = ValueAfterColon(line, "id")) {
      current_station.id = *value;
      has_id = !current_station.id.empty();
      continue;
    }
    if (auto value = ValueAfterColon(line, "frame_id")) {
      current_station.frame_id = value->empty() ? default_frame_id : *value;
      continue;
    }
    if (auto value = ValueAfterColon(line, "x")) {
      if (auto parsed = ParseDouble(*value)) {
        current_station.x = *parsed;
        has_x = true;
      }
      continue;
    }
    if (auto value = ValueAfterColon(line, "y")) {
      if (auto parsed = ParseDouble(*value)) {
        current_station.y = *parsed;
        has_y = true;
      }
      continue;
    }
    if (auto value = ValueAfterColon(line, "yaw")) {
      if (auto parsed = ParseDouble(*value)) {
        current_station.yaw = *parsed;
      }
      continue;
    }
  }
  flush_station();
  flush_road_node();
  flush_edge();

  std::string message;
  if (!ValidateStationCatalog(catalog, &message)) {
    return std::nullopt;
  }
  return catalog;
}

const Station* FindStation(const StationCatalog& catalog, const std::string& id) {
  for (const auto& station : catalog.stations) {
    if (station.id == id) {
      return &station;
    }
  }
  return nullptr;
}

std::optional<double> EstimateRouteDistance(
    const StationCatalog& catalog, const std::string& from, const std::string& to) {
  const auto path = FindRoutePath(catalog, from, to);
  if (!path.has_value()) {
    return std::nullopt;
  }
  double distance_m = 0.0;
  for (std::size_t index = 1; index < path->size(); ++index) {
    for (const auto& edge : catalog.edges) {
      const bool forward = edge.from == (*path)[index - 1] && edge.to == (*path)[index];
      const bool reverse =
          edge.bidirectional && edge.to == (*path)[index - 1] && edge.from == (*path)[index];
      if (edge.enabled && (forward || reverse)) {
        distance_m += edge.distance_m;
        break;
      }
    }
  }
  return distance_m;
}

std::optional<std::vector<std::string>> FindRoutePath(
    const StationCatalog& catalog, const std::string& from, const std::string& to) {
  if (from == to) {
    return std::vector<std::string>{from};
  }

  std::vector<std::string> ids;
  ids.reserve(catalog.stations.size() + catalog.road_nodes.size());
  for (const auto& station : catalog.stations) {
    ids.push_back(station.id);
  }
  // 通行节点与站点共用同一图节点集合，支持途经路口的混合图寻路
  for (const auto& road_node : catalog.road_nodes) {
    ids.push_back(road_node.id);
  }
  const auto index_of = [&](const std::string& id) -> std::optional<std::size_t> {
    const auto it = std::find(ids.begin(), ids.end(), id);
    if (it == ids.end()) {
      return std::nullopt;
    }
    return static_cast<std::size_t>(std::distance(ids.begin(), it));
  };

  const auto start = index_of(from);
  const auto goal = index_of(to);
  if (!start.has_value() || !goal.has_value()) {
    return std::nullopt;
  }

  const double infinity = std::numeric_limits<double>::infinity();
  std::vector<double> distance(ids.size(), infinity);
  std::vector<std::optional<std::size_t>> previous(ids.size(), std::nullopt);
  std::vector<bool> visited(ids.size(), false);
  distance[*start] = 0.0;

  for (std::size_t iteration = 0; iteration < ids.size(); ++iteration) {
    std::optional<std::size_t> current;
    for (std::size_t i = 0; i < ids.size(); ++i) {
      if (!visited[i] && (!current.has_value() || distance[i] < distance[*current])) {
        current = i;
      }
    }
    if (!current.has_value() || !std::isfinite(distance[*current])) {
      break;
    }
    if (*current == *goal) {
      break;
    }
    visited[*current] = true;

    for (const auto& edge : catalog.edges) {
      if (!edge.enabled) {
        continue;
      }
      const auto edge_from = index_of(edge.from);
      const auto edge_to = index_of(edge.to);
      if (!edge_from.has_value() || !edge_to.has_value()) {
        continue;
      }
      auto relax = [&](const std::size_t source, const std::size_t target) {
        if (source == *current && distance[source] + edge.distance_m < distance[target]) {
          distance[target] = distance[source] + edge.distance_m;
          previous[target] = source;
        }
      };
      relax(*edge_from, *edge_to);
      if (edge.bidirectional) {
        relax(*edge_to, *edge_from);
      }
    }
  }
  if (!std::isfinite(distance[*goal])) {
    return std::nullopt;
  }
  std::vector<std::string> path;
  for (auto current = *goal;;) {
    path.push_back(ids[current]);
    if (current == *start) {
      break;
    }
    if (!previous[current].has_value()) {
      return std::nullopt;
    }
    current = *previous[current];
  }
  std::reverse(path.begin(), path.end());
  return path;
}

bool ValidateStationCatalog(const StationCatalog& catalog, std::string* message) {
  if (catalog.stations.empty()) {
    if (message != nullptr) {
      *message = "station catalog is empty";
    }
    return false;
  }

  std::unordered_set<std::string> ids;
  for (const auto& station : catalog.stations) {
    if (station.id.empty()) {
      if (message != nullptr) {
        *message = "station id is empty";
      }
      return false;
    }
    if (station.frame_id.empty()) {
      if (message != nullptr) {
        *message = "station frame_id is empty: " + station.id;
      }
      return false;
    }
    if (!std::isfinite(station.x) || !std::isfinite(station.y) || !std::isfinite(station.yaw)) {
      if (message != nullptr) {
        *message = "station pose is not finite: " + station.id;
      }
      return false;
    }
    if (!ids.insert(station.id).second) {
      if (message != nullptr) {
        *message = "duplicate station id: " + station.id;
      }
      return false;
    }
  }

  // road_nodes 与 stations 共用同一 id 空间：集合内与跨集合均不允许重复
  for (const auto& road_node : catalog.road_nodes) {
    if (road_node.id.empty()) {
      if (message != nullptr) {
        *message = "road node id is empty";
      }
      return false;
    }
    if (road_node.frame_id.empty()) {
      if (message != nullptr) {
        *message = "road node frame_id is empty: " + road_node.id;
      }
      return false;
    }
    if (!std::isfinite(road_node.x) || !std::isfinite(road_node.y) ||
        !std::isfinite(road_node.yaw)) {
      if (message != nullptr) {
        *message = "road node pose is not finite: " + road_node.id;
      }
      return false;
    }
    if (!ids.insert(road_node.id).second) {
      if (message != nullptr) {
        *message = "duplicate road node id: " + road_node.id;
      }
      return false;
    }
  }

  for (const auto& edge : catalog.edges) {
    if (edge.from.empty() || edge.to.empty()) {
      if (message != nullptr) {
        *message = "station edge endpoint is empty";
      }
      return false;
    }
    if (ids.find(edge.from) == ids.end()) {
      if (message != nullptr) {
        *message = "station edge has unknown from endpoint: " + edge.from;
      }
      return false;
    }
    if (ids.find(edge.to) == ids.end()) {
      if (message != nullptr) {
        *message = "station edge has unknown to endpoint: " + edge.to;
      }
      return false;
    }
    if (!std::isfinite(edge.distance_m) || edge.distance_m < 0.0) {
      if (message != nullptr) {
        *message = "station edge distance is invalid: " + edge.from + " -> " + edge.to;
      }
      return false;
    }
    for (const auto& point : edge.polyline) {
      if (!std::isfinite(point.first) || !std::isfinite(point.second)) {
        if (message != nullptr) {
          *message = "station edge polyline point is not finite: " + edge.from + " -> " + edge.to;
        }
        return false;
      }
    }
  }

  if (message != nullptr) {
    *message = "ok";
  }
  return true;
}

StationListProjection BuildStationListProjection(const StationCatalog& catalog) {
  StationListProjection projection;
  projection.station_ids.reserve(catalog.stations.size());
  projection.frame_ids.reserve(catalog.stations.size());
  projection.x.reserve(catalog.stations.size());
  projection.y.reserve(catalog.stations.size());
  projection.yaw.reserve(catalog.stations.size());
  for (const auto& station : catalog.stations) {
    projection.station_ids.push_back(station.id);
    projection.frame_ids.push_back(station.frame_id);
    projection.x.push_back(station.x);
    projection.y.push_back(station.y);
    projection.yaw.push_back(station.yaw);
  }
  projection.message = "loaded " + std::to_string(catalog.stations.size()) + " station(s)";
  return projection;
}

StationRouteListProjection BuildStationRouteListProjection(const StationCatalog& catalog) {
  StationRouteListProjection projection;
  projection.from_station.reserve(catalog.edges.size());
  projection.to_station.reserve(catalog.edges.size());
  projection.distance_m.reserve(catalog.edges.size());
  projection.bidirectional.reserve(catalog.edges.size());
  projection.enabled.reserve(catalog.edges.size());
  for (const auto& edge : catalog.edges) {
    projection.from_station.push_back(edge.from);
    projection.to_station.push_back(edge.to);
    projection.distance_m.push_back(edge.distance_m);
    projection.bidirectional.push_back(edge.bidirectional);
    projection.enabled.push_back(edge.enabled);
  }
  projection.message = "loaded " + std::to_string(catalog.edges.size()) + " route edge(s)";
  return projection;
}

// ---- StationCatalogReloader：mtime 轮询热重载，校验失败保留旧目录 ----
StationCatalogReloader::StationCatalogReloader(std::filesystem::path path)
    : path_(std::move(path)),
      watcher_(path_.string(), [this](const YAML::Node&) { TryLoad(true); }) {
  TryLoad(false);   // 初始加载等价节点 on_configure 的一次性读取
  watcher_.Tick();  // 建立 mtime 基线：首拍只记基线不回调，之后的变化才重载
}

bool StationCatalogReloader::Tick() {
  const auto before = reload_count_;
  watcher_.Tick();
  if (!watcher_.last_error().empty()) {
    last_error_ = watcher_.last_error();  // YAML 解析失败：回调未触发，旧目录保持
  }
  return reload_count_ > before;
}

std::string StationCatalogReloader::last_error() const {
  return last_error_;
}

void StationCatalogReloader::TryLoad(bool count_as_reload) {
  if (auto loaded = LoadStationCatalog(path_)) {
    catalog_ = std::move(*loaded);
    loaded_ = true;
    if (count_as_reload) {
      ++reload_count_;
    }
    last_error_.clear();
  } else {
    // 校验失败（空目录、重复 id、非法位姿等）：保留旧目录，记录错误
    last_error_ = "station catalog invalid or unreadable: " + path_.string();
  }
}

}  // namespace amr_dispatcher_core::catalog
