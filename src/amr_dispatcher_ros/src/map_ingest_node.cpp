#include "amr_dispatcher_ros/map_ingest_node.hpp"

#include <chrono>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <system_error>
#include <unordered_map>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <yaml-cpp/yaml.h>

#include "amr_dispatcher_core/path_tracking/geometry_utils.hpp"
#include "amr_dispatcher_core/planning/safety_space_filter.hpp"
#include "amr_dispatcher_core/planning/skeleton_extractor.hpp"

namespace amr_dispatcher_ros {
namespace {

using amr_dispatcher_core::catalog::LoadStationCatalog;
using amr_dispatcher_core::catalog::StationCatalog;
using amr_dispatcher_core::planning::AnchorStation;
using amr_dispatcher_core::planning::BinaryGrid;
using amr_dispatcher_core::planning::MapGrid;
using amr_dispatcher_core::planning::PixelCoord;
using amr_dispatcher_core::planning::PruneConfig;
using amr_dispatcher_core::planning::SafetyFilterConfig;
using amr_dispatcher_core::planning::TopologyBuilderConfig;
using amr_dispatcher_core::planning::WorldCoord;

// FNV-1a 64 位内容哈希：同地图必同 hash、异地图（尺寸/分辨率/origin/栅格数据
// 任一变化）必异 hash，供调度侧版本门禁（expected_map_hash）比对与自身去重。
// 直接对 OccupancyGrid 消息计算（I1）：origin（平移/旋转）必须参与——仅平移
// 地图的尺寸与栅格 data 不变，漏掉 origin 会被去重误判为"同图"而跳过重导入；
// 消息级计算也让 check_timer_ 补跑时无需为去重重建 MapGrid。
std::string ComputeMapHash(const nav_msgs::msg::OccupancyGrid& msg) {
  uint64_t h = 14695981039346656037ULL;  // FNV-1a offset basis
  const auto mix = [&h](const void* bytes, std::size_t n) {
    const auto* p = static_cast<const uint8_t*>(bytes);
    for (std::size_t i = 0; i < n; ++i) {
      h ^= p[i];
      h *= 1099511628211ULL;  // FNV prime
    }
  };
  const auto mix_double = [&mix](double v) {
    uint64_t u = 0;
    static_assert(sizeof(u) == sizeof(v), "double must be 64-bit");
    std::memcpy(&u, &v, sizeof(v));
    mix(&u, sizeof(u));
  };
  mix(&msg.info.width, sizeof(msg.info.width));
  mix(&msg.info.height, sizeof(msg.info.height));
  mix(&msg.info.resolution, sizeof(msg.info.resolution));
  mix_double(msg.info.origin.position.x);
  mix_double(msg.info.origin.position.y);
  mix_double(amr_dispatcher_core::path_tracking::YawFromQuaternion(
      msg.info.origin.orientation.z, msg.info.origin.orientation.w));
  if (!msg.data.empty()) {
    mix(msg.data.data(), msg.data.size());
  }
  std::ostringstream out;
  out << "fnv1a64_" << std::hex << std::setfill('0') << std::setw(16) << h;
  return out.str();
}

// 站点锚点摘要：markers 内容变化（新增/移动站点）也要触发重新生成，
// 否则同图重复消息会被跳过。仅摘要参与 hash，不额外读文件系统状态。
std::string ComputeAnchorsDigest(const std::vector<AnchorStation>& anchors) {
  uint64_t h = 14695981039346656037ULL;
  const auto mix_str = [&h](const std::string& s) {
    for (const char c : s) {
      h ^= static_cast<uint8_t>(c);
      h *= 1099511628211ULL;
    }
    h ^= 0xFFu;  // 分隔符，防 "ab"+"c" 与 "a"+"bc" 摘要碰撞
    h *= 1099511628211ULL;
  };
  const auto mix_double = [&h](double v) {
    uint64_t u = 0;
    static_assert(sizeof(u) == sizeof(v), "double must be 64-bit");
    std::memcpy(&u, &v, sizeof(v));
    for (int i = 0; i < 8; ++i) {
      h ^= static_cast<uint8_t>(u >> (i * 8));
      h *= 1099511628211ULL;
    }
  };
  for (const auto& a : anchors) {
    mix_str(a.id);
    mix_str(a.type);
    mix_double(a.x);
    mix_double(a.y);
    mix_double(a.yaw);
  }
  std::ostringstream out;
  out << std::hex << std::setfill('0') << std::setw(16) << h;
  return out.str();
}

// double -> YAML 文本：max_digits10 保证往返解析不丢精度。
std::string FormatDouble(double v) {
  std::ostringstream out;
  out << std::setprecision(std::numeric_limits<double>::max_digits10) << v;
  return out.str();
}

std::string Quote(const std::string& value) {
  return "\"" + value + "\"";
}

std_msgs::msg::ColorRGBA Color(float r, float g, float b, float a = 1.0f) {
  std_msgs::msg::ColorRGBA c;
  c.r = r;
  c.g = g;
  c.b = b;
  c.a = a;
  return c;
}

// 站点类型 -> RViz 颜色：pickup 绿 / dropoff 橙 / charge 黄 / 其他蓝。
std_msgs::msg::ColorRGBA StationColor(const std::string& type) {
  if (type == "pickup") {
    return Color(0.2f, 0.8f, 0.2f);
  }
  if (type == "dropoff") {
    return Color(0.95f, 0.6f, 0.1f);
  }
  if (type == "charge") {
    return Color(0.95f, 0.85f, 0.2f);
  }
  return Color(0.3f, 0.5f, 0.9f);
}

}  // namespace

MapIngestNode::MapIngestNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("map_ingest_node", options) {
  declare_parameter<std::string>("map_topic", "/map");
  declare_parameter<std::string>("markers_file", "");
  declare_parameter<std::string>("output_stations_file", "config/generated/stations.yaml");
  declare_parameter<double>("robot_radius_m", 0.25);
  declare_parameter<double>("safety_margin_m", 0.05);
  declare_parameter<double>("min_branch_length_m", 0.5);
  declare_parameter<double>("max_snap_dist_m", 2.0);
  declare_parameter<bool>("publish_markers", true);
  declare_parameter<std::string>("marker_topic", "/dispatcher/topology_markers");

  map_topic_ = get_parameter("map_topic").as_string();
  markers_file_ = get_parameter("markers_file").as_string();
  output_stations_file_ = get_parameter("output_stations_file").as_string();
  robot_radius_m_ = get_parameter("robot_radius_m").as_double();
  safety_margin_m_ = get_parameter("safety_margin_m").as_double();
  min_branch_length_m_ = get_parameter("min_branch_length_m").as_double();
  max_snap_dist_m_ = get_parameter("max_snap_dist_m").as_double();
  publish_markers_ = get_parameter("publish_markers").as_bool();
  marker_topic_ = get_parameter("marker_topic").as_string();

  // transient_local + reliable：与 nav2_map_server / 静态地图发布者的
  // latched 语义对接——即使 map_ingest_node 启动晚于 map_server 也能收到地图。
  map_sub_ = create_subscription<OccupancyGrid>(
      map_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      std::bind(&MapIngestNode::MapCallback, this, std::placeholders::_1));

  // transient_local：RViz 晚启动也能立即看到完整拓扑。
  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      marker_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());

  // I2：/map 是 latched 单发消息——首次回调若 markers 未就绪则本次作废，此后
  // 由 1Hz 定时器基于缓存地图自动补跑（成功后 markers 被编辑同样经此再生成）。
  check_timer_ = create_wall_timer(std::chrono::seconds(1),
                                   std::bind(&MapIngestNode::CheckTimerCallback, this));

  RCLCPP_INFO(get_logger(),
              "MapIngestNode started: map_topic=%s markers_file='%s' output='%s' "
              "(robot_radius=%.2fm margin=%.2fm min_branch=%.2fm max_snap=%.2fm)",
              map_topic_.c_str(), markers_file_.c_str(), output_stations_file_.c_str(),
              robot_radius_m_, safety_margin_m_, min_branch_length_m_, max_snap_dist_m_);
}

std::vector<AnchorStation> MapIngestNode::LoadAnchors() {
  std::vector<AnchorStation> anchors;
  if (markers_file_.empty()) {
    return anchors;
  }
  std::error_code ec;
  if (!std::filesystem::exists(markers_file_, ec)) {
    // 节流：check_timer_ 会以 1Hz 重入本函数，避免等待窗口刷屏。
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                        "markers_file '%s' does not exist yet; waiting",
                        markers_file_.c_str());
    return anchors;
  }
  try {
    const YAML::Node root = YAML::LoadFile(markers_file_);
    if (!root["anchors"] || !root["anchors"].IsSequence()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                          "markers_file '%s' has no 'anchors' list; waiting",
                          markers_file_.c_str());
      return anchors;
    }
    for (const auto& node : root["anchors"]) {
      AnchorStation anchor;
      anchor.id = node["id"].as<std::string>();
      anchor.type = node["type"].as<std::string>("normal");
      anchor.x = node["x"].as<double>();
      anchor.y = node["y"].as<double>();
      anchor.yaw = node["yaw"].as<double>(0.0);
      if (anchor.id.empty()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                            "markers_file '%s' contains an anchor with empty id; skipped",
                            markers_file_.c_str());
        continue;
      }
      anchors.push_back(std::move(anchor));
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000,
                         "Failed to parse markers_file '%s': %s",
                         markers_file_.c_str(), e.what());
    anchors.clear();
  }
  return anchors;
}

std::string MapIngestNode::SerializeCatalog(const StationCatalog& catalog) {
  // 与 LoadStationCatalog 的逐行严格解析 schema 对齐（Task 1）：
  // version(map_hash/graph_version) -> stations -> road_nodes -> edges(polyline)。
  std::ostringstream out;
  out << "version:\n";
  out << "  map_hash: " << Quote(catalog.map_hash) << "\n";
  out << "  graph_version: " << Quote(catalog.graph_version) << "\n";

  out << "stations:\n";
  for (const auto& station : catalog.stations) {
    out << "  - id: " << Quote(station.id) << "\n";
    out << "    x: " << FormatDouble(station.x) << "\n";
    out << "    y: " << FormatDouble(station.y) << "\n";
    out << "    yaw: " << FormatDouble(station.yaw) << "\n";
    out << "    type: " << Quote(station.type) << "\n";
  }

  out << "road_nodes:\n";
  for (const auto& node : catalog.road_nodes) {
    out << "  - id: " << Quote(node.id) << "\n";
    out << "    x: " << FormatDouble(node.x) << "\n";
    out << "    y: " << FormatDouble(node.y) << "\n";
    out << "    yaw: " << FormatDouble(node.yaw) << "\n";
  }

  out << "edges:\n";
  for (const auto& edge : catalog.edges) {
    out << "  - from: " << Quote(edge.from) << "\n";
    out << "    to: " << Quote(edge.to) << "\n";
    out << "    distance_m: " << FormatDouble(edge.distance_m) << "\n";
    out << "    bidirectional: " << (edge.bidirectional ? "true" : "false") << "\n";
    out << "    enabled: " << (edge.enabled ? "true" : "false") << "\n";
    out << "    polyline:\n";
    for (const auto& point : edge.polyline) {
      out << "      - [" << FormatDouble(point.first) << ", " << FormatDouble(point.second)
          << "]\n";
    }
  }
  return out.str();
}

bool MapIngestNode::WriteCatalogAtomic(const StationCatalog& catalog) const {
  const std::filesystem::path final_path(output_stations_file_);
  const std::filesystem::path tmp_path(final_path.string() + ".tmp");
  std::error_code ec;

  if (final_path.has_parent_path()) {
    std::filesystem::create_directories(final_path.parent_path(), ec);
    if (ec) {
      RCLCPP_ERROR(get_logger(), "Cannot create directory for %s: %s",
                   final_path.parent_path().string().c_str(), ec.message().c_str());
      return false;
    }
  }

  {
    std::ofstream out(tmp_path, std::ios::trunc);
    if (!out) {
      RCLCPP_ERROR(get_logger(), "Cannot open temp file %s for writing", tmp_path.c_str());
      return false;
    }
    out << SerializeCatalog(catalog);
    out.flush();
    if (!out) {
      out.close();
      std::filesystem::remove(tmp_path, ec);
      RCLCPP_ERROR(get_logger(), "Failed writing temp file %s", tmp_path.c_str());
      return false;
    }
  }

  // 回读校验：tmp 文件必须能被调度侧的严格解析器 LoadStationCatalog 接受，
  // 否则删除 tmp、保留旧文件——绝不让半截/坏目录进入热重载路径。
  if (!LoadStationCatalog(tmp_path)) {
    std::filesystem::remove(tmp_path, ec);
    RCLCPP_ERROR(get_logger(),
                 "Round-trip validation failed for %s; generated YAML is not parseable "
                 "by LoadStationCatalog (keeping existing catalog)",
                 tmp_path.c_str());
    return false;
  }

  // 同目录内 rename(2) 为原子操作：调度端要么看到旧文件要么看到新文件。
  std::filesystem::rename(tmp_path, final_path, ec);
  if (ec) {
    std::filesystem::remove(tmp_path, ec);
    RCLCPP_ERROR(get_logger(), "Atomic rename %s -> %s failed: %s", tmp_path.c_str(),
                 final_path.c_str(), ec.message().c_str());
    return false;
  }
  return true;
}

void MapIngestNode::MapCallback(const OccupancyGrid::SharedPtr msg) {
  if (msg->info.width == 0 || msg->info.height == 0) {
    RCLCPP_WARN(get_logger(), "Ignoring invalid map: width=%u height=%u", msg->info.width,
                msg->info.height);
    return;
  }
  const std::size_t expected_cells =
      static_cast<std::size_t>(msg->info.width) * msg->info.height;
  if (msg->data.size() != expected_cells) {
    RCLCPP_WARN(get_logger(), "Ignoring malformed map: data size %zu != %u x %u",
                msg->data.size(), msg->info.width, msg->info.height);
    return;
  }

  // I2：/map 单发（transient_local）不会重来——缓存消息与内容 hash（I1 含
  // origin），本次立即处理；若 markers 未就绪，check_timer_ 之后基于缓存补跑。
  last_map_msg_ = msg;
  last_map_hash_ = ComputeMapHash(*msg);
  ProcessMap();
}

void MapIngestNode::CheckTimerCallback() {
  // 尚未收到 /map，或未配置 markers（anchors 唯一来源）时补跑无意义；
  // 是否需要真正重跑统一交由 ProcessMap 内的 ingest_hash 去重裁决：
  // 首次未成功（last_ingest_hash_ 为空）必然重试；成功后仅当 markers 被
  // 编辑（锚点摘要变化）才会重新生成目录并触发调度侧热重载。
  if (!last_map_msg_ || markers_file_.empty()) {
    return;
  }
  ProcessMap();
}

void MapIngestNode::ProcessMap() {
  if (!last_map_msg_ || last_map_hash_.empty()) {
    return;
  }
  const OccupancyGrid& msg = *last_map_msg_;

  // ---- ① 加载站点锚点（markers.yaml anchors 段） ----
  const std::vector<AnchorStation> anchors = LoadAnchors();
  if (anchors.empty()) {
    // 节流日志：check_timer_ 每秒重试，等待窗口内不刷屏。
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000,
                         "No station anchors available (markers_file: '%s'); waiting for a "
                         "valid markers file before generating the catalog",
                         markers_file_.c_str());
    return;
  }

  // 同图 + 同锚点重复触发：跳过重写，避免无谓触发调度侧热重载。
  const std::string ingest_hash = last_map_hash_ + "#" + ComputeAnchorsDigest(anchors);
  if (ingest_hash == last_ingest_hash_) {
    RCLCPP_DEBUG(get_logger(), "Map and anchors unchanged (%s); skipping re-ingest",
                 last_map_hash_.c_str());
    return;
  }

  // ---- ② OccupancyGrid -> 核心库 MapGrid ----
  MapGrid grid;
  grid.width = msg.info.width;
  grid.height = msg.info.height;
  grid.resolution = msg.info.resolution;
  grid.origin_x = msg.info.origin.position.x;
  grid.origin_y = msg.info.origin.position.y;
  grid.origin_yaw = amr_dispatcher_core::path_tracking::YawFromQuaternion(
      msg.info.origin.orientation.z, msg.info.origin.orientation.w);
  grid.data.assign(msg.data.begin(), msg.data.end());

  // ---- ③ 核心库算法流水线 ----
  SafetyFilterConfig safety_cfg;
  safety_cfg.robot_radius_m = robot_radius_m_;
  safety_cfg.safety_margin_m = safety_margin_m_;

  PruneConfig prune_cfg;
  prune_cfg.min_branch_length_m = min_branch_length_m_;
  prune_cfg.resolution = grid.resolution;

  TopologyBuilderConfig builder_cfg;
  builder_cfg.max_snap_dist_m = max_snap_dist_m_;
  builder_cfg.map_hash = last_map_hash_;
  builder_cfg.graph_version = "v1";

  const BinaryGrid safety_grid = amr_dispatcher_core::planning::FilterSafetySpace(grid, safety_cfg);
  const BinaryGrid skeleton = amr_dispatcher_core::planning::ZhangSuenThinning(safety_grid);

  std::vector<PixelCoord> protected_points;
  protected_points.reserve(anchors.size());
  for (const auto& anchor : anchors) {
    protected_points.push_back(
        amr_dispatcher_core::planning::WorldToPixel(WorldCoord{anchor.x, anchor.y}, grid));
  }
  const BinaryGrid pruned =
      amr_dispatcher_core::planning::PruneSkeletonProtected(skeleton, protected_points, prune_cfg);

  // ---- ④ 拓扑构建与站点吸附 ----
  const auto result = amr_dispatcher_core::planning::BuildTopologyCatalog(pruned, grid, anchors,
                                                                          builder_cfg);
  if (!result.success) {
    // 节流：失败由 check_timer_ 每秒重试（last_ingest_hash_ 仍为空），不刷屏。
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000,
                         "Map ingest failed: %s (keeping existing catalog; will retry)",
                         result.error_message.c_str());
    return;
  }

  // ---- ⑤ 原子写出 + RViz Marker ----
  if (!WriteCatalogAtomic(result.catalog)) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000,
                         "Failed to write catalog to '%s' (keeping existing catalog; will retry)",
                         output_stations_file_.c_str());
    return;
  }
  last_ingest_hash_ = ingest_hash;
  if (!msg.header.frame_id.empty()) {
    map_frame_id_ = msg.header.frame_id;
  }

  RCLCPP_INFO(get_logger(),
              "Successfully ingested map and generated %s (%zu stations, %zu road nodes, "
              "%zu edges) [map_hash=%s]",
              output_stations_file_.c_str(), result.catalog.stations.size(),
              result.catalog.road_nodes.size(), result.catalog.edges.size(),
              last_map_hash_.c_str());

  if (publish_markers_) {
    PublishTopologyMarkers(result.catalog);
  }
}

void MapIngestNode::PublishTopologyMarkers(const StationCatalog& catalog) {
  visualization_msgs::msg::MarkerArray array;
  const auto stamp = now();

  // 先 DELETEALL 清残影：重新导入（地图更新）时上一批 Marker 全部作废。
  visualization_msgs::msg::Marker clear;
  clear.header.frame_id = map_frame_id_;
  clear.header.stamp = stamp;
  clear.ns = "topology_reset";
  clear.id = 0;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  array.markers.push_back(clear);

  int id = 0;
  const auto base_marker = [&](const std::string& ns, int type) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = map_frame_id_;
    marker.header.stamp = stamp;
    marker.ns = ns;
    marker.id = id++;
    marker.type = type;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    // 永久驻留（lifetime=0）+ transient_local 发布：晚加入的 RViz 也能看到。
    return marker;
  };

  // 端点查找表：折线退化（<2 点）时回退用站点/节点坐标补两个端点。
  std::unordered_map<std::string, std::pair<double, double>> endpoint_xy;
  for (const auto& node : catalog.road_nodes) {
    endpoint_xy[node.id] = {node.x, node.y};
    auto marker = base_marker("road_nodes", visualization_msgs::msg::Marker::SPHERE);
    marker.pose.position.x = node.x;
    marker.pose.position.y = node.y;
    marker.pose.position.z = 0.05;
    marker.scale.x = 0.15;
    marker.scale.y = 0.15;
    marker.scale.z = 0.15;
    marker.color = Color(0.65f, 0.65f, 0.65f, 0.9f);
    array.markers.push_back(marker);
  }

  for (const auto& station : catalog.stations) {
    endpoint_xy[station.id] = {station.x, station.y};
    auto marker = base_marker("stations", visualization_msgs::msg::Marker::CYLINDER);
    marker.pose.position.x = station.x;
    marker.pose.position.y = station.y;
    marker.pose.position.z = 0.05;
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.1;
    marker.color = StationColor(station.type);
    array.markers.push_back(marker);

    auto label = base_marker("station_labels", visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
    label.pose.position.x = station.x;
    label.pose.position.y = station.y;
    label.pose.position.z = 0.5;
    label.scale.z = 0.25;
    label.color = Color(1.0f, 1.0f, 1.0f, 0.95f);
    label.text = station.id;
    array.markers.push_back(label);
  }

  // 边：LINE_STRIP 沿真实 polyline 走廊中心线（而非两点直线）。
  for (const auto& edge : catalog.edges) {
    auto marker = base_marker("edges", visualization_msgs::msg::Marker::LINE_STRIP);
    marker.scale.x = 0.06;  // 线宽（米）
    marker.color = Color(0.1f, 0.8f, 0.9f, 0.9f);
    if (edge.polyline.size() >= 2) {
      marker.points.reserve(edge.polyline.size());
      for (const auto& point : edge.polyline) {
        geometry_msgs::msg::Point p;
        p.x = point.first;
        p.y = point.second;
        p.z = 0.03;
        marker.points.push_back(p);
      }
    } else {
      // 折线缺失的退化边：以两端点连线兜底，保证拓扑关系仍可视化。
      const auto from = endpoint_xy.find(edge.from);
      const auto to = endpoint_xy.find(edge.to);
      if (from == endpoint_xy.end() || to == endpoint_xy.end()) {
        continue;
      }
      geometry_msgs::msg::Point a;
      a.x = from->second.first;
      a.y = from->second.second;
      a.z = 0.03;
      geometry_msgs::msg::Point b;
      b.x = to->second.first;
      b.y = to->second.second;
      b.z = 0.03;
      marker.points.push_back(a);
      marker.points.push_back(b);
    }
    array.markers.push_back(marker);
  }

  marker_pub_->publish(array);
  RCLCPP_INFO(get_logger(), "Published topology markers: %zu stations, %zu road nodes, "
              "%zu edges on %s",
              catalog.stations.size(), catalog.road_nodes.size(), catalog.edges.size(),
              marker_topic_.c_str());
}

}  // namespace amr_dispatcher_ros

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<amr_dispatcher_ros::MapIngestNode>());
  rclcpp::shutdown();
  return 0;
}
