#include <chrono>
#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <yaml-cpp/yaml.h>

#include "amr_dispatcher_interfaces/msg/topology_state.hpp"

using namespace std::chrono_literals;

namespace amr_dispatcher_ros {

struct StationInfo {
  std::string id;
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  std::string type;
};

class DispatcherVisualizerNode : public rclcpp::Node {
 public:
  explicit DispatcherVisualizerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : rclcpp::Node("dispatcher_visualizer_node", options) {
    declare_parameter("frame_id", "map");
    declare_parameter("marker_topic", "/dispatcher/markers");
    declare_parameter("stations_file", "config/stations.yaml");

    frame_id_ = get_parameter("frame_id").as_string();
    marker_topic_ = get_parameter("marker_topic").as_string();
    stations_file_ = get_parameter("stations_file").as_string();

    LoadStations(stations_file_);

    // transient_local 保证 RViz 开启即收到完整的路网
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        marker_topic_, rclcpp::QoS(1).transient_local());

    topology_sub_ = create_subscription<amr_dispatcher_interfaces::msg::TopologyState>(
        "/dispatcher/topology_state", 10,
        [this](const amr_dispatcher_interfaces::msg::TopologyState::SharedPtr msg) {
          latest_topology_ = *msg;
          has_topology_ = true;
          PublishMarkers();
        });

    timer_ = create_wall_timer(500ms, [this]() { PublishMarkers(); });
    RCLCPP_INFO(get_logger(), "Dispatcher Visualizer started on topic: %s", marker_topic_.c_str());
  }

 private:
  void LoadStations(const std::string& filepath) {
    try {
      YAML::Node root = YAML::LoadFile(filepath);
      if (root["stations"]) {
        for (const auto& node : root["stations"]) {
          StationInfo st;
          st.id = node["id"].as<std::string>();
          st.x = node["x"].as<double>();
          st.y = node["y"].as<double>();
          st.yaw = node["yaw"].as<double>(0.0);
          st.type = node["type"].as<std::string>("station");
          stations_[st.id] = st;
        }
      }
      RCLCPP_INFO(get_logger(), "Loaded %zu stations from %s", stations_.size(), filepath.c_str());
    } catch (const std::exception& e) {
      RCLCPP_WARN(get_logger(), "Failed to load stations YAML: %s, using defaults", e.what());
      // 默认提供基准拓扑站点
      stations_["station_pickup_A"] = {"station_pickup_A", 1.0, 2.0, 0.0, "pickup"};
      stations_["station_pickup_B"] = {"station_pickup_B", 3.0, 2.0, 0.0, "pickup"};
      stations_["station_dropoff_1"] = {"station_dropoff_1", 8.0, 5.0, 1.57, "dropoff"};
      stations_["station_charge_1"] = {"station_charge_1", 0.0, 0.0, 3.14, "charge"};
    }
  }

  visualization_msgs::msg::Marker BaseMarker(int id, const std::string& ns, int type) const {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = now();
    marker.ns = ns;
    marker.id = id;
    marker.type = type;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    // 设置 1.2s 生命周期，保证过期 Marker 自动消除
    marker.lifetime = rclcpp::Duration::from_seconds(1.2);
    return marker;
  }

  std_msgs::msg::ColorRGBA Color(float r, float g, float b, float a = 1.0f) const {
    std_msgs::msg::ColorRGBA c;
    c.r = r;
    c.g = g;
    c.b = b;
    c.a = a;
    return c;
  }

  geometry_msgs::msg::Point Point(double x, double y, double z = 0.0) const {
    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
  }

  void PublishMarkers() {
    visualization_msgs::msg::MarkerArray array;
    int id = 0;

    // 1. 全局清除重置帧，防残影
    auto reset = BaseMarker(id++, "reset", visualization_msgs::msg::Marker::CUBE);
    reset.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(reset);

    // 2. 静态站点 (Cylinder) 与浮空文字 (Text)
    for (const auto& [st_id, st] : stations_) {
      auto st_marker = BaseMarker(id++, "stations", visualization_msgs::msg::Marker::CYLINDER);
      st_marker.pose.position = Point(st.x, st.y, 0.02);
      st_marker.scale.x = 0.5;
      st_marker.scale.y = 0.5;
      st_marker.scale.z = 0.05;
      if (st.type == "charge") {
        st_marker.color = Color(0.1f, 0.85f, 0.3f, 0.7f); // 绿色充电桩
      } else if (st.type == "dropoff") {
        st_marker.color = Color(0.9f, 0.5f, 0.1f, 0.7f); // 橙色出库
      } else {
        st_marker.color = Color(0.2f, 0.5f, 1.0f, 0.6f); // 蓝色装配区
      }
      array.markers.push_back(st_marker);

      auto text_marker = BaseMarker(id++, "station_labels", visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
      text_marker.pose.position = Point(st.x, st.y, 0.35);
      text_marker.scale.z = 0.22;
      text_marker.color = Color(0.15f, 0.2f, 0.3f, 0.95f);
      text_marker.text = st_id;
      array.markers.push_back(text_marker);
    }

    // 3. 基础路网连线 (灰色细线)
    auto route_marker = BaseMarker(id++, "station_routes", visualization_msgs::msg::Marker::LINE_LIST);
    route_marker.scale.x = 0.04;
    route_marker.color = Color(0.65f, 0.72f, 0.8f, 0.4f);

    std::vector<std::string> st_names;
    for (const auto& [name, _] : stations_) st_names.push_back(name);
    for (size_t i = 0; i < st_names.size(); ++i) {
      for (size_t j = i + 1; j < st_names.size(); ++j) {
        const auto& p1 = stations_[st_names[i]];
        const auto& p2 = stations_[st_names[j]];
        route_marker.points.push_back(Point(p1.x, p1.y, 0.01));
        route_marker.points.push_back(Point(p2.x, p2.y, 0.01));
      }
    }
    array.markers.push_back(route_marker);

    // 4. 动态拓扑锁 (高亮绿色加粗折线)
    if (has_topology_) {
      for (size_t i = 0; i < latest_topology_.locked_edges.size(); ++i) {
        const auto& edge = latest_topology_.locked_edges[i];
        // 解析形如 "route_edge:A__B"
        std::string owner = (i < latest_topology_.edge_owners.size()) ? latest_topology_.edge_owners[i] : "locked";
        auto edge_marker = BaseMarker(id++, "traffic_locked_edges", visualization_msgs::msg::Marker::LINE_LIST);
        edge_marker.scale.x = 0.12;
        edge_marker.color = Color(0.05f, 0.95f, 0.25f, 0.9f); // 高亮绿

        std::string prefix = "route_edge:";
        size_t p_pos = edge.find(prefix);
        if (p_pos != std::string::npos) {
          std::string pair_str = edge.substr(prefix.size());
          size_t sep = pair_str.find("__");
          if (sep != std::string::npos) {
            std::string from = pair_str.substr(0, sep);
            std::string to = pair_str.substr(sep + 2);
            if (stations_.count(from) && stations_.count(to)) {
              const auto& f_pt = stations_[from];
              const auto& t_pt = stations_[to];
              edge_marker.points.push_back(Point(f_pt.x, f_pt.y, 0.03));
              edge_marker.points.push_back(Point(t_pt.x, t_pt.y, 0.03));
              array.markers.push_back(edge_marker);

              // 路段中间漂浮文字提示锁持有者
              auto lock_text = BaseMarker(id++, "traffic_locked_edges", visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
              lock_text.pose.position = Point((f_pt.x + t_pt.x) * 0.5, (f_pt.y + t_pt.y) * 0.5, 0.4);
              lock_text.scale.z = 0.18;
              lock_text.color = Color(0.05f, 0.7f, 0.15f, 0.95f);
              lock_text.text = "[LOCKED: " + owner + "]";
              array.markers.push_back(lock_text);
            }
          }
        }
      }

      // 5. 死锁冲突警戒圆柱 (半透明红色圆柱 + 3D 文字警报)
      if (latest_topology_.deadlock_detected) {
        for (const auto& node_id : latest_topology_.deadlocked_nodes) {
          double x = 0.0, y = 0.0;
          if (stations_.count(node_id)) {
            x = stations_[node_id].x;
            y = stations_[node_id].y;
          }
          auto deadlock_col = BaseMarker(id++, "deadlock_warning", visualization_msgs::msg::Marker::CYLINDER);
          deadlock_col.pose.position = Point(x, y, 0.5);
          deadlock_col.scale.x = 1.2;
          deadlock_col.scale.y = 1.2;
          deadlock_col.scale.z = 1.0;
          deadlock_col.color = Color(1.0f, 0.05f, 0.05f, 0.5f); // 红色警戒光柱
          array.markers.push_back(deadlock_col);

          auto deadlock_text = BaseMarker(id++, "deadlock_warning", visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
          deadlock_text.pose.position = Point(x, y, 1.2);
          deadlock_text.scale.z = 0.28;
          deadlock_text.color = Color(1.0f, 0.1f, 0.1f, 1.0f);
          deadlock_text.text = "🚨 DEADLOCK ALERT: " + node_id;
          array.markers.push_back(deadlock_text);
        }
      }
    }

    marker_pub_->publish(array);
  }

  std::string frame_id_{"map"};
  std::string marker_topic_{"/dispatcher/markers"};
  std::string stations_file_{"config/stations.yaml"};
  std::map<std::string, StationInfo> stations_;

  bool has_topology_{false};
  amr_dispatcher_interfaces::msg::TopologyState latest_topology_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Subscription<amr_dispatcher_interfaces::msg::TopologyState>::SharedPtr topology_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace amr_dispatcher_ros

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<amr_dispatcher_ros::DispatcherVisualizerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
