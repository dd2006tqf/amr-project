#pragma once

#include <string>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "amr_dispatcher_core/catalog/station_catalog.hpp"
#include "amr_dispatcher_core/planning/map_geometry_utils.hpp"
#include "amr_dispatcher_core/planning/topology_graph_builder.hpp"

namespace amr_dispatcher_ros {

// 地图导入节点（ROS 薄壳，唯一新增进程）：
//   1. 订阅 /map (nav_msgs/OccupancyGrid, transient_local QoS，兼容 nav2_map_server)。
//   2. 转换为核心库 MapGrid 纯数据结构（零 ROS 依赖）。
//   3. 调用核心库算法流水线：安全空间腐蚀 -> Zhang-Suen 骨架细化 ->
//      受保护毛刺剪枝 -> 拓扑路网构建与站点吸附边拆分。
//   4. 结果成功时以 tmp+rename 原子写出 stations.yaml（调度侧
//      StationCatalogReloader 按 mtime 热重载）；失败时保留旧文件。
//   5. 发布 MarkerArray（transient_local）供 RViz 预览路网折线与站点。
//   6. /map 为 transient_local 单发消息：回调缓存最新地图后，1Hz check_timer_
//      基于缓存补跑——markers 晚于地图就绪、或成功后 markers 被编辑，
//      都能自动（重）生成目录，而不依赖 /map 重发。
class MapIngestNode : public rclcpp::Node {
 public:
  explicit MapIngestNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
  using OccupancyGrid = nav_msgs::msg::OccupancyGrid;

  // /map 回调：校验消息后缓存（连同内容 hash）并立即尝试处理。
  void MapCallback(const OccupancyGrid::SharedPtr msg);

  // 1Hz 补跑定时器：基于缓存地图驱动 ProcessMap（详见类注释 6）。
  void CheckTimerCallback();

  // 完整导入流水线：锚点加载 -> 与 last_ingest_hash_ 去重 -> 算法流水线 ->
  // 原子写出 -> Marker 发布。依赖 last_map_msg_ / last_map_hash_ 成员状态。
  void ProcessMap();

  // 解析 markers_file 的 anchors 段（id/type/x/y/yaw）；未配置/不存在/为空时返回空。
  // 非 const：等待类日志需节流（check_timer_ 会高频重入）。
  std::vector<amr_dispatcher_core::planning::AnchorStation> LoadAnchors();

  // 序列化目录为 Task 1 规定的 schema YAML 文本。
  static std::string SerializeCatalog(
      const amr_dispatcher_core::catalog::StationCatalog& catalog);

  // 原子写出：先写 <output>.tmp 并回读校验（LoadStationCatalog 严格解析），
  // 校验通过才 rename 覆盖正式文件——保证调度端永远不会读到半截/坏文件。
  bool WriteCatalogAtomic(
      const amr_dispatcher_core::catalog::StationCatalog& catalog) const;

  // 发布拓扑 Marker：road_nodes 灰色球、站点按类型着色圆柱 + 浮空文字、
  // 边为 LINE_STRIP 沿真实 polyline 走廊中心线。
  void PublishTopologyMarkers(
      const amr_dispatcher_core::catalog::StationCatalog& catalog);

  // ---- 参数 ----
  std::string map_topic_{"/map"};
  std::string markers_file_{};
  std::string output_stations_file_{"config/generated/stations.yaml"};
  double robot_radius_m_{0.25};
  double safety_margin_m_{0.05};
  double min_branch_length_m_{0.5};
  double max_snap_dist_m_{2.0};
  bool publish_markers_{true};
  std::string marker_topic_{"/dispatcher/topology_markers"};

  // 最近一次成功导入的地图坐标系（供 Marker header.frame_id 使用）。
  std::string map_frame_id_{"map"};
  // 最近一次成功写出的 (map_hash + anchors 摘要)：同图同站点重复消息跳过重写，
  // 避免无谓触发调度侧 StationCatalogReloader 热重载。
  std::string last_ingest_hash_;

  // I2：缓存的最新有效 /map 消息及其内容 hash（FNV-1a，含 origin 与栅格数据）。
  // /map 只发一次，markers 晚就绪/后续编辑时 check_timer_ 用这两者补跑。
  OccupancyGrid::SharedPtr last_map_msg_;
  std::string last_map_hash_;

  rclcpp::Subscription<OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  // 1Hz 补跑定时器：/map 单发不重来，处理失败或 markers 后续变化时自动重试。
  rclcpp::TimerBase::SharedPtr check_timer_;
};

}  // namespace amr_dispatcher_ros
