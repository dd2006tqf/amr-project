#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "amr_dispatcher_core/config/config_loader.hpp"

namespace amr_dispatcher_core::catalog {

struct Station {
  std::string id;
  std::string frame_id = "map";
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  // 业务语义类型（"pickup"/"dropoff"/"charge"/"normal" 等），透传自 AnchorStation。
  std::string type = "normal";
};

// 骨架路网通行节点（路口/转折点）：与站点共用同一 id 空间，
// 使 FindRoutePath 能在 stations + road_nodes 混合图上寻路。
struct RoadNode {
  std::string id;
  std::string frame_id = "map";
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

struct CatalogStationEdge {
  std::string from;
  std::string to;
  double distance_m = 0.0;
  bool bidirectional = true;
  bool enabled = true;
  // 沿骨架提取的真实几何折线（世界坐标米制），供 SkeletonPlanner 循迹展开
  std::vector<std::pair<double, double>> polyline{};
};

struct StationCatalog {
  std::vector<Station> stations;
  std::vector<CatalogStationEdge> edges;
  // 地图包内容指纹与路网算法版本，供调度侧版本门禁比对
  std::string map_hash;
  std::string graph_version;
  std::vector<RoadNode> road_nodes;
};

struct StationListProjection {
  std::vector<std::string> station_ids;
  std::vector<std::string> frame_ids;
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> yaw;
  std::string message;
};

struct StationRouteListProjection {
  std::vector<std::string> from_station;
  std::vector<std::string> to_station;
  std::vector<double> distance_m;
  std::vector<bool> bidirectional;
  std::vector<bool> enabled;
  std::string message;
};

std::optional<StationCatalog> LoadStationCatalog(const std::filesystem::path& path);
const Station* FindStation(const StationCatalog& catalog, const std::string& id);
std::optional<double> EstimateRouteDistance(
    const StationCatalog& catalog, const std::string& from, const std::string& to);
std::optional<std::vector<std::string>> FindRoutePath(
    const StationCatalog& catalog, const std::string& from, const std::string& to);
// 绕开指定路段的寻路（静态拓扑绕行）。
//
// `blocked_edge_keys` 的元素是**资源键**（`route_edge:<lo>__<hi>`，与
// dispatcher::ResourceTable::RouteEdgeId 的输出一致），不是裸边名——
// 绕行与路权预约必须用同一套键，否则"规划时绕开、预约时按另一套算"。
// 两端点同键（无序）——双向边被禁时两个方向都不可走。
// 实现与 FindRoutePath 共用同一个 Dijkstra（不复制第二份寻路）；
// 空集合时行为与 FindRoutePath 逐字一致。
std::optional<std::vector<std::string>> FindRoutePathAvoiding(
    const StationCatalog& catalog, const std::string& from, const std::string& to,
    const std::unordered_set<std::string>& blocked_edge_keys);
bool ValidateStationCatalog(const StationCatalog& catalog, std::string* message);
StationListProjection BuildStationListProjection(const StationCatalog& catalog);
StationRouteListProjection BuildStationRouteListProjection(const StationCatalog& catalog);

// 站点目录热重载器：构造时加载初始目录，Tick 按文件 mtime 变化重载；
// 构造时缺失的文件在后续 Tick 中晚出现也会补加载（覆盖 map_ingest 先启动
// 于 dispatcher 之前写出 generated 目录的闭环场景）；
// 新目录校验失败时保留旧目录（不清空），错误经 last_error 暴露。
class StationCatalogReloader {
 public:
  explicit StationCatalogReloader(std::filesystem::path path);

  // 驱动一次 mtime 检查；返回本次是否完成换新（含晚出现文件的补加载）。
  bool Tick();

  const StationCatalog& catalog() const { return catalog_; }
  bool loaded() const { return loaded_; }
  std::string last_error() const;

 private:
  void TryLoad(bool count_as_reload);

  std::filesystem::path path_;
  StationCatalog catalog_{};
  bool loaded_ = false;
  std::size_t reload_count_ = 0;
  std::string last_error_;
  config::HotReloadWatcher watcher_;
};

}  // namespace amr_dispatcher_core::catalog
