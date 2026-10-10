#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include "amr_dispatcher_core/config/config_loader.hpp"

namespace amr_dispatcher_core::catalog {

struct Station {
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
};

struct StationCatalog {
  std::vector<Station> stations;
  std::vector<CatalogStationEdge> edges;
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
bool ValidateStationCatalog(const StationCatalog& catalog, std::string* message);
StationListProjection BuildStationListProjection(const StationCatalog& catalog);
StationRouteListProjection BuildStationRouteListProjection(const StationCatalog& catalog);

// 站点目录热重载器：构造时加载初始目录，Tick 按文件 mtime 变化重载；
// 新目录校验失败时保留旧目录（不清空），错误经 last_error 暴露。
class StationCatalogReloader {
 public:
  explicit StationCatalogReloader(std::filesystem::path path);

  // 驱动一次 mtime 检查；返回本次是否完成换新。
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
