#include "amr_dispatcher_core/catalog/station_catalog.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cstdio>
#include <fstream>
#include <thread>

namespace amr_dispatcher_core::workflow {
using namespace amr_dispatcher_core::catalog;
namespace {

TEST(StationCatalogTest, BuildStationListProjection_PreservesStationFields) {
  StationCatalog catalog;
  catalog.stations.push_back(Station{"receiving", "map", 1.0, 2.0, 0.25});
  catalog.stations.push_back(Station{"storage", "odom", -3.0, 4.5, -1.5});

  const auto projection = BuildStationListProjection(catalog);

  EXPECT_EQ(projection.station_ids, (std::vector<std::string>{"receiving", "storage"}));
  EXPECT_EQ(projection.frame_ids, (std::vector<std::string>{"map", "odom"}));
  EXPECT_EQ(projection.x, (std::vector<double>{1.0, -3.0}));
  EXPECT_EQ(projection.y, (std::vector<double>{2.0, 4.5}));
  EXPECT_EQ(projection.yaw, (std::vector<double>{0.25, -1.5}));
  EXPECT_EQ(projection.message, "loaded 2 station(s)");
}

TEST(StationCatalogTest, BuildStationRouteListProjection_PreservesRouteFields) {
  StationCatalog catalog;
  catalog.edges.push_back(CatalogStationEdge{"receiving", "storage", 5.5, true, true});
  catalog.edges.push_back(CatalogStationEdge{"storage", "dock", 2.0, false, false});

  const auto projection = BuildStationRouteListProjection(catalog);

  EXPECT_EQ(projection.from_station, (std::vector<std::string>{"receiving", "storage"}));
  EXPECT_EQ(projection.to_station, (std::vector<std::string>{"storage", "dock"}));
  EXPECT_EQ(projection.distance_m, (std::vector<double>{5.5, 2.0}));
  EXPECT_EQ(projection.bidirectional, (std::vector<bool>{true, false}));
  EXPECT_EQ(projection.enabled, (std::vector<bool>{true, false}));
  EXPECT_EQ(projection.message, "loaded 2 route edge(s)");
}

}  // namespace
}  // namespace amr_dispatcher_core::workflow

namespace {

std::string WriteCatalogTemp(const std::string& content) {
  std::string path = std::tmpnam(nullptr);
  std::ofstream out(path);
  out << content;
  return path;
}

constexpr const char* kTwoStations =
    "stations:\n"
    "  - id: s1\n    x: 1.0\n    y: 2.0\n"
    "  - id: s2\n    x: 3.0\n    y: 4.0\n";
constexpr const char* kThreeStations =
    "stations:\n"
    "  - id: s1\n    x: 1.0\n    y: 2.0\n"
    "  - id: s2\n    x: 3.0\n    y: 4.0\n"
    "  - id: s3\n    x: 5.0\n    y: 6.0\n";
constexpr const char* kDuplicateIds =
    "stations:\n"
    "  - id: s1\n    x: 1.0\n    y: 2.0\n"
    "  - id: s1\n    x: 3.0\n    y: 4.0\n";

}  // namespace

// 构造即完成初始加载（等价节点 on_configure 的一次性读取）。
TEST(StationCatalogReloaderTest, LoadsInitialCatalog) {
  auto p = WriteCatalogTemp(kTwoStations);
  amr_dispatcher_core::catalog::StationCatalogReloader reloader(p);
  EXPECT_TRUE(reloader.loaded());
  EXPECT_EQ(reloader.catalog().stations.size(), 2u);
  EXPECT_FALSE(reloader.Tick());  // mtime 未变，不触发重载
  std::remove(p.c_str());
}

// mtime 变化触发重载换新；schema 非法时保留旧目录不清空。
TEST(StationCatalogReloaderTest, ReloadsOnMtimeChangeAndKeepsOldOnInvalid) {
  auto p = WriteCatalogTemp(kTwoStations);
  amr_dispatcher_core::catalog::StationCatalogReloader reloader(p);
  ASSERT_TRUE(reloader.loaded());

  std::this_thread::sleep_for(std::chrono::milliseconds(20));  // 跨过文件系统 mtime 粒度
  {
    std::ofstream out(p, std::ios::trunc);
    out << kThreeStations;
  }
  EXPECT_TRUE(reloader.Tick());
  EXPECT_EQ(reloader.catalog().stations.size(), 3u);

  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  {
    std::ofstream out(p, std::ios::trunc);
    out << kDuplicateIds;
  }
  EXPECT_FALSE(reloader.Tick());                     // 校验失败，没有换新
  EXPECT_EQ(reloader.catalog().stations.size(), 3u); // 旧目录仍在
  EXPECT_FALSE(reloader.last_error().empty());
  std::remove(p.c_str());
}
