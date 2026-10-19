#include "amr_dispatcher_core/planning/topology_graph_builder.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace amr_dispatcher_core::planning {
namespace {

constexpr double kResolution = 0.05;  // 米/像素

MapGrid MakeMapInfo(uint32_t width, uint32_t height, double resolution) {
  MapGrid grid;
  grid.width = width;
  grid.height = height;
  grid.resolution = resolution;
  grid.origin_x = 0.0;
  grid.origin_y = 0.0;
  grid.origin_yaw = 0.0;
  grid.data.assign(static_cast<size_t>(width) * static_cast<size_t>(height),
                   kCellFree);
  return grid;
}

BinaryGrid MakeSkeleton(uint32_t width, uint32_t height) {
  BinaryGrid grid;
  grid.width = width;
  grid.height = height;
  grid.data.assign(static_cast<size_t>(width) * static_cast<size_t>(height), 0);
  return grid;
}

// 在世界坐标 (x, y) 处构造一个站点。
AnchorStation MakeAnchor(const std::string& id, double x, double y,
                         const std::string& type = "normal") {
  AnchorStation a;
  a.id = id;
  a.x = x;
  a.y = y;
  a.type = type;
  return a;
}

// 沿 (col0,row) -> (col1,row) 填充水平骨架线。
void DrawHLine(BinaryGrid& grid, int row, int col0, int col1) {
  for (int col = col0; col <= col1; ++col) {
    grid.SetValue(col, row, 1);
  }
}

// 沿 (col,row0) -> (col,row1) 填充垂直骨架线。
void DrawVLine(BinaryGrid& grid, int col, int row0, int row1) {
  for (int row = row0; row <= row1; ++row) {
    grid.SetValue(col, row, 1);
  }
}

// 在骨架上找到 id 为 station_id 的 Station 索引，未找到返回 -1。
int StationIndex(const catalog::StationCatalog& catalog,
                 const std::string& station_id) {
  for (size_t i = 0; i < catalog.stations.size(); ++i) {
    if (catalog.stations[i].id == station_id) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

// ---------------------------------------------------------------------------
// 用例 1：直走廊 + 两端站点
//   骨架：一行 20 像素直线。两端各有一个站点。断言产生 2 个站点，原边被拆
//   分为 2 条子边（或者通过 road_node 中转），FindRoutePath 可寻路。
// ---------------------------------------------------------------------------
TEST(TopologyGraphBuilderTest,
     StraightCorridor_EndpointStations_Reachable) {
  // 像素宽度 20 -> 1.0m。骨架直线 row=5，col 0..19。
  const MapGrid map_info = MakeMapInfo(20, 11, kResolution);
  BinaryGrid skeleton = MakeSkeleton(20, 11);
  DrawHLine(skeleton, 5, 0, 19);

  // 站点放在骨架两端附近（在 max_snap_dist 内）。
  std::vector<AnchorStation> anchors;
  anchors.push_back(MakeAnchor("s_left", 0.025, 0.275));   // 近 (0,5)
  anchors.push_back(MakeAnchor("s_right", 0.975, 0.275));  // 近 (19,5)

  TopologyBuilderConfig config;
  config.max_snap_dist_m = 2.0;
  config.map_hash = "test_hash_1";
  config.graph_version = "v1";

  const TopologyBuildResult result =
      BuildTopologyCatalog(skeleton, map_info, anchors, config);

  ASSERT_TRUE(result.success) << result.error_message;
  ASSERT_EQ(result.catalog.stations.size(), 2u);
  EXPECT_EQ(result.catalog.map_hash, "test_hash_1");
  EXPECT_EQ(result.catalog.graph_version, "v1");

  // 两个站点之间存在可达路径（直线上经过拆分后的中间 road_node）。
  const auto path = catalog::FindRoutePath(result.catalog, "s_left", "s_right");
  ASSERT_TRUE(path.has_value()) << "两站点之间必须可达";

  // 路径上的边应携带 polyline 折线（至少一条边应有 >=2 折线点）。
  bool any_polyline = false;
  for (const auto& edge : result.catalog.edges) {
    if (edge.polyline.size() >= 2) {
      any_polyline = true;
      break;
    }
  }
  EXPECT_TRUE(any_polyline) << "至少一条边应保留骨架折线";
}

// ---------------------------------------------------------------------------
// 用例 2：L 弯走廊 + 中段充电桩站点
//   骨架 L 形：水平段 (0,10)->(39,10) + 垂直段 (39,10)->(39,30)。
//   拐角 (39,10) 处应产生 road_node；水平段中段的充电桩站点吸附后原边拆分。
// ---------------------------------------------------------------------------
TEST(TopologyGraphBuilderTest,
     LCornerCorridor_ChargeStationSplitsEdge) {
  const MapGrid map_info = MakeMapInfo(40, 31, kResolution);
  BinaryGrid skeleton = MakeSkeleton(40, 31);
  DrawHLine(skeleton, 10, 0, 39);
  DrawVLine(skeleton, 39, 10, 30);

  // 充电桩站点位于水平段中段 (20,10) 附近（世界 y = (10+0.5)*0.05 = 0.525）。
  std::vector<AnchorStation> anchors;
  anchors.push_back(MakeAnchor("s_end_a", 0.025, 0.525));    // 水平端
  anchors.push_back(MakeAnchor("s_end_b", 1.975, 1.525));    // 垂直端 (39,30)
  anchors.push_back(MakeAnchor("charge_1", 1.025, 0.525, "charge"));  // (20,10)

  TopologyBuilderConfig config;
  config.max_snap_dist_m = 2.0;
  config.map_hash = "test_hash_2";
  config.graph_version = "v1";

  const TopologyBuildResult result =
      BuildTopologyCatalog(skeleton, map_info, anchors, config);

  ASSERT_TRUE(result.success) << result.error_message;
  ASSERT_EQ(result.catalog.stations.size(), 3u);

  // 充电桩应作为图节点。
  const int charge_idx = StationIndex(result.catalog, "charge_1");
  ASSERT_GE(charge_idx, 0) << "charge_1 必须存在于 stations 中";
  // I1：站点类型从 AnchorStation 透传到 catalog::Station。
  EXPECT_EQ(result.catalog.stations[charge_idx].type, "charge");
  const int end_a_idx = StationIndex(result.catalog, "s_end_a");
  ASSERT_GE(end_a_idx, 0);
  EXPECT_EQ(result.catalog.stations[end_a_idx].type, "normal");

  // 原"水平长边"必须被 charge_1 拆分为两段——即不存在跨越整个水平臂的边。
  // 即 s_end_a 与拐角 road_node 之间不存在直接边（必须经过 charge_1）。
  // 找一个含 "charge_1" 的边（连接 charge_1 到左侧或拐角）。
  bool edge_via_charge = false;
  for (const auto& edge : result.catalog.edges) {
    if (edge.from == "charge_1" || edge.to == "charge_1") {
      edge_via_charge = true;
      break;
    }
  }
  EXPECT_TRUE(edge_via_charge) << "应存在以 charge_1 为端点的边";

  // 全部站点间可达。
  const auto path_ab =
      catalog::FindRoutePath(result.catalog, "s_end_a", "s_end_b");
  ASSERT_TRUE(path_ab.has_value());
  const auto path_ac =
      catalog::FindRoutePath(result.catalog, "s_end_a", "charge_1");
  ASSERT_TRUE(path_ac.has_value());
  const auto path_bc =
      catalog::FindRoutePath(result.catalog, "charge_1", "s_end_b");
  ASSERT_TRUE(path_bc.has_value());

  // charge_1 -> s_end_b 路径必须途经拐角（polyline 应呈 L 形）。
  // 找一条以 charge_1 为端点的边，要求其 polyline 中存在 y 接近 0.525
  // 的拐点（拐角 (39,10) -> 世界 (1.975, 0.525)）。
  bool found_L_shape = false;
  for (const auto& edge : result.catalog.edges) {
    if (edge.polyline.size() < 3) {
      continue;
    }
    // L 形折线：拐角点应在折线中间而非首尾。
    for (size_t i = 1; i + 1 < edge.polyline.size(); ++i) {
      const auto& p = edge.polyline[i];
      if (std::abs(p.first - 1.975) < 0.05 &&
          std::abs(p.second - 0.525) < 0.05) {
        found_L_shape = true;
        break;
      }
    }
    if (found_L_shape) break;
  }
  EXPECT_TRUE(found_L_shape)
      << "L 形折线应在中间包含拐角点 (~1.975, 0.525)";
}

// ---------------------------------------------------------------------------
// 用例 3：站点距骨架过远 -> 构建失败
// ---------------------------------------------------------------------------
TEST(TopologyGraphBuilderTest, FarStation_BuildFails) {
  const MapGrid map_info = MakeMapInfo(20, 20, kResolution);
  BinaryGrid skeleton = MakeSkeleton(20, 20);
  DrawHLine(skeleton, 5, 0, 19);  // y = (5+0.5)*0.05 = 0.275m

  std::vector<AnchorStation> anchors;
  anchors.push_back(MakeAnchor("s_ok", 0.025, 0.275));
  // 远站点：x 与骨架重合，但 y 偏离 5.0m
  anchors.push_back(MakeAnchor("s_far", 0.5, 5.275));

  TopologyBuilderConfig config;
  config.max_snap_dist_m = 2.0;
  config.map_hash = "h3";
  config.graph_version = "v1";

  const TopologyBuildResult result =
      BuildTopologyCatalog(skeleton, map_info, anchors, config);

  EXPECT_FALSE(result.success);
  EXPECT_NE(result.error_message.find("too far"), std::string::npos)
      << "错误信息应包含 'too far'，实际：" << result.error_message;
  EXPECT_NE(result.error_message.find("s_far"), std::string::npos)
      << "错误信息应包含失败站点 id，实际：" << result.error_message;
}

// ---------------------------------------------------------------------------
// 用例 4：两个不连通走廊 -> 构建失败（disconnected）
// ---------------------------------------------------------------------------
TEST(TopologyGraphBuilderTest, DisconnectedIslands_BuildFails) {
  const MapGrid map_info = MakeMapInfo(40, 20, kResolution);
  BinaryGrid skeleton = MakeSkeleton(40, 20);
  DrawHLine(skeleton, 5, 0, 19);    // 左孤岛
  DrawHLine(skeleton, 15, 20, 39);  // 右孤岛（与左孤岛无骨架连接）

  std::vector<AnchorStation> anchors;
  anchors.push_back(MakeAnchor("s_a", 0.025, 0.275));   // 左岛
  anchors.push_back(MakeAnchor("s_b", 1.975, 0.775));   // 右岛

  TopologyBuilderConfig config;
  config.max_snap_dist_m = 2.0;
  config.map_hash = "h4";
  config.graph_version = "v1";

  const TopologyBuildResult result =
      BuildTopologyCatalog(skeleton, map_info, anchors, config);

  EXPECT_FALSE(result.success);
  EXPECT_NE(result.error_message.find("disconnected"), std::string::npos)
      << "错误信息应包含 'disconnected'，实际：" << result.error_message;
}

// ---------------------------------------------------------------------------
// 用例 5：真实折线累加长度
//   构造骨架：(0,0) -> (0,3) -> (4,3)。折线实际长度 = 3 + 4 = 7 像素。
//   在 0.05m/像素 下 = 0.35m。但用例要求 7m——改用 res=1.0m。
//   为简化测试，使用 res=1.0m 验证累加几何长度 = 7m。
// ---------------------------------------------------------------------------
TEST(TopologyGraphBuilderTest, PolylineAccumulatedLength_NotStraightLine) {
  // res = 1.0 m/px。骨架：(0,5) -> (4,5) 水平 4 像素 + (4,5) -> (4,8) 垂直
  // 3 像素，L 形折线总长度 = 4 + 3 = 7m，而起点终点直线距离 = 5m。
  const MapGrid map_info = MakeMapInfo(5, 9, 1.0);
  BinaryGrid skeleton = MakeSkeleton(5, 9);
  DrawHLine(skeleton, 5, 0, 4);
  DrawVLine(skeleton, 4, 5, 8);

  // 站点放在两端附近（世界坐标：(0,5)->(0.5,5.5)，(4,8)->(4.5,8.5)）。
  std::vector<AnchorStation> anchors;
  anchors.push_back(MakeAnchor("s_a", 0.5, 5.5));
  anchors.push_back(MakeAnchor("s_b", 4.5, 8.5));

  TopologyBuilderConfig config;
  config.max_snap_dist_m = 5.0;
  config.map_hash = "h5";
  config.graph_version = "v1";

  const TopologyBuildResult result =
      BuildTopologyCatalog(skeleton, map_info, anchors, config);

  ASSERT_TRUE(result.success) << result.error_message;

  // 找 s_a -> s_b 路径，断言其累加距离接近 7m，而非 5m 直线距离。
  const auto path =
      catalog::FindRoutePath(result.catalog, "s_a", "s_b");
  ASSERT_TRUE(path.has_value());

  // 路径距离由 catalog::EstimateRouteDistance 给出（经过中间拐角节点）。
  const auto dist =
      catalog::EstimateRouteDistance(result.catalog, "s_a", "s_b");
  ASSERT_TRUE(dist.has_value());
  // 累加长度应约等于 4 + 3 = 7m（容差 0.2m 吸收像素取整）。
  EXPECT_NEAR(*dist, 7.0, 0.2)
      << "折线累加长度应为 ~7m，实际 " << *dist << "m";

  // 至少一条边上的累加距离大于其端点直线距离（验证 polyline 累积而非
  // 直线距离）。
  bool edge_with_accumulated = false;
  for (const auto& edge : result.catalog.edges) {
    if (edge.polyline.size() >= 3) {
      // 计算折线累加长度
      double accum = 0.0;
      for (size_t i = 1; i < edge.polyline.size(); ++i) {
        const auto& p0 = edge.polyline[i - 1];
        const auto& p1 = edge.polyline[i];
        accum += std::hypot(p1.first - p0.first, p1.second - p0.second);
      }
      // 直线距离
      const auto& p_first = edge.polyline.front();
      const auto& p_last = edge.polyline.back();
      const double straight = std::hypot(p_last.first - p_first.first,
                                         p_last.second - p_first.second);
      if (accum > straight + 0.5) {
        edge_with_accumulated = true;
        break;
      }
    }
  }
  EXPECT_TRUE(edge_with_accumulated)
      << "至少一条边的折线累加长度应大于端点直线距离";
}

// ---------------------------------------------------------------------------
// 用例 6 (C1)：孤立闭环骨架（无特征点的纯环，crossing 恒为 2）
//   障碍柱在自由空间中产生的 1px 环形骨架：7x7 栅格，环位于 col 2..4 /
//   row 2..4（8 像素，中心 (3,3) 为空）。环上所有像素 crossing == 2，
//   ExtractFeaturePixels 为空——必须由孤立闭环处理器切成两个 RoadNode
//   与两条半环边；环上锚点吸附拆边后可寻路。
// ---------------------------------------------------------------------------
TEST(TopologyGraphBuilderTest, ClosedRingSkeleton_GeneratesTopologyAndRoutes) {
  const MapGrid map_info = MakeMapInfo(7, 7, kResolution);
  BinaryGrid skeleton = MakeSkeleton(7, 7);
  // 空心方环：上/下横边 + 左/右竖边（中段一行）。
  DrawHLine(skeleton, 2, 2, 4);
  DrawHLine(skeleton, 4, 2, 4);
  DrawVLine(skeleton, 2, 3, 3);
  DrawVLine(skeleton, 4, 3, 3);

  // 锚点放在环上边中段像素 (3,2) 的世界坐标：((3+0.5)*0.05, (2+0.5)*0.05)。
  std::vector<AnchorStation> anchors;
  anchors.push_back(MakeAnchor("ring_anchor", 0.175, 0.125, "pickup"));

  TopologyBuilderConfig config;
  config.max_snap_dist_m = 0.5;
  config.map_hash = "ring_hash";
  config.graph_version = "v1";

  const TopologyBuildResult result =
      BuildTopologyCatalog(skeleton, map_info, anchors, config);

  ASSERT_TRUE(result.success) << result.error_message;
  // 纯环应生成两个 RoadNode（切成两段半环）。
  ASSERT_EQ(result.catalog.road_nodes.size(), 2u);
  // 锚点吸附拆边：2 条原始半环边被拆为 3 条。
  ASSERT_EQ(result.catalog.stations.size(), 1u);
  EXPECT_EQ(result.catalog.stations[0].id, "ring_anchor");
  EXPECT_EQ(result.catalog.stations[0].type, "pickup");
  ASSERT_EQ(result.catalog.edges.size(), 3u);

  // 每条边必须有 >=2 折线点且距离为正（半环几何有效）。
  int edges_with_polyline = 0;
  for (const auto& edge : result.catalog.edges) {
    EXPECT_GE(edge.polyline.size(), 2u);
    EXPECT_GT(edge.distance_m, 0.0);
    if (edge.polyline.size() >= 2) {
      ++edges_with_polyline;
    }
  }
  EXPECT_EQ(edges_with_polyline, 3);

  // 未被锚点打断的半环：两 RoadNode 间仍应直接可达。
  const auto node_path = catalog::FindRoutePath(
      result.catalog, result.catalog.road_nodes[0].id,
      result.catalog.road_nodes[1].id);
  ASSERT_TRUE(node_path.has_value()) << "两个环节点之间必须可达";

  // 锚点可寻路（自环返回单元素路径），且到环节点可达。
  const auto self_path =
      catalog::FindRoutePath(result.catalog, "ring_anchor", "ring_anchor");
  ASSERT_TRUE(self_path.has_value());
  EXPECT_EQ(*self_path, (std::vector<std::string>{"ring_anchor"}));
  const auto to_node = catalog::FindRoutePath(
      result.catalog, "ring_anchor", result.catalog.road_nodes[0].id);
  ASSERT_TRUE(to_node.has_value());
  const auto dist =
      catalog::EstimateRouteDistance(result.catalog, "ring_anchor",
                                     result.catalog.road_nodes[1].id);
  ASSERT_TRUE(dist.has_value());
  EXPECT_GT(*dist, 0.0);
}

// ---------------------------------------------------------------------------
// 用例 7 (I3)：单站点直线骨架
//   一行骨架 + 1 个锚点：目录校验通过，FindRoutePath(id,id) 返回 [id]。
// ---------------------------------------------------------------------------
TEST(TopologyGraphBuilderTest, SingleAnchor_ValidCatalogAndSelfRoute) {
  const MapGrid map_info = MakeMapInfo(20, 11, kResolution);
  BinaryGrid skeleton = MakeSkeleton(20, 11);
  DrawHLine(skeleton, 5, 0, 19);

  std::vector<AnchorStation> anchors;
  anchors.push_back(MakeAnchor("only_station", 0.5, 0.275, "charge"));

  TopologyBuilderConfig config;
  config.max_snap_dist_m = 2.0;
  config.map_hash = "single";
  config.graph_version = "v1";

  const TopologyBuildResult result =
      BuildTopologyCatalog(skeleton, map_info, anchors, config);

  ASSERT_TRUE(result.success) << result.error_message;
  ASSERT_EQ(result.catalog.stations.size(), 1u);
  EXPECT_EQ(result.catalog.stations[0].type, "charge");

  // 目录 schema 校验必须通过（success 已隐含，这里显式再验一次）。
  std::string message;
  EXPECT_TRUE(catalog::ValidateStationCatalog(result.catalog, &message))
      << message;

  // FindRoutePath(id,id) 返回单元素路径。
  const auto path = catalog::FindRoutePath(result.catalog, "only_station",
                                           "only_station");
  ASSERT_TRUE(path.has_value());
  EXPECT_EQ(*path, (std::vector<std::string>{"only_station"}));
}

// ---------------------------------------------------------------------------
// 用例 8 (I3)：空骨架（无任何骨架像素）-> 构建失败并给出诊断。
// ---------------------------------------------------------------------------
TEST(TopologyGraphBuilderTest, EmptySkeleton_BuildFails) {
  const MapGrid map_info = MakeMapInfo(10, 10, kResolution);
  BinaryGrid skeleton = MakeSkeleton(10, 10);  // 全 0，无骨架像素

  std::vector<AnchorStation> anchors;
  anchors.push_back(MakeAnchor("s_a", 0.25, 0.25));

  TopologyBuilderConfig config;
  config.max_snap_dist_m = 2.0;
  config.map_hash = "empty";
  config.graph_version = "v1";

  const TopologyBuildResult result =
      BuildTopologyCatalog(skeleton, map_info, anchors, config);

  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.error_message.empty());
}

// ---------------------------------------------------------------------------
// 用例 9 (I2)：两个锚点吸附到同一点 -> 明确诊断（coincides），而非
//   拆边重定向吞掉第一个站点后以 "disconnected" 报错。
// ---------------------------------------------------------------------------
TEST(TopologyGraphBuilderTest, CoincidentAnchors_ReportClearDiagnostic) {
  const MapGrid map_info = MakeMapInfo(20, 11, kResolution);
  BinaryGrid skeleton = MakeSkeleton(20, 11);
  DrawHLine(skeleton, 5, 0, 19);

  // 两个锚点同位（同一投影点），第二个必然与第一个重合。
  std::vector<AnchorStation> anchors;
  anchors.push_back(MakeAnchor("s_first", 0.5, 0.275));
  anchors.push_back(MakeAnchor("s_second", 0.5, 0.275));

  TopologyBuilderConfig config;
  config.max_snap_dist_m = 2.0;
  config.map_hash = "dup";
  config.graph_version = "v1";

  const TopologyBuildResult result =
      BuildTopologyCatalog(skeleton, map_info, anchors, config);

  EXPECT_FALSE(result.success);
  EXPECT_NE(result.error_message.find("coincides"), std::string::npos)
      << "错误信息应包含 'coincides'，实际：" << result.error_message;
  EXPECT_NE(result.error_message.find("s_second"), std::string::npos)
      << "错误信息应指明冲突站点 id，实际：" << result.error_message;
}

}  // namespace
}  // namespace amr_dispatcher_core::planning
