#ifndef AMR_DISPATCHER_CORE__PLANNING__TOPOLOGY_GRAPH_BUILDER_HPP_
#define AMR_DISPATCHER_CORE__PLANNING__TOPOLOGY_GRAPH_BUILDER_HPP_

#include <string>
#include <vector>

#include "amr_dispatcher_core/catalog/station_catalog.hpp"
#include "amr_dispatcher_core/planning/map_geometry_utils.hpp"
#include "amr_dispatcher_core/planning/safety_space_filter.hpp"

namespace amr_dispatcher_core::planning {

// 业务站点锚点（来自 markers.yaml）：世界坐标 + 语义类型。
struct AnchorStation {
  std::string id;
  std::string type = "normal";  // "pickup", "dropoff", "charge", etc.
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

struct TopologyBuilderConfig {
  // 站点距离最近路网边的最大吸附允许距离（米），超过则构建失败。
  double max_snap_dist_m = 2.0;
  std::string map_hash;
  std::string graph_version = "v1";
};

struct TopologyBuildResult {
  bool success = false;
  std::string error_message;
  catalog::StationCatalog catalog;
};

// 拓扑路网构建与站点吸附边拆分。
//
// 算法流水线：
//   1. 骨架特征点提取：识别 8 邻域环连通组数 != 2 的骨架像素
//      （端点=1、分叉点>=3、孤立点=0），每个特征点生成 RoadNode。
//      纯闭合环（无特征点）任取一环上像素作为 RoadNode，避免环丢失。
//   2. 骨架链追踪：从特征点沿骨架追踪到下一特征点，生成 CatalogStationEdge，
//      polyline 保存真实折线像素序列（世界坐标），distance_m 为累加长度。
//   3. 站点吸附：将每个 AnchorStation 正交投影到最近边的 polyline，
//      距离超过 max_snap_dist_m 时报错；投影点处将原边拆分为两条子边，
//      站点作为新节点接入路网。
//   4. 连通性校验：所有站点必须处于同一连通分量（通过 FindRoutePath 验证）。
//   5. 版本标记：写入 map_hash / graph_version，最后经 ValidateStationCatalog 校验。
TopologyBuildResult BuildTopologyCatalog(
    const BinaryGrid& skeleton, const MapGrid& map_info,
    const std::vector<AnchorStation>& anchors,
    const TopologyBuilderConfig& config);

}  // namespace amr_dispatcher_core::planning

#endif  // AMR_DISPATCHER_CORE__PLANNING__TOPOLOGY_GRAPH_BUILDER_HPP_
