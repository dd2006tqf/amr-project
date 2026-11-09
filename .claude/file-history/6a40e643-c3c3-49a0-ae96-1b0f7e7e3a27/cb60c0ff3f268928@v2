#include "amr_dispatcher_core/fleet/vda5050_order_builder.hpp"

#include <algorithm>
#include <cmath>

namespace amr_dispatcher_core::fleet {
namespace {

// 在 stations 与 road_nodes 的混合 id 空间中查找节点坐标。
// 未命中返回 nullptr（调用方据此判定非法路径）。
const void* FindNodeCoords(const catalog::StationCatalog& catalog,
                           const std::string& node_id,
                           double* x,
                           double* y,
                           double* yaw) {
  for (const auto& station : catalog.stations) {
    if (station.id == node_id) {
      *x = station.x;
      *y = station.y;
      *yaw = station.yaw;
      return &station;
    }
  }
  for (const auto& road_node : catalog.road_nodes) {
    if (road_node.id == node_id) {
      *x = road_node.x;
      *y = road_node.y;
      *yaw = road_node.yaw;
      return &road_node;
    }
  }
  return nullptr;
}

// 查找连接 start/end 的目录边：支持正向与双向边的反向行驶。
// 返回 nullptr 表示目录中无此边（调用方走直线插值兜底）。
const catalog::CatalogStationEdge* FindCatalogEdge(const catalog::StationCatalog& catalog,
                                                   const std::string& start,
                                                   const std::string& end,
                                                   bool* traversed_backward) {
  for (const auto& edge : catalog.edges) {
    if (edge.from == start && edge.to == end) {
      *traversed_backward = false;
      return &edge;
    }
    if (edge.bidirectional && edge.to == start && edge.from == end) {
      *traversed_backward = true;
      return &edge;
    }
  }
  return nullptr;
}

// 把 (x, y) 折线点转为带航向的轨迹点：每点航向取到下一点方向，末点沿用前段方向。
std::vector<VehicleTrajectoryPoint> BuildTrajectory(
    const std::vector<std::pair<double, double>>& points) {
  std::vector<VehicleTrajectoryPoint> trajectory;
  trajectory.reserve(points.size());
  for (std::size_t i = 0; i < points.size(); ++i) {
    VehicleTrajectoryPoint point;
    point.x = points[i].first;
    point.y = points[i].second;
    if (points.size() == 1) {
      point.yaw = 0.0;
    } else if (i + 1 < points.size()) {
      point.yaw = std::atan2(points[i + 1].second - points[i].second,
                             points[i + 1].first - points[i].first);
    } else {
      point.yaw = trajectory.back().yaw;  // 末点沿用上一段航向
    }
    trajectory.push_back(point);
  }
  return trajectory;
}

// 边缺失或折线为空时的兜底：两端节点直线插值（首尾两点，带航向）。
std::vector<VehicleTrajectoryPoint> BuildStraightLineFallback(double start_x,
                                                             double start_y,
                                                             double end_x,
                                                             double end_y) {
  std::vector<VehicleTrajectoryPoint> trajectory(2);
  const double yaw = std::atan2(end_y - start_y, end_x - start_x);
  trajectory[0].x = start_x;
  trajectory[0].y = start_y;
  trajectory[0].yaw = yaw;
  trajectory[1].x = end_x;
  trajectory[1].y = end_y;
  trajectory[1].yaw = yaw;
  return trajectory;
}

}  // namespace

std::optional<VehicleOrder> BuildVda5050OrderFromPath(
    const std::string& vehicle_id,
    const std::string& mission_id,
    const uint32_t order_update_id,
    const std::vector<std::string>& path_node_ids,
    const catalog::StationCatalog& catalog,
    const std::unordered_set<std::string>& reserved_locks,
    const std::vector<VehicleAction>& terminal_actions,
    const Vda5050OrderBuilderConfig& config) {
  if (path_node_ids.empty()) {
    return std::nullopt;
  }

  const std::size_t node_count = path_node_ids.size();

  // 1. 提取所有节点坐标；任一节点缺失即整单作废（防止小车驶向未定义坐标）。
  std::vector<double> xs(node_count);
  std::vector<double> ys(node_count);
  std::vector<double> yaws(node_count);
  for (std::size_t i = 0; i < node_count; ++i) {
    if (FindNodeCoords(catalog, path_node_ids[i], &xs[i], &ys[i], &yaws[i]) == nullptr) {
      return std::nullopt;
    }
  }

  // 2. 原始放行判定（仅看锁，尚未做协议一致性拉平）。
  const std::size_t edge_count = node_count - 1;
  std::vector<bool> node_raw(node_count, false);
  std::vector<bool> edge_raw(edge_count, false);
  for (std::size_t i = 0; i < node_count; ++i) {
    node_raw[i] = reserved_locks.count(dispatcher::ResourceTable::RouteNodeId(
                      path_node_ids[i])) > 0;
  }
  for (std::size_t i = 0; i < edge_count; ++i) {
    const auto edge_id = dispatcher::ResourceTable::RouteEdgeId(path_node_ids[i],
                                                                path_node_ids[i + 1]);
    // 边锁存在且两端节点均已获路权
    edge_raw[i] = reserved_locks.count(edge_id) > 0 && node_raw[i] && node_raw[i + 1];
  }

  // 3. 协议一致性拉平：按 node0 -> edge0 -> node1 -> edge1 ... 的行驶顺序遍历，
  //    一旦出现 released = false（Horizon 区间），其后所有节点与边一律为 false
  //    （VDA 5050 规定 Horizon 之后不得再出现 Base 区间）。
  std::vector<bool> node_released(node_count, false);
  std::vector<bool> edge_released(edge_count, false);
  bool horizon_started = false;
  for (std::size_t i = 0; i < node_count; ++i) {
    node_released[i] = !horizon_started && node_raw[i];
    horizon_started = horizon_started || !node_released[i];
    if (i + 1 < node_count) {
      edge_released[i] = !horizon_started && edge_raw[i];
      horizon_started = horizon_started || !edge_released[i];
    }
  }

  // 4. 组装订单：sequence_id 取 VDA 5050 规范的偶数序列 0, 2, 4 ...
  VehicleOrder order;
  order.order_id = mission_id;
  order.order_update_id = order_update_id;
  order.vehicle_id = vehicle_id;
  order.nodes.reserve(node_count);
  order.edges.reserve(edge_count);

  const std::size_t last_index = node_count - 1;
  for (std::size_t i = 0; i < node_count; ++i) {
    VehicleNode node;
    node.node_id = path_node_ids[i];
    node.sequence_id = static_cast<uint32_t>(i * 2);
    node.x = xs[i];
    node.y = ys[i];
    node.yaw = yaws[i];
    node.released = node_released[i];
    if (i == last_index) {
      node.actions = terminal_actions;  // 终点动作挂载
    }
    order.nodes.push_back(std::move(node));

    if (i + 1 < node_count) {
      VehicleEdge edge;
      edge.start_node_id = path_node_ids[i];
      edge.end_node_id = path_node_ids[i + 1];
      edge.edge_id = dispatcher::ResourceTable::RouteEdgeId(edge.start_node_id,
                                                            edge.end_node_id);
      edge.released = edge_released[i];
      edge.max_speed = config.default_max_speed_mps;

      bool traversed_backward = false;
      const auto* catalog_edge =
          FindCatalogEdge(catalog, edge.start_node_id, edge.end_node_id, &traversed_backward);
      // 折线必须至少 2 个点才构成合法轨迹段；单点折线视同缺失，走直线兜底，
      // 避免向小车下发只有一个控制点的无效 VDA 5050 轨迹。
      if (catalog_edge != nullptr && catalog_edge->polyline.size() >= 2) {
        auto points = catalog_edge->polyline;
        if (traversed_backward) {
          std::reverse(points.begin(), points.end());  // 反向行驶：折线按行驶方向反转
        }
        edge.polyline = BuildTrajectory(points);
      } else {
        edge.polyline = BuildStraightLineFallback(xs[i], ys[i], xs[i + 1], ys[i + 1]);
      }

      order.edges.push_back(std::move(edge));
    }
  }

  return order;
}

}  // namespace amr_dispatcher_core::fleet
