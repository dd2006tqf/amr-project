#include "amr_dispatcher_core/planning/skeleton_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

namespace amr_dispatcher_core::planning {
namespace {

constexpr double kJoinEps = 1e-9;  // 接缝去重 / 零长度判定阈值（米）

using Polyline = std::vector<std::pair<double, double>>;

// 在 stations + road_nodes 混合节点集合中按 id 查找坐标。
std::optional<std::pair<double, double>> FindNodeCoord(
    const catalog::StationCatalog& catalog, const std::string& id) {
  for (const auto& station : catalog.stations) {
    if (station.id == id) {
      return std::pair<double, double>{station.x, station.y};
    }
  }
  for (const auto& road_node : catalog.road_nodes) {
    if (road_node.id == id) {
      return std::pair<double, double>{road_node.x, road_node.y};
    }
  }
  return std::nullopt;
}

// 追加折线点并去除相邻边接缝处的重复点（零长度段会破坏等距插值）。
void AppendPoint(Polyline& polyline, double x, double y) {
  if (!polyline.empty() &&
      std::hypot(polyline.back().first - x, polyline.back().second - y) <
          kJoinEps) {
    return;
  }
  polyline.emplace_back(x, y);
}

// 沿折线按弧长 d 插值位姿（d ∈ [0, total]），航向取所在段方向。
Pose2D InterpolateAt(const Polyline& polyline,
                     const std::vector<double>& cumulative, double d,
                     bool final_point,
                     const std::optional<double>& final_theta) {
  const double total = cumulative.back();
  d = std::clamp(d, 0.0, total);

  // 定位所在段：最后一个满足 cumulative[i] <= d 的段 [i, i+1]。
  std::size_t index = 0;
  for (std::size_t i = 0; i + 1 < cumulative.size(); ++i) {
    if (cumulative[i] <= d + kJoinEps) {
      index = i;
    } else {
      break;
    }
  }
  const auto& a = polyline[index];
  const auto& b = polyline[index + 1];
  const double seg_len = cumulative[index + 1] - cumulative[index];
  const double ratio = seg_len > kJoinEps ? (d - cumulative[index]) / seg_len : 0.0;

  Pose2D pose;
  pose.x = a.first + ratio * (b.first - a.first);
  pose.y = a.second + ratio * (b.second - a.second);
  pose.theta = std::atan2(b.second - a.second, b.first - a.first);
  if (final_point && final_theta.has_value()) {
    pose.theta = *final_theta;
  }
  return pose;
}

// 等距插值重采样：总长 L、步长 step，取 num_steps = ceil(L / step) 段，
// 采样间距 L / num_steps ≤ step（与 MockLinearPlanner 一致），
// 首路点严格为折线起点、末路点严格为折线终点。
std::vector<Pose2D> ResamplePolyline(
    const Polyline& polyline, double step_size,
    const std::optional<double>& first_theta = std::nullopt,
    const std::optional<double>& final_theta = std::nullopt) {
  std::vector<Pose2D> path;
  if (polyline.empty()) {
    return path;
  }

  std::vector<double> cumulative(polyline.size(), 0.0);
  for (std::size_t i = 1; i < polyline.size(); ++i) {
    cumulative[i] =
        cumulative[i - 1] +
        std::hypot(polyline[i].first - polyline[i - 1].first,
                   polyline[i].second - polyline[i - 1].second);
  }
  const double total = cumulative.back();

  if (total <= kJoinEps || polyline.size() == 1) {
    Pose2D pose;
    pose.x = polyline.front().first;
    pose.y = polyline.front().second;
    pose.theta = first_theta.value_or(final_theta.value_or(0.0));
    path.push_back(pose);
    return path;
  }

  const int num_steps =
      std::max(1, static_cast<int>(std::ceil(total / step_size)));
  path.reserve(static_cast<std::size_t>(num_steps) + 1);
  for (int i = 0; i <= num_steps; ++i) {
    const double d = total * static_cast<double>(i) / num_steps;
    const bool first = (i == 0);
    const bool last = (i == num_steps);
    Pose2D pose = InterpolateAt(polyline, cumulative, d, last, final_theta);
    if (first && first_theta.has_value()) {
      pose.theta = *first_theta;
    }
    path.push_back(pose);
  }
  return path;
}

// 在混合节点集合中查找离 (x, y) 最近的节点 id 与其坐标。
struct NearestNode {
  std::string id;
  double x = 0.0;
  double y = 0.0;
};

std::optional<NearestNode> FindNearestNode(const catalog::StationCatalog& catalog,
                                           double x, double y) {
  std::optional<NearestNode> best;
  double best_dist = std::numeric_limits<double>::infinity();
  const auto consider = [&](const std::string& id, double nx, double ny) {
    const double dist = std::hypot(nx - x, ny - y);
    if (dist < best_dist) {
      best_dist = dist;
      best = NearestNode{id, nx, ny};
    }
  };
  for (const auto& station : catalog.stations) {
    consider(station.id, station.x, station.y);
  }
  for (const auto& road_node : catalog.road_nodes) {
    consider(road_node.id, road_node.x, road_node.y);
  }
  return best;
}

// 查找路径相邻节点对 (from, to) 对应的边折线：
// 正向直接使用 edge.polyline；反向（双向边）使用其反转版本；
// 边缺失或折线不足 2 点时退化为两节点坐标间的直线。
Polyline EdgePolylineFor(const catalog::StationCatalog& catalog,
                         const std::string& from, const std::string& to,
                         const std::pair<double, double>& from_coord,
                         const std::pair<double, double>& to_coord) {
  const catalog::CatalogStationEdge* matched = nullptr;
  bool reverse = false;
  for (const auto& edge : catalog.edges) {
    if (!edge.enabled) {
      continue;
    }
    if (edge.from == from && edge.to == to) {
      matched = &edge;
      reverse = false;
      break;
    }
    if (edge.bidirectional && edge.to == from && edge.from == to) {
      matched = &edge;
      reverse = true;
      break;
    }
  }

  Polyline segment;
  if (matched != nullptr && matched->polyline.size() >= 2) {
    segment.reserve(matched->polyline.size());
    if (reverse) {
      for (auto it = matched->polyline.rbegin(); it != matched->polyline.rend();
           ++it) {
        segment.push_back(*it);
      }
    } else {
      segment = matched->polyline;
    }
  } else {
    // 退化：无匹配边 / 无有效折线 -> 两点直线
    segment.push_back(from_coord);
    segment.push_back(to_coord);
  }
  return segment;
}

}  // namespace

SkeletonPlanner::SkeletonPlanner(catalog::StationCatalog catalog, Config config)
    : catalog_(std::move(catalog)), config_(config) {
  if (config_.step_size <= 0.0) {
    config_.step_size = 0.1;
  }
}

void SkeletonPlanner::UpdateCatalog(catalog::StationCatalog catalog) {
  catalog_ = std::move(catalog);
}

std::vector<Pose2D> SkeletonPlanner::PlanBetweenNodes(const std::string& from_node,
                                                      const std::string& to_node) {
  const auto route = catalog::FindRoutePath(catalog_, from_node, to_node);
  if (!route.has_value() || route->empty()) {
    return {};
  }

  if (route->size() == 1) {
    const auto coord = FindNodeCoord(catalog_, from_node);
    if (!coord.has_value()) {
      return {};
    }
    Pose2D pose;
    pose.x = coord->first;
    pose.y = coord->second;
    pose.theta = 0.0;
    return {pose};
  }

  Polyline polyline;
  for (std::size_t i = 0; i + 1 < route->size(); ++i) {
    const auto from_coord = FindNodeCoord(catalog_, (*route)[i]);
    const auto to_coord = FindNodeCoord(catalog_, (*route)[i + 1]);
    if (!from_coord.has_value() || !to_coord.has_value()) {
      return {};  // 路径含未知节点（目录异常）：规划失败
    }
    const Polyline segment =
        EdgePolylineFor(catalog_, (*route)[i], (*route)[i + 1], *from_coord,
                        *to_coord);
    for (const auto& point : segment) {
      AppendPoint(polyline, point.first, point.second);
    }
  }

  return ResamplePolyline(polyline, config_.step_size);
}

std::vector<Pose2D> SkeletonPlanner::Plan(const Pose2D& start, const Pose2D& goal,
                                          const std::string& /*edge_id*/) {
  if (std::hypot(goal.x - start.x, goal.y - start.y) <= kJoinEps) {
    return {start};
  }

  const auto start_node = FindNearestNode(catalog_, start.x, start.y);
  const auto goal_node = FindNearestNode(catalog_, goal.x, goal.y);
  if (!start_node.has_value() || !goal_node.has_value()) {
    return {};  // 空目录：无可吸附节点
  }

  const auto route = catalog::FindRoutePath(catalog_, start_node->id, goal_node->id);
  if (!route.has_value() || route->empty()) {
    return {};
  }

  // 拼接总折线：start 接驳段 + 各边折线 + goal 接驳段
  Polyline polyline;
  AppendPoint(polyline, start.x, start.y);
  if (route->size() == 1) {
    // 起终点吸附到同一节点
    AppendPoint(polyline, start_node->x, start_node->y);
  } else {
    for (std::size_t i = 0; i + 1 < route->size(); ++i) {
      const auto from_coord = FindNodeCoord(catalog_, (*route)[i]);
      const auto to_coord = FindNodeCoord(catalog_, (*route)[i + 1]);
      if (!from_coord.has_value() || !to_coord.has_value()) {
        return {};
      }
      const Polyline segment =
          EdgePolylineFor(catalog_, (*route)[i], (*route)[i + 1], *from_coord,
                          *to_coord);
      for (const auto& point : segment) {
        AppendPoint(polyline, point.first, point.second);
      }
    }
  }
  AppendPoint(polyline, goal.x, goal.y);

  // 首路点严格为 start（含其航向），末路点严格为 goal（含其航向）
  return ResamplePolyline(polyline, config_.step_size, start.theta, goal.theta);
}

}  // namespace amr_dispatcher_core::planning
