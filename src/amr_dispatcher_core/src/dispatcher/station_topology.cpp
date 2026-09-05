#include "amr_dispatcher_core/dispatcher/station_topology.hpp"

#include <algorithm>
#include <limits>
#include <queue>

namespace amr_dispatcher_core::dispatcher {

void StationTopologyGraph::AddStation(StationNode station) {
  std::string id = station.id;
  stations_[id] = std::move(station);
}

void StationTopologyGraph::AddEdge(StationEdge edge) {
  double dist = edge.distance_m;
  if (dist <= 0.0) {
    auto from_it = stations_.find(edge.from);
    auto to_it = stations_.find(edge.to);
    if (from_it != stations_.end() && to_it != stations_.end()) {
      double dx = from_it->second.x - to_it->second.x;
      double dy = from_it->second.y - to_it->second.y;
      dist = std::sqrt(dx * dx + dy * dy);
    }
  }
  adj_[edge.from].emplace_back(edge.to, dist);
  if (edge.bidirectional) {
    adj_[edge.to].emplace_back(edge.from, dist);
  }
}

const StationNode* StationTopologyGraph::FindStation(const std::string& id) const {
  auto it = stations_.find(id);
  if (it == stations_.end()) return nullptr;
  return &it->second;
}

std::vector<std::string> StationTopologyGraph::ListStationIds() const {
  std::vector<std::string> ids;
  ids.reserve(stations_.size());
  for (const auto& [id, _] : stations_) {
    ids.push_back(id);
  }
  return ids;
}

std::optional<std::vector<std::string>> StationTopologyGraph::FindShortestPath(
    const std::string& from, const std::string& to) const {
  if (stations_.find(from) == stations_.end() || stations_.find(to) == stations_.end()) {
    return std::nullopt;
  }
  if (from == to) {
    return std::vector<std::string>{from};
  }

  using NodeDist = std::pair<double, std::string>;
  std::priority_queue<NodeDist, std::vector<NodeDist>, std::greater<NodeDist>> pq;
  std::unordered_map<std::string, double> min_dist;
  std::unordered_map<std::string, std::string> parent;

  min_dist[from] = 0.0;
  pq.push({0.0, from});

  while (!pq.empty()) {
    auto [d, u] = pq.top();
    pq.pop();

    if (u == to) break;
    if (d > min_dist[u]) continue;

    auto it = adj_.find(u);
    if (it == adj_.end()) continue;

    for (const auto& [v, weight] : it->second) {
      double new_dist = d + weight;
      if (min_dist.find(v) == min_dist.end() || new_dist < min_dist[v]) {
        min_dist[v] = new_dist;
        parent[v] = u;
        pq.push({new_dist, v});
      }
    }
  }

  if (min_dist.find(to) == min_dist.end()) {
    return std::nullopt;
  }

  std::vector<std::string> path;
  std::string curr = to;
  while (curr != from) {
    path.push_back(curr);
    curr = parent[curr];
  }
  path.push_back(from);
  std::reverse(path.begin(), path.end());
  return path;
}

std::optional<double> StationTopologyGraph::EstimatePathDistance(
    const std::string& from, const std::string& to) const {
  auto path = FindShortestPath(from, to);
  if (!path) return std::nullopt;

  double total = 0.0;
  for (std::size_t i = 0; i + 1 < path->size(); ++i) {
    const auto& u = (*path)[i];
    const auto& v = (*path)[i + 1];
    auto it = adj_.find(u);
    if (it != adj_.end()) {
      for (const auto& [next, w] : it->second) {
        if (next == v) {
          total += w;
          break;
        }
      }
    }
  }
  return total;
}

void StationTopologyGraph::Clear() {
  stations_.clear();
  adj_.clear();
}

}  // namespace amr_dispatcher_core::dispatcher
