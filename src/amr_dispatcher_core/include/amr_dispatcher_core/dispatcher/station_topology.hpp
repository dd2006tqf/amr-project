#pragma once

#include <cmath>
#include <filesystem>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace amr_dispatcher_core::dispatcher {

struct StationNode {
  std::string id;
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  std::string type = "normal";  // pickup / dropoff / charge / normal
};

struct StationEdge {
  std::string from;
  std::string to;
  double distance_m = 0.0;
  bool bidirectional = true;
};

// 拓扑站点图表与最短路径寻路 (Dijkstra)
class StationTopologyGraph {
 public:
  StationTopologyGraph() = default;

  void AddStation(StationNode station);
  void AddEdge(StationEdge edge);

  const StationNode* FindStation(const std::string& id) const;
  std::vector<std::string> ListStationIds() const;

  // 使用 Dijkstra 算法计算站点间最短拓扑路点序列
  std::optional<std::vector<std::string>> FindShortestPath(
      const std::string& from, const std::string& to) const;

  // 动态避障重寻路：排除一组拥堵/死锁路段后，找一条不经过被封锁边的最短绕行路径
  std::optional<std::vector<std::string>> FindShortestPathAvoiding(
      const std::string& from, const std::string& to,
      const std::unordered_set<std::string>& blocked_edges) const;

  // 估算总欧式/拓扑里程
  std::optional<double> EstimatePathDistance(
      const std::string& from, const std::string& to) const;

  void Clear();

 private:
  std::unordered_map<std::string, StationNode> stations_;
  std::unordered_map<std::string, std::vector<std::pair<std::string, double>>> adj_;
};

}  // namespace amr_dispatcher_core::dispatcher
