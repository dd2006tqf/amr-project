#include "amr_dispatcher_core/workflow/topology_change_impact.hpp"

#include <algorithm>
#include <unordered_map>
#include <unordered_set>

#include "amr_dispatcher_core/workflow/in_flight_reroute_planning.hpp"

namespace amr_dispatcher_core::workflow {
namespace {

std::string EdgeKey(const std::string& a, const std::string& b) {
  return a <= b ? "route_edge:" + a + "__" + b : "route_edge:" + b + "__" + a;
}

// 目录里是否存在一条可通行（enabled 且方向许可）的 a->b 边
bool HasTraversableEdge(const catalog::StationCatalog& catalog, const std::string& a,
                        const std::string& b) {
  for (const auto& edge : catalog.edges) {
    if (!edge.enabled) {
      continue;
    }
    if (edge.from == a && edge.to == b) {
      return true;
    }
    if (edge.bidirectional && edge.from == b && edge.to == a) {
      return true;
    }
  }
  return false;
}

}  // namespace

bool RemainingPathTraversable(const catalog::StationCatalog& catalog,
                             const std::vector<std::string>& full_path,
                             const std::string& from_node,
                             std::vector<std::string>* blocked_edges) {
  auto start = std::find(full_path.begin(), full_path.end(), from_node);
  if (start == full_path.end()) {
    // 起点不在原路径里（数据不一致）：无法判断"剩下哪段"，保守判为不可通行
    if (blocked_edges != nullptr) {
      blocked_edges->push_back("unknown_start:" + from_node);
    }
    return false;
  }
  bool traversable = true;
  for (auto it = start; std::next(it) != full_path.end(); ++it) {
    const std::string& a = *it;
    const std::string& b = *std::next(it);
    if (!HasTraversableEdge(catalog, a, b)) {
      traversable = false;
      if (blocked_edges != nullptr) {
        blocked_edges->push_back(EdgeKey(a, b));
      }
    }
  }
  return traversable;
}

TopologyChangeImpact AnalyzeTopologyChangeImpact(const TopologyChangeImpactRequest& request) {
  TopologyChangeImpact impact;

  if (request.new_catalog == nullptr) {
    impact.safe_to_apply = false;
    impact.error_code = "internal_error";
    impact.message = "new_catalog is null";
    return impact;
  }
  if (request.missions.empty()) {
    impact.message = "no in-flight missions";
    return impact;
  }

  for (const auto& mission : request.missions) {
    VehicleImpact result;
    result.mission_id = mission.mission_id;
    result.vehicle_id = mission.vehicle_id;

    // 1. 已到达终点
    if (!mission.destination_node.empty() &&
        mission.current_safe_node == mission.destination_node) {
      result.kind = VehicleImpact::Kind::kAlreadyAtGoal;
      result.detail = "vehicle is already at its destination";
      impact.vehicles.push_back(std::move(result));
      continue;
    }

    // 2. 剩余航段仍可通行 -> 不受影响（不产生锁置换，这是最常见的路径）
    std::vector<std::string> blocked_remaining;
    if (RemainingPathTraversable(*request.new_catalog, mission.full_original_path,
                                 mission.current_safe_node, &blocked_remaining)) {
      result.kind = VehicleImpact::Kind::kUnaffected;
      result.detail = "remaining route still traversable";
      impact.vehicles.push_back(std::move(result));
      continue;
    }
    result.blocked_remaining_edges = blocked_remaining;

    // 3. 车可能正压在被封的边上：离开当前安全节点的那一段被封 = 它已经在上面
    //    （lastNodeId 只报告最后确认到达的节点）。让它在半路失去路权在物理上不安全。
    bool on_blocked_edge = false;
    for (const auto& edge_key : blocked_remaining) {
      if (edge_key.rfind("route_edge:", 0) != 0) {
        continue;
      }
      // edge_key 形如 route_edge:<lo>__<hi>：拆两端，看是否一端就是当前安全节点
      const std::string body = edge_key.substr(std::string("route_edge:").size());
      const auto sep = body.find("__");
      if (sep == std::string::npos) {
        continue;
      }
      const std::string a = body.substr(0, sep);
      const std::string b = body.substr(sep + 2);
      if (a == mission.current_safe_node || b == mission.current_safe_node) {
        on_blocked_edge = true;
        break;
      }
    }
    if (on_blocked_edge) {
      if (!request.allow_edge_under_vehicle) {
        result.kind = VehicleImpact::Kind::kOnBlockedEdge;
        result.detail = "vehicle may be traversing a blocked edge from " +
                        mission.current_safe_node;
        ++impact.occupied_count;
        impact.vehicles.push_back(std::move(result));
        continue;
      }
      // 操作员已确认该车会安全驶离：变更对**新**规划立即生效，
      // 该边在预约层的锁保留给在途车（提交时跳过 Block），等它自己释放。
      result.kind = VehicleImpact::Kind::kUnaffected;
      result.detail =
          "vehicle is on a blocked edge; change applies to new planning only "
          "(edge lock retained by the in-flight vehicle)";
      impact.vehicles.push_back(std::move(result));
      continue;
    }

    // 4. 需要绕行：复用既有两阶段重规划（新锁 / 废弃锁 / 全量更新路径）
    InFlightRerouteRequest reroute_request;
    reroute_request.mission_id = mission.mission_id;
    reroute_request.vehicle_id = mission.vehicle_id;
    reroute_request.current_safe_node = mission.current_safe_node;
    reroute_request.destination_node = mission.destination_node;
    reroute_request.current_held_locks = mission.current_held_locks;
    reroute_request.full_original_path = mission.full_original_path;
    // 恒为空：限制已编码进 new_catalog；传无序禁用键会把单向边的合法方向一起封死
    reroute_request.blocked_edge_keys = {};
    reroute_request.catalog = request.new_catalog;

    const auto decision = PlanInFlightReroute(reroute_request);
    switch (decision.outcome) {
      case InFlightRerouteDecision::Outcome::kRerouteReady:
        result.kind = VehicleImpact::Kind::kRerouteReady;
        result.new_required_locks = decision.new_required_locks;
        result.obsolete_locks = decision.obsolete_locks;
        result.full_updated_path = decision.full_updated_path;
        result.detail = "reroute available from " + mission.current_safe_node;
        ++impact.reroute_count;
        break;
      case InFlightRerouteDecision::Outcome::kAlreadyAtGoal:
        result.kind = VehicleImpact::Kind::kAlreadyAtGoal;
        result.detail = decision.message;
        break;
      case InFlightRerouteDecision::Outcome::kInvalidStart:
      case InFlightRerouteDecision::Outcome::kNoAlternative:
      default:
        result.kind = VehicleImpact::Kind::kNoAlternative;
        result.detail = decision.message;
        ++impact.stranded_count;
        break;
    }
    impact.vehicles.push_back(std::move(result));
  }

  // 汇总：**任一车不可行就整体拒绝**（不做部分应用 —— 部分应用正是"有些车回不了家"的成因）
  if (impact.occupied_count > 0) {
    impact.safe_to_apply = false;
    impact.error_code = "edge_occupied_in_flight";
    impact.message = std::to_string(impact.occupied_count) +
                     " vehicle(s) are traversing an edge being closed";
  } else if (impact.stranded_count > 0) {
    impact.safe_to_apply = false;
    impact.error_code = "would_strand_vehicles";
    impact.message =
        std::to_string(impact.stranded_count) + " in-flight mission(s) would have no route";
  } else {
    impact.message = impact.reroute_count > 0
                         ? std::to_string(impact.reroute_count) + " mission(s) will be rerouted"
                         : "no in-flight mission affected";
  }
  return impact;
}

}  // namespace amr_dispatcher_core::workflow
