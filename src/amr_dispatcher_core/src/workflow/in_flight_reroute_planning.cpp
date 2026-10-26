#include "amr_dispatcher_core/workflow/in_flight_reroute_planning.hpp"

#include <algorithm>
#include <unordered_set>

namespace amr_dispatcher_core::workflow {
namespace {

std::string RouteEdgeId(const std::string& a, const std::string& b) {
  return a <= b ? "route_edge:" + a + "__" + b : "route_edge:" + b + "__" + a;
}

std::string RouteNodeId(const std::string& node) {
  return "route_node:" + node;
}

}  // namespace

InFlightRerouteDecision PlanInFlightReroute(const InFlightRerouteRequest& request) {
  InFlightRerouteDecision decision;

  if (request.catalog == nullptr) {
    decision.outcome = InFlightRerouteDecision::Outcome::kNoAlternative;
    decision.message = "catalog is null";
    return decision;
  }
  if (request.current_safe_node.empty()) {
    decision.outcome = InFlightRerouteDecision::Outcome::kInvalidStart;
    decision.message = "current_safe_node is empty";
    return decision;
  }
  if (request.destination_node.empty()) {
    decision.outcome = InFlightRerouteDecision::Outcome::kNoAlternative;
    decision.message = "destination_node is empty";
    return decision;
  }
  if (request.current_safe_node == request.destination_node) {
    decision.outcome = InFlightRerouteDecision::Outcome::kAlreadyAtGoal;
    decision.new_route_path = {request.current_safe_node};
    decision.full_updated_path = request.full_original_path;
    decision.message = "vehicle is already at destination";
    return decision;
  }

  // 1. 调用派发同源的绕行入口计算从当前安全节点到目的地的替代路径
  const auto alternative_opt = catalog::FindRoutePathAvoiding(
      *request.catalog, request.current_safe_node, request.destination_node,
      request.blocked_edge_keys);

  if (!alternative_opt.has_value() || alternative_opt->empty()) {
    decision.outcome = InFlightRerouteDecision::Outcome::kNoAlternative;
    decision.message = "no alternative topological path avoiding blocked edges";
    return decision;
  }

  decision.new_route_path = *alternative_opt;

  // 2. 构建 full_updated_path：拼接原路径中当前节点之前的已到达节点
  auto current_pos_it = std::find(request.full_original_path.begin(),
                                  request.full_original_path.end(),
                                  request.current_safe_node);
  if (current_pos_it != request.full_original_path.end()) {
    decision.full_updated_path.assign(request.full_original_path.begin(), current_pos_it);
  }
  decision.full_updated_path.insert(decision.full_updated_path.end(),
                                    decision.new_route_path.begin(),
                                    decision.new_route_path.end());

  // 3. 计算新路线所需的全部路权锁集合
  std::unordered_set<std::string> new_locks_set;
  for (std::size_t i = 0; i < decision.new_route_path.size(); ++i) {
    // 节点锁（包括道口或站点）
    new_locks_set.insert(RouteNodeId(decision.new_route_path[i]));
    if (i + 1 < decision.new_route_path.size()) {
      new_locks_set.insert(RouteEdgeId(decision.new_route_path[i], decision.new_route_path[i + 1]));
    }
  }

  // 4. 计算置换集合：
  // new_required_locks = 新路径所需锁 - 当前已经持有的锁（避免重复申请自己已持有的锁）
  std::unordered_set<std::string> currently_held(request.current_held_locks.begin(),
                                                 request.current_held_locks.end());

  for (const auto& lock_id : new_locks_set) {
    if (currently_held.find(lock_id) == currently_held.end()) {
      decision.new_required_locks.push_back(lock_id);
    }
  }

  // 5. 计算 obsolete_locks（待释放的旧未来锁）：
  // 规则：当前持有的锁中，既不在新路径锁集合中，也不是当前安全停靠节点锁的，可以释放。
  // 【铁律】：当前停靠节点锁必须保留！
  const std::string current_node_lock = RouteNodeId(request.current_safe_node);

  for (const auto& held_lock : request.current_held_locks) {
    if (held_lock == current_node_lock) {
      continue;  // 保护当前物理停靠点
    }
    if (new_locks_set.find(held_lock) == new_locks_set.end()) {
      decision.obsolete_locks.push_back(held_lock);
    }
  }

  std::sort(decision.new_required_locks.begin(), decision.new_required_locks.end());
  std::sort(decision.obsolete_locks.begin(), decision.obsolete_locks.end());

  decision.outcome = InFlightRerouteDecision::Outcome::kRerouteReady;
  decision.message = "reroute path calculated with " + std::to_string(decision.new_route_path.size()) +
                     " nodes, " + std::to_string(decision.new_required_locks.size()) +
                     " new locks to reserve";
  return decision;
}

}  // namespace amr_dispatcher_core::workflow
