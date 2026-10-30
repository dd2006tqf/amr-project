// 拓扑变更的在途影响分析（纯函数）。
//
// 场景：操作员要封一条边。系统必须在**改任何东西之前**回答一个问题：
// "现在有车在这条边上 / 要靠它回家吗？"
//
// 逐车判定（顺序即优先级）
// ------------------------
// 1. 已在终点            -> kAlreadyAtGoal，无事可做
// 2. 剩余航段仍可通行    -> kUnaffected，**不产生任何锁置换**（绝大多数情况）
// 3. 车可能正压在被封边上 -> kOnBlockedEdge（安全拒绝）
//    `lastNodeId` 只说明"最后确认到达的节点"，车可能已经在去下一段的路上。
//    此时那条边物理上必须让它走完 —— 我们不会提交一个让车失去立足点的变更。
// 4. 需要绕行            -> 复用既有 PlanInFlightReroute 算新锁/旧锁（两阶段置换）
//
// 关键设计：传给 PlanInFlightReroute 的 blocked_edge_keys **恒为空**。
// 限制已经编码进 new_catalog（enabled=false / bidirectional=false），再传禁用键
// 会把单向边两个方向一起封死（那是无序键，表达不了方向）。
#pragma once

#include <string>
#include <vector>

#include "amr_dispatcher_core/catalog/station_catalog.hpp"

namespace amr_dispatcher_core::workflow {

struct InFlightMissionView {
  std::string mission_id;
  std::string vehicle_id;
  std::string current_safe_node;  // 最后确认到达/停靠的节点（重规划起点）
  std::string destination_node;
  std::vector<std::string> full_original_path;
  std::vector<std::string> current_held_locks;
};

struct VehicleImpact {
  enum class Kind {
    kUnaffected,   // 剩余航段不受影响，无需动作
    kRerouteReady, // 需要绕行，且已算出两阶段锁置换集合
    kOnBlockedEdge,// 车可能正压在被禁用的边上：物理安全上不能提交
    kNoAlternative,// 拓扑上无替代路线：提交会让它成为孤岛
    kAlreadyAtGoal // 已到达终点
  };

  Kind kind = Kind::kUnaffected;
  std::string mission_id;
  std::string vehicle_id;
  std::vector<std::string> new_required_locks;
  std::vector<std::string> obsolete_locks;
  std::vector<std::string> full_updated_path;
  std::vector<std::string> blocked_remaining_edges;
  std::string detail;
};

struct TopologyChangeImpact {
  std::vector<VehicleImpact> vehicles;
  bool safe_to_apply = true;
  std::size_t reroute_count = 0;
  std::size_t stranded_count = 0;
  std::size_t occupied_count = 0;
  // ok | edge_occupied_in_flight | would_strand_vehicles
  std::string error_code = "ok";
  std::string message;
};

struct TopologyChangeImpactRequest {
  const catalog::StationCatalog* new_catalog = nullptr;  // 已应用叠加层
  std::vector<InFlightMissionView> missions;
  // 操作员已确认"压在边上的车会安全驶离"时置 true：此时该类车降级为 kUnaffected
  // 之外的可提交状态（提交时该边的资源锁保留给在途车，等它自己释放）。
  bool allow_edge_under_vehicle = false;
};

// 剩余航段（从 from_node 起到末尾）在新目录下是否仍可通行；
// 不可通行时把阻塞的边写进 blocked_edges（无序键，仅供报告）。
bool RemainingPathTraversable(const catalog::StationCatalog& catalog,
                             const std::vector<std::string>& full_path,
                             const std::string& from_node,
                             std::vector<std::string>* blocked_edges);

TopologyChangeImpact AnalyzeTopologyChangeImpact(const TopologyChangeImpactRequest& request);

}  // namespace amr_dispatcher_core::workflow
