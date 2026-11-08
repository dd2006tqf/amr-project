#ifndef AMR_DISPATCHER_CORE__DISPATCHER__TOPOLOGY_PROJECTION_HPP_
#define AMR_DISPATCHER_CORE__DISPATCHER__TOPOLOGY_PROJECTION_HPP_

#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace amr_dispatcher_core::dispatcher {

// /dispatcher/topology_state 的纯函数投影结果：
// 字段与 amr_dispatcher_interfaces/msg/TopologyState.msg 逐字段对应
//（active_edges/locked_edges/edge_owners/deadlocked_nodes + deadlock_detected）。
struct TopologyProjection {
  std::vector<std::string> active_edges;      // 任务持有且为 route_edge: 的资源
  std::vector<std::string> locked_edges;      // 预约表全量资源 id（Snapshot 已排序）
  std::vector<std::string> edge_owners;       // 与 locked_edges 逐项对齐的持有者
  std::vector<std::string> deadlocked_nodes;  // 死锁环解析出的站点 id（去重有序）
  bool deadlock_detected = false;             // cycle_path 非空
};

// 由预约表快照 + 活跃任务集合 + 死锁环路径，投影出拓扑状态话题载荷。
// deadlocked_nodes 解析规则（对齐 visualizer 消费——必须是站点 id）：
//   route_edge:A__B -> A、B；route_node:X -> X；其余（mission id 等）跳过。
TopologyProjection BuildTopologyProjection(
    const std::vector<std::pair<std::string, std::string>>& resource_owner_snapshot,
    const std::unordered_set<std::string>& active_mission_ids,
    const std::vector<std::vector<std::string>>& deadlock_cycle_paths);

}  // namespace amr_dispatcher_core::dispatcher

#endif  // AMR_DISPATCHER_CORE__DISPATCHER__TOPOLOGY_PROJECTION_HPP_
