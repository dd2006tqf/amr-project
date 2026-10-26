// 在途任务安全重规划决策（纯函数）。
//
// 对应 Phase 7 (P1)：
// 当在途任务遇到通道阻塞、资源冲突或死锁时，系统必须从当前安全停靠锚点重新规划替代路线。
//
// 关键设计与安全铁律：
// 1. 【两阶段资源置换原则】：本纯函数计算出新路线所需的全部新锁（new_required_locks），
//    以及旧路线中未来尚未驶入、可以被安全置换的废弃锁（obsolete_locks）。
//    调用方必须先对 new_required_locks 执行 Reserve 成功后，才允许 Release obsolete_locks。
// 2. 【当前锚点保护】：车辆当前停靠的节点锁（current_safe_node）以及已驶过的历史后方锁，
//    绝对不能被划入 obsolete_locks，防止在重规划生效期间车辆失去物理立足点的保护。
// 3. 【诚实无伪造】：若拓扑不可达，返回 kNoAlternative，绝不伪造直线穿越或跳跃路径。
#pragma once

#include <string>
#include <unordered_set>
#include <vector>

#include "amr_dispatcher_core/catalog/station_catalog.hpp"

namespace amr_dispatcher_core::workflow {

struct InFlightRerouteRequest {
  std::string mission_id;
  std::string vehicle_id;
  std::string current_safe_node;              // 当前所在/已到达的安全节点（重规划起点）
  std::string destination_node;               // 任务最终目的地节点
  std::vector<std::string> current_held_locks;// 当前该任务持有的所有路权锁（来自预约表快照）
  std::vector<std::string> full_original_path;// 原规划的完整节点序列
  std::unordered_set<std::string> blocked_edge_keys; // 触发阻断的边（route_edge:A__B）
  const catalog::StationCatalog* catalog = nullptr;
};

struct InFlightRerouteDecision {
  enum class Outcome {
    kRerouteReady,     // 成功找到替代路线并规划出新旧锁置换集合
    kNoAlternative,    // 拓扑上无可用替代路线
    kInvalidStart,     // 起点安全节点无效或未解析
    kAlreadyAtGoal,    // 已经位于目标点，无需重规划
  };

  Outcome outcome = Outcome::kNoAlternative;
  std::vector<std::string> new_route_path;     // 从当前节点到终点的新节点序列
  std::vector<std::string> full_updated_path;  // 已走过节点 + 新节点序列（全量拼接供 VDA 订单更新）
  std::vector<std::string> new_required_locks; // 新路线所需申请的锁集合
  std::vector<std::string> obsolete_locks;     // 原路线中未来不再需要、待置换释放的旧锁
  std::string message;
};

InFlightRerouteDecision PlanInFlightReroute(const InFlightRerouteRequest& request);

}  // namespace amr_dispatcher_core::workflow
