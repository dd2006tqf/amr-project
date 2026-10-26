// 交通冲突与死锁分级恢复协调器（纯函数）。
//
// 对应 Phase 7 (P2)：
// 当 DeadlockDetector 识别出 WFG 互斥死锁环时，本模块对死锁环进行策略分析与破环裁决：
// 1. 优先选择环中具有可行替代路线的任务进行拓扑绕行（BreakCycleByReroute）。
// 2. 若涉及多台车，优先重规划优先级较低或绕行代价较小的车辆。
// 3. 若所有涉案车辆均无替代路线（如窄道互卡死胡同），则裁定转入受控安全等待/挂起（kRecoveryFailed），
//    绝不盲动或随意倒车。
#pragma once

#include <string>
#include <vector>

#include "amr_dispatcher_core/catalog/station_catalog.hpp"
#include "amr_dispatcher_core/dispatcher/deadlock_detector.hpp"
#include "amr_dispatcher_core/workflow/in_flight_reroute_planning.hpp"

namespace amr_dispatcher_core::workflow {

struct DeadlockRecoveryCandidate {
  std::string mission_id;
  std::string vehicle_id;
  int priority = 0;
  std::string current_safe_node;
  std::string destination_node;
  std::vector<std::string> current_held_locks;
  std::vector<std::string> full_original_path;
};

struct DeadlockRecoveryRequest {
  std::vector<std::string> cycle_mission_ids;       // 死锁环中包含的任务列表
  std::unordered_set<std::string> cycle_blocked_edges;// 死锁环中发生冲突的边
  std::vector<DeadlockRecoveryCandidate> candidates; // 涉案任务详细状态
  const catalog::StationCatalog* catalog = nullptr;
};

struct DeadlockRecoveryDecision {
  enum class Action {
    kRerouteMission,    // 选定其中某任务执行安全绕行以破环
    kSafeStopSuspend,   // 无法绕行，下发驻车并在道口等待
    kNoActionNeeded,    // 无死锁环或参数无效
  };

  Action action = Action::kNoActionNeeded;
  std::string selected_mission_id;
  InFlightRerouteDecision reroute_decision;
  std::string message;
};

DeadlockRecoveryDecision PlanDeadlockRecovery(const DeadlockRecoveryRequest& request);

}  // namespace amr_dispatcher_core::workflow
