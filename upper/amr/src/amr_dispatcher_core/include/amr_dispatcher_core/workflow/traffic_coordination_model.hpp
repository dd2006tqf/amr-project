// 运行时交通协调状态模型与在途任务进度定义。
//
// 对应 Phase 7 (P0)：
// 1. TrafficCoordinationState：任务在运行时的交通控制状态（RUNNING / BLOCKED / REPLANNING / WAITING / RECOVERY_FAILED）。
// 2. InFlightTrafficWaitInfo：描述在途车辆因前方资源冲突而在安全道口等待的元信息。
// 3. 纯 C++ 领域模型，零 ROS/网络依赖，100% 宿主单测可覆盖。
#pragma once

#include <string>
#include <unordered_map>
#include <vector>

namespace amr_dispatcher_core::workflow {

enum class TrafficCoordinationState {
  kRunning,              // 正常行进中
  kBlocked,              // 感知到前方受阻 / 边封锁 / 资源争用
  kReplanning,           // 正在重规划替代路线
  kWaitingForResource,   // 在安全等待点等待路权释放
  kRecoveryFailed,       // 无替代路线且无法等待，告警挂起
};

const char* TrafficCoordinationStateLabel(TrafficCoordinationState state);

struct InFlightTrafficWaitInfo {
  std::string mission_id;
  std::string vehicle_id;
  std::string current_safe_node;              // 车辆当前/最近停靠的安全节点
  std::vector<std::string> waiting_resources; // 正在等待的前方路权锁（如 route_edge:A__B）
  TrafficCoordinationState state = TrafficCoordinationState::kRunning;
  std::string reason;
};

// 状态流转合法性校验：判定旧状态是否能合法转移到新状态
bool IsValidTrafficStateTransition(TrafficCoordinationState from, TrafficCoordinationState to);

}  // namespace amr_dispatcher_core::workflow
