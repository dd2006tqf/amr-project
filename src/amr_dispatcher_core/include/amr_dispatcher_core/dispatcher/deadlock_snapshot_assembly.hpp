// 死锁快照组包（纯函数）。
//
// 原来这一步内联在 DispatcherNode::BuildDeadlockSnapshot 里，并且**自带副作用**：
// 它一边组包一边就地删掉 pending_wait_ 里的"幽灵等待者"（已不在队列里的任务）。
// 副作用让这块逻辑既不能单测（要造一个 dispatch 节点），也不能被
// DetectTrafficDeadlock 服务安全复用（读操作会改状态）。
//
// 这里拆成"纯组包 + 返回待清扫 id"：调用方拿到 stale_wait_mission_ids 后
// 自己决定何时 erase。语义与原来完全一致（等待者是否有效的判据仍是
// "任务是否还在队列里"），只是把写操作挪到了调用方。
#pragma once

#include <chrono>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "amr_dispatcher_core/dispatcher/deadlock_detector.hpp"
#include "amr_dispatcher_core/dispatcher/mission_state.hpp"

namespace amr_dispatcher_core::dispatcher {

// 组包所需的全部输入（由 ROS 侧从自己的容器里拷出来，core 不持有任何句柄）。
struct DeadlockSnapshotSources {
  // 当前 ACTIVE 的任务（mission_id -> Mission）
  std::unordered_map<std::string, Mission> active_missions;
  // mission_id -> 最近一次进度时刻（缺失视为默认构造值，与原实现一致）
  std::unordered_map<std::string, std::chrono::steady_clock::time_point> last_progress;
  // mission_id -> 路由锁预留成功时刻（长持锁启发式取证）
  std::unordered_map<std::string, std::chrono::steady_clock::time_point> reservation_acquired_at;
  // mission_id -> 该任务此刻在等的资源（Peek 预留失败记下的静态锁）
  std::unordered_map<std::string, std::vector<std::string>> pending_wait;
  // 预约表快照（resource_id -> owner_id）
  std::vector<std::pair<std::string, std::string>> reserved_resources;
  // 当前仍在排队的任务 id 集合：不在其中的等待者视为幽灵，需要清扫。
  // filter_stale_waiters=false 时（队列尚未建立，与原实现 queue_==nullptr 分支一致）
  // 不做幽灵判定，全部等待者原样进图。
  std::unordered_set<std::string> queued_mission_ids;
  bool filter_stale_waiters = true;
};

struct DeadlockSnapshotAssembly {
  DeadlockSnapshot snapshot;
  // 调用方应在组包之后从 pending_wait 里删除这些 key（原实现是就地 erase）
  std::vector<std::string> stale_wait_mission_ids;
};

// 组包。不修改输入，不读时钟，不持有引用。
DeadlockSnapshotAssembly AssembleDeadlockSnapshot(const DeadlockSnapshotSources& sources);

}  // namespace amr_dispatcher_core::dispatcher
