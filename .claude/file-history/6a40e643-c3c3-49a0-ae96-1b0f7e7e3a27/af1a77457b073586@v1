#include "amr_dispatcher_core/dispatcher/deadlock_snapshot_assembly.hpp"

#include <algorithm>

namespace amr_dispatcher_core::dispatcher {

DeadlockSnapshotAssembly AssembleDeadlockSnapshot(const DeadlockSnapshotSources& sources) {
  DeadlockSnapshotAssembly assembly;
  DeadlockSnapshot& snap = assembly.snapshot;

  // 等待记录只对仍在排队的任务有效：被批量取消/离场的等待者不进 waiting_for
  //（防 WFG 残留幽灵等待者）。原实现是就地 erase，这里改为"记下 id 由调用方删"。
  for (const auto& [mission_id, _] : sources.pending_wait) {
    if (sources.filter_stale_waiters &&
        sources.queued_mission_ids.find(mission_id) == sources.queued_mission_ids.end()) {
      assembly.stale_wait_mission_ids.push_back(mission_id);
    }
  }
  // 稳定的输出顺序：调用方可能按顺序写日志/事件，哈希序会让两次相同输入
  // 产出不同顺序（e2e 断言会因此变成"偶尔不一致"）
  std::sort(assembly.stale_wait_mission_ids.begin(), assembly.stale_wait_mission_ids.end());

  for (const auto& [id, mission] : sources.active_missions) {
    snap.active_missions.push_back(mission);
    const auto progress_it = sources.last_progress.find(id);
    // 缺失 progress 记录时插入默认构造的时间点——与原实现 mission_last_progress_[id]
    // 的隐式插入行为一致（operator[] 语义）
    snap.last_progress[id] = (progress_it == sources.last_progress.end())
                                 ? std::chrono::steady_clock::time_point{}
                                 : progress_it->second;
  }

  // WFG 数据上环：held_by 与预留资源集合来自预约表快照（resource -> owner）
  for (const auto& [resource_id, owner_id] : sources.reserved_resources) {
    snap.allocation_graph.held_by[resource_id] = owner_id;
    snap.reserved_resource_ids.insert(resource_id);
  }

  // waiting_for 只收有效等待者（幽灵等待者已由调用方清扫，这里也不放进去）
  for (const auto& [mission_id, resources] : sources.pending_wait) {
    if (sources.filter_stale_waiters &&
        sources.queued_mission_ids.find(mission_id) == sources.queued_mission_ids.end()) {
      continue;
    }
    snap.allocation_graph.waiting_for[mission_id] = resources;
  }

  snap.reservation_acquired_at = sources.reservation_acquired_at;
  return assembly;
}

}  // namespace amr_dispatcher_core::dispatcher
