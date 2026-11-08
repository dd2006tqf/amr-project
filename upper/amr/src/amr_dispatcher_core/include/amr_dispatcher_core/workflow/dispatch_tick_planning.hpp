// 一次派发 tick 的选车扫描决策（纯函数）。
//
// 原实现是 SelectReadyCandidate 里的一段串行内联循环，混了四类动作：
//   1. 选车三态处置（留队 / 暂停自动分配 / 判负）
//   2. 路由锁预留尝试（静态排他锁 或 时空窗预约）
//   3. 预留被拒时的记账（pending_wait_）与事件（reservation_denied）
//   4. 命中后出队派发
// 其中 1、3 的**相对顺序**、以及"一 tick 只派一个"的短路位置，是兼容红线，
// 却只能靠起 ROS 图 + 跑 e2e 才能验。组包与副作用分离后即可单测。
//
// 设计：预留在本函数之外完成（由调用方注入 `try_reserve` 回调，回调内部
// 落表），本函数只按队列顺序产出**有序 step 列表**。产出聚合 map 会丢序：
// `kRecordWait` / `kEmitReservationDenied` / `kRejectCandidate` 的相对顺序
// 与原实现逐条对应。
#pragma once

#include <functional>
#include <string>
#include <vector>

#include "amr_dispatcher_core/dispatcher/mission_state.hpp"
#include "amr_dispatcher_core/fleet/vehicle_assignment.hpp"

namespace amr_dispatcher_core::workflow {

// 一个待扫描候选：任务 + 选车结论 + 预计算的路由锁（空 = 无锁可预留）。
struct DispatchCandidate {
  dispatcher::Mission mission;
  fleet::VehicleAssignmentDecision assignment;
  std::vector<std::string> route_locks;
};

// 预留尝试的结果。denial_kind 决定是否发 reservation_denied 事件：
// 静态锁被拒会发（事件消费者依赖），时空窗被拒只告警不发——这是原实现的差异。
struct ReservationAttempt {
  enum class DenialKind {
    kNone,      // 成功（或无需预留）
    kTimeSpace, // 时空窗预约被拒：只告警 + 记账
    kStatic,    // 静态锁被拒：告警 + 记账 + reservation_denied 事件
  };
  bool success = false;
  DenialKind denial_kind = DenialKind::kNone;
  std::string message;
};

struct DispatchTickRequest {
  // 按队列顺序（PeekN 的结果顺序）
  std::vector<DispatchCandidate> candidates;
  // 预留回调：返回成功与否 + 拒绝类型。允许为空（表示"无需预留，全部成功"）。
  std::function<ReservationAttempt(const std::string& mission_id,
                                   const std::vector<std::string>& route_locks)>
      try_reserve;
};

struct DispatchStep {
  enum class Kind {
    kRejectCandidate,       // 永久不满足 -> 该单判负
    kRecordWait,            // 预留被拒 -> 告警 + 记 pending_wait_（留队等待）
    kEmitReservationDenied, // 静态锁被拒 -> 额外发 reservation_denied 事件
    kDispatch,              // 命中：出队派发（隐含清除该单的 pending_wait_ 记录）
  };
  Kind kind = Kind::kRejectCandidate;
  std::string mission_id;
  std::string reason;
  std::vector<std::string> route_locks;  // kRecordWait / kDispatch 用
  std::string vehicle_id;                // 仅 kDispatch
  // 仅 kRecordWait：拒绝来源。决定告警文案与"先告警还是先记账"的顺序——
  // 时空窗被拒是"先告警后记账"，静态锁被拒是"先记账后告警"（原实现如此）。
  ReservationAttempt::DenialKind denial_kind = ReservationAttempt::DenialKind::kNone;
};

struct DispatchTickDecision {
  // 有序 step 列表（按候选顺序，命中后不再有后续 step）
  std::vector<DispatchStep> steps;
  bool saw_deferred = false;
  std::string deferred_reason;
  bool saw_indeterminate = false;
  std::string indeterminate_reason;
  bool dispatched = false;
  std::string dispatched_mission_id;
  std::string dispatched_vehicle_id;
  std::vector<std::string> dispatched_route_locks;
};

inline DispatchTickDecision PlanDispatchTick(const DispatchTickRequest& request) {
  DispatchTickDecision decision;
  for (const auto& candidate : request.candidates) {
    const std::string& mission_id = candidate.mission.id;
    // 1) 三态处置（与 DecideAssignmentDispatch 同一映射，这里内联以保持
    //    "step 顺序 = 原实现语句顺序"可读）
    switch (candidate.assignment.outcome) {
      case fleet::AssignmentOutcome::kDeferred:
        decision.saw_deferred = true;
        decision.deferred_reason = candidate.assignment.reject_reason;
        continue;  // 留队等待，下个 tick 重试
      case fleet::AssignmentOutcome::kIndeterminate:
        decision.saw_indeterminate = true;
        decision.indeterminate_reason = candidate.assignment.reject_reason;
        continue;  // 状态不确定：暂停自动分配
      case fleet::AssignmentOutcome::kRejected: {
        DispatchStep step;
        step.kind = DispatchStep::Kind::kRejectCandidate;
        step.mission_id = mission_id;
        step.reason = candidate.assignment.reject_reason;
        decision.steps.push_back(std::move(step));
        continue;  // 永久不满足 -> 判负后继续扫描（不静默改派）
      }
      case fleet::AssignmentOutcome::kAssigned:
        break;
    }
    // 2) 预留（由调用方注入的回调落地）
    ReservationAttempt attempt;
    attempt.success = true;
    if (request.try_reserve) {
      attempt = request.try_reserve(mission_id, candidate.route_locks);
    }
    if (!attempt.success) {
      DispatchStep wait;
      wait.kind = DispatchStep::Kind::kRecordWait;
      wait.mission_id = mission_id;
      wait.reason = attempt.message;
      wait.route_locks = candidate.route_locks;
      wait.denial_kind = attempt.denial_kind;
      decision.steps.push_back(std::move(wait));
      if (attempt.denial_kind == ReservationAttempt::DenialKind::kStatic) {
        DispatchStep denied;
        denied.kind = DispatchStep::Kind::kEmitReservationDenied;
        denied.mission_id = mission_id;
        denied.reason = attempt.message;
        decision.steps.push_back(std::move(denied));
      }
      continue;  // 换下一个候选（队首阻塞规避）
    }
    // 3) 命中：本 tick 到此为止
    DispatchStep dispatch;
    dispatch.kind = DispatchStep::Kind::kDispatch;
    dispatch.mission_id = mission_id;
    dispatch.vehicle_id = candidate.assignment.vehicle_id;
    dispatch.route_locks = candidate.route_locks;
    decision.steps.push_back(std::move(dispatch));
    decision.dispatched = true;
    decision.dispatched_mission_id = mission_id;
    decision.dispatched_vehicle_id = candidate.assignment.vehicle_id;
    decision.dispatched_route_locks = candidate.route_locks;
    break;  // 一 tick 只派一个
  }
  return decision;
}

}  // namespace amr_dispatcher_core::workflow
