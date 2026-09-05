#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

namespace amr_dispatcher_core::dispatcher {

// 任务生命周期状态：4 态 + 结果分离设计。
//
//   kPending ──► kActive ──► kFinished
//       ▲  │        │  ▲
//       │  ▼        ▼  │
//       └─ kPaused ◄───┘
//
// 终态结果（kSucceeded/kFailed/kCanceled）与状态正交，单独记录，
// 避免用状态字符串编码结果导致的组合爆炸。
enum class MissionState { kPending, kActive, kPaused, kFinished };

enum class MissionResult { kNone, kSucceeded, kFailed, kCanceled };

// 合法迁移表校验。to==from 一律视为非法（幂等重复到达应被上层去重，而非放行）。
bool CanTransition(MissionState from, MissionState to);

std::optional<MissionState> ParseMissionState(std::string_view text);
std::optional<MissionResult> ParseMissionResult(std::string_view text);
std::string_view ToString(MissionState state);
std::string_view ToString(MissionResult result);

// 一次状态迁移的完整记录，供事件日志与状态机回调复用。
struct MissionTransition {
  MissionState from = MissionState::kPending;
  MissionState to = MissionState::kPending;
  std::string message;
};

// 调度域任务实体。纯数据，ROS 话题转换在 ros 层完成。
struct Mission {
  std::string id;
  std::string order_id;              // 关联业务订单号（批量提交/整单取消用），可空
  std::string fleet_id = "default";  // V1.2 多机预留
  std::string type = "transport";    // transport / charge / patrol ...
  std::string pickup_station;
  std::string dropoff_station;
  int priority = 0;
  std::uint64_t sequence = 0;          // 入队序号，同优先级先到先服务
  std::int64_t deadline_unix_ms = 0;   // <=0 表示无截止期
  int expected_seconds = 0;            // 预估耗时，供 SJF 类策略
  MissionState state = MissionState::kPending;
  MissionResult result = MissionResult::kNone;
  int attempt = 0;                     // 已执行次数（含重试）
  std::string reservation_id;          // 任务当前持有的 traffic 资源 id（可空）
};

// 状态机封装：所有状态变更必须经过 Transition()，杜绝散落的裸赋值。
class MissionStateMachine {
 public:
  explicit MissionStateMachine(Mission mission = {});

  // 校验并执行迁移。to==kFinished 时要求 result != kNone；非终态要求 result == kNone。
  // 非法迁移返回 false 并填充 reason，状态保持不变。
  bool Transition(MissionState to, MissionResult result, std::string* reason = nullptr);

  // 进入 kFinished 的便捷入口。
  bool Finish(MissionResult result, std::string* reason = nullptr);

  const Mission& mission() const { return mission_; }
  Mission& mutable_mission() { return mission_; }
  MissionState state() const { return mission_.state; }
  MissionResult result() const { return mission_.result; }

  // 累计迁移次数与最近一次迁移记录（供诊断话题）。
  std::uint64_t transition_count() const { return transition_count_; }
  const MissionTransition& last_transition() const { return last_transition_; }

 private:
  Mission mission_;
  std::uint64_t transition_count_ = 0;
  MissionTransition last_transition_;
};

}  // namespace amr_dispatcher_core::dispatcher
