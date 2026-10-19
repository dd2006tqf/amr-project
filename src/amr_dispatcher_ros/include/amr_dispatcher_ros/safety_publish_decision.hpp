#pragma once

#include <chrono>
#include <string>

namespace amr_dispatcher_ros {

// 安全门 50Hz 周期发布的纯判定逻辑（B1）：
// 从 ROS 消息与节点状态中抽出，保证可脱离 rclcpp 单测。
// 语义：
//   - estop / auto_fault / bumper / critical 触发 -> 周期帧强制零速；
//   - 上次 /cmd_vel_raw 距 now 超过 heartbeat_timeout -> 周期帧零速（上游失联兜底）；
//   - 否则重发最近一次门控裁决值；从未收到过 raw 指令（has_decision=false）时不发。
enum class PeriodicPublishKind { kSkip, kZero, kDecision };

struct PeriodicPublishInput {
  bool estop_or_fault = false;
  bool bumper = false;
  bool decision_critical = false;
  bool decision_warned = false;
  bool has_decision = false;              // 是否已收到过至少一条 /cmd_vel_raw
  bool has_last_raw_stamp = false;        // last_raw_stamp 是否有效
  std::chrono::steady_clock::time_point last_raw_stamp{};
  std::chrono::steady_clock::time_point now{};
  std::chrono::milliseconds heartbeat_timeout{500};
  double last_linear_x_mps = 0.0;
  double last_linear_y_mps = 0.0;
  double last_angular_z_radps = 0.0;
};

struct PeriodicPublishOutput {
  PeriodicPublishKind kind = PeriodicPublishKind::kSkip;
  double linear_x_mps = 0.0;
  double linear_y_mps = 0.0;
  double angular_z_radps = 0.0;
  std::string reason;
};

inline PeriodicPublishOutput ComputePeriodicPublish(const PeriodicPublishInput& in) {
  PeriodicPublishOutput out;

  // 阻止源激活：estop/auto_fault/bumper 或门控裁决 critical，周期帧一律零速。
  if (in.estop_or_fault || in.bumper || in.decision_critical) {
    out.kind = PeriodicPublishKind::kZero;
    out.reason = in.estop_or_fault   ? "estop"
                 : in.bumper         ? "bumper"
                                     : "critical_event";
    return out;
  }

  // 上游失联兜底：已经收到过 raw 指令且距上次接收超过超时阈值 -> 零速。
  //（从未收到 raw 的系统视为"尚未开始运动"，不持续发零速，避免空转告警）
  if (in.has_last_raw_stamp &&
      in.now - in.last_raw_stamp > in.heartbeat_timeout) {
    out.kind = PeriodicPublishKind::kZero;
    out.reason = "cmd_vel_raw_stale";
    return out;
  }

  // 正常路径：重放最近一次裁决结果；尚未裁决过则跳过。
  if (!in.has_decision) {
    out.kind = PeriodicPublishKind::kSkip;
    return out;
  }
  out.kind = PeriodicPublishKind::kDecision;
  out.linear_x_mps = in.last_linear_x_mps;
  out.linear_y_mps = in.last_linear_y_mps;
  out.angular_z_radps = in.last_angular_z_radps;
  return out;
}

}  // namespace amr_dispatcher_ros
