// VDA 5050 受管车辆链路态（纯数据 + 纯迁移）。
//
// 现状问题：DispatcherNode::Vda5050Link 把"ROS 订阅句柄"和"链路纯数据"塞进
// 同一个结构体，导致：
//   * 想测"离线期间收到 state 不得回置 online"必须起 ROS 图（e2e Stage 6.4
//     当年就是靠 kill -9 真车才复现出来的）；
//   * 链路状态无法喂给 core 的纯函数（选车、守卫都要先从 ROS 结构里抠字段）。
//
// 这里把纯数据与迁移规则拆出来。ROS 侧保留句柄表（vda5050_link_subs_），
// 数据表（vda5050_link_states_）用本类型的对象。
//
// 铁律（与本文件所有函数相关）：
//   * 离线期间收到 state **绝不**回置 online —— DDS 跨话题乱序会把断链前的
//     最后一帧 state 排在 online=false 之后送达，据此回置会让链路"永久在线"
//     并绕过 Spec §5.2 的调度挂起；
//   * 链路存活的唯一权威判据是网关在 MQTT 有序流中确认断链之后、收到重连后的
//     新 state 或 connection=ONLINE 时重发的 online=true。
#pragma once

#include <chrono>
#include <string>

namespace amr_dispatcher_core::fleet {

struct Vda5050LinkState {
  // 乐观初值：online=false 到达前不阻塞派发（与网关 transient_local 重放收敛）
  bool online = true;
  bool seen_state = false;

  // 最近一帧 state 的定位与电量（选车输入：位置不可用即不合格）
  bool position_valid = false;
  double position_x = 0.0;
  double position_y = 0.0;
  double battery_percent = 100.0;

  // 陈旧判定所需：最近一帧 state 的时刻（单调时钟；默认值 = 从未收到）。
  // core 不读时钟，调用方在收到 state 时传入 now()。
  std::chrono::steady_clock::time_point last_state_time{};

  // 隔离态（人工强清后置入）。隔离不等于离线：仍在受管清单、仍可见可审计，
  // 但不参与自动分配；它被放进链路态里是为了让"链路 + 隔离"这一个整体
  // 能直接喂给 core 的守卫/选车纯函数。
  bool quarantined = false;
};

// 一帧合法 state 报文里与链路态相关的字段。
struct Vda5050StateObservation {
  bool position_initialized = false;
  double x = 0.0;
  double y = 0.0;
  double battery_percent = 100.0;
};

enum class LinkStateApplyAction {
  kUpdated,                  // 正常更新
  kUpdatedWhileOffline,      // 更新了数据，但链路仍是离线（调用方应打告警）
};

// 应用一帧 state。**不改 online**：离线时只更新数据并返回 kUpdatedWhileOffline。
LinkStateApplyAction ApplyStateObservation(
    Vda5050LinkState* state, const Vda5050StateObservation& observation,
    std::chrono::steady_clock::time_point now);

// 应用一条 online 报文。返回值表示"链路状态是否真的翻转"：
// 重复报文（transient_local 重放）不翻转，调用方据此避免重复告警。
bool ApplyOnlineReport(Vda5050LinkState* state, bool online);

// state 是否陈旧：online && seen_state && 距最近一次 state 超过阈值。
// 从未收到 state 的在线车**不算**陈旧（没有"多久没更新"可比）。
bool IsStateStale(const Vda5050LinkState& state, std::chrono::steady_clock::time_point now,
                  std::chrono::milliseconds stale_threshold);

}  // namespace amr_dispatcher_core::fleet
