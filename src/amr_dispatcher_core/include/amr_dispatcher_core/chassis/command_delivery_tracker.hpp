#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>

namespace amr_dispatcher_core::chassis {

// T3：命令送达率统计（纯逻辑，可脱离 ROS 单测）。
//
// 为什么需要它
// ------------
// 没有下行序号时，链路质量只能统计"读空了"和"CRC 错"：
//   * avg_latency_ms 恒为 0（RecordSuccess(0.0) 没有往返时延可用）；
//   * frames_lost 里的"丢包"是接收侧现象，命令在串口上真的丢了测不出来。
// MCU 回显 last_cmd_seq 之后，"我发了 N 帧、它收到的是第几帧"成为可观测量。
//
// 语义
// ----
// * 只统计**最近 window 帧**（滑动窗口），长期历史不稀释突发丢包，与
//   LinkQualityMonitor 的窗口语义保持一致；
// * seq 是 uint8 回绕，比较用"距最新已确认序号的环形距离"，不能直接用 <；
// * 只记录"比已确认序号更新"的回显：乱序/重复的旧回显不重复计数；
// * delivery_rate 在窗口未满时按已发帧数计算（不让启动期显示 0%）。
//
// 已知局限（刻意不处理）
// --------------------
// 无法区分"命令没到 MCU"与"回显帧丢了"。两者都表现为窗口里缺席，
// 对"链路是否可信"这个判断来说是同一件事，故不细分。
class CommandDeliveryTracker {
 public:
  explicit CommandDeliveryTracker(std::size_t window = 32) : window_(window == 0 ? 1 : window) {}

  // 每发一帧下行命令时调用，返回应写入该帧的 seq。
  uint8_t NextSequence() {
    const uint8_t seq = next_seq_;
    ++next_seq_;
    sent_.push_back(seq);
    if (sent_.size() > window_) {
      sent_.pop_front();
    }
    return seq;
  }

  // MCU 回显 last_cmd_seq 时调用。first=true 表示这是本次连接的首个回显
  // （此前没有基准，无法判断新旧，只能无条件接受）。
  void OnEcho(const uint8_t echoed_seq) {
    if (!has_echo_) {
      has_echo_ = true;
      last_confirmed_ = echoed_seq;
      echoed_.push_back(echoed_seq);
      TrimEchoed();
      return;
    }
    // 环形距离：正值表示 echoed 比已确认的更新（含回绕）。
    const uint8_t delta = static_cast<uint8_t>(echoed_seq - last_confirmed_);
    if (delta == 0) {
      return;  // 重复回显
    }
    if (delta > 128) {
      return;  // 落后于已确认序号超过半圈：陈旧/乱序回显，忽略
    }
    // 跨过的序号都算"MCU 收到了"（回显滞后几帧是常态，不是丢包）。
    last_confirmed_ = echoed_seq;
    echoed_.push_back(echoed_seq);
    TrimEchoed();
  }

  // 最近 window 帧的送达率；窗口内尚未有回显时为 0。
  double DeliveryRate() const {
    if (sent_.empty() || echoed_.empty()) {
      return 0.0;
    }
    std::size_t delivered = 0;
    for (const auto sent_seq : sent_) {
      if (WasDelivered(sent_seq)) {
        ++delivered;
      }
    }
    return static_cast<double>(delivered) / static_cast<double>(sent_.size());
  }

  // 窗口内已确认送达的帧数 / 发送帧数。
  std::size_t DeliveredCount() const {
    std::size_t delivered = 0;
    for (const auto sent_seq : sent_) {
      if (WasDelivered(sent_seq)) {
        ++delivered;
      }
    }
    return delivered;
  }

  std::size_t SentCount() const { return sent_.size(); }
  bool HasEcho() const { return has_echo_; }
  uint8_t LastConfirmedSequence() const { return last_confirmed_; }

  void Reset() {
    sent_.clear();
    echoed_.clear();
    has_echo_ = false;
    last_confirmed_ = 0;
    next_seq_ = 0;
  }

 private:
  // 某个已发序号是否已被"追上"：从它的下一帧到最新已确认序号的距离
  // 不超过半圈，说明 MCU 的回显已经越过它。
  // 用环形距离而不是朴素比较，否则 255 -> 0 回绕时整窗会瞬间判成全丢。
  bool WasDelivered(const uint8_t sent_seq) const {
    if (!has_echo_) {
      return false;
    }
    const uint8_t ahead = static_cast<uint8_t>(last_confirmed_ - sent_seq);
    return ahead < 128;
  }

  // 回显记录只用于判断"是否曾有过回显"，保留窗口长度即可，
  // 避免长时间运行后无界增长。
  void TrimEchoed() {
    while (echoed_.size() > window_) {
      echoed_.pop_front();
    }
  }

  std::size_t window_;
  std::deque<uint8_t> sent_;
  std::deque<uint8_t> echoed_;
  uint8_t next_seq_ = 0;
  uint8_t last_confirmed_ = 0;
  bool has_echo_ = false;
};

}  // namespace amr_dispatcher_core::chassis
