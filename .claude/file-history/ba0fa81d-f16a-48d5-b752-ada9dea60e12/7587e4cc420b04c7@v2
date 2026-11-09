#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include "amr_dispatcher_core/safety/cmd_vel_gate.hpp"
#include "amr_dispatcher_core/safety/fault_supervisor.hpp"
#include "amr_dispatcher_core/safety/watchdog.hpp"
#include "amr_dispatcher_interfaces/msg/chassis_link_health.hpp"
#include "amr_dispatcher_interfaces/msg/safety_state.hpp"
#include "amr_dispatcher_interfaces/msg/topology_state.hpp"

namespace amr_dispatcher_ros {

class SafetyGateNode : public rclcpp::Node {
 public:
  explicit SafetyGateNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~SafetyGateNode() override;

 private:
  void RawCmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void EstopCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void BumperCallback(const std_msgs::msg::Bool::SharedPtr msg);
  // B1 心跳分源：source 与 watched_nodes_ 中登记的来源名一致（"chassis_driver" / "path_tracker"），
  // 让 SafetyWatchdog 的 Inspect 能区分是谁在按时喂心跳。
  void WatchdogHeartbeatCallback(const std_msgs::msg::Bool::SharedPtr msg,
                                 const std::string& source);

  void WatchdogTick();
  // B1 50Hz 周期发布：保证 /cmd_vel_safe 有持续数据流，estop/bumper/上游失联均零速兜底。
  void PeriodicPublishTick();
  void PublishSafetyState(const amr_dispatcher_core::safety::CmdVelGateDecision& decision);
  void PublishIdleSafetyState();

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr raw_cmd_vel_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr teleop_cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr manual_takeover_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr bumper_sub_;
  // B1 心跳分源订阅（不再共用单话题）：每条只 Feed 自己的源，避免一侧挂掉拉低另一侧。
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr heartbeat_chassis_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr heartbeat_path_tracker_sub_;
  rclcpp::Subscription<amr_dispatcher_interfaces::msg::ChassisLinkHealth>::SharedPtr link_health_sub_;
  rclcpp::Subscription<amr_dispatcher_interfaces::msg::TopologyState>::SharedPtr topology_state_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr safe_cmd_vel_pub_;
  rclcpp::Publisher<amr_dispatcher_interfaces::msg::SafetyState>::SharedPtr safety_state_pub_;

  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  // B1 50Hz 兜底发布定时器（独立于 100ms WatchdogTick）
  rclcpp::TimerBase::SharedPtr publish_timer_;

  std::unique_ptr<amr_dispatcher_core::safety::CmdVelGate> gate_;
  std::unique_ptr<amr_dispatcher_core::safety::SafetyWatchdog> watchdog_;
  std::unique_ptr<amr_dispatcher_core::safety::FaultSupervisor> fault_supervisor_;

  bool estop_active_ = false;
  bool auto_fault_latched_ = false;
  bool bumper_active_ = false;
  bool manual_takeover_active_ = false;
  bool watchdog_ok_ = true;
  double chassis_loss_rate_ = 0.0;
  bool chassis_healthy_ = true;
  bool deadlock_active_ = false;
  std::vector<std::string> watched_nodes_;
  // 最近一次门控裁决结果，供空闲周期播报与就绪探针使用
  amr_dispatcher_core::safety::CmdVelGateDecision last_decision_{};
  // 最近一次 /cmd_vel_raw（含手动接管转发的 teleop）接收时刻，B1 上游失联兜底判 stale 用
  std::chrono::steady_clock::time_point last_raw_cmd_stamp_{};
  bool has_raw_cmd_stamp_ = false;
  // B1：是否已产生过至少一次门控裁决（周期发布重放 last_decision_ 的前置条件）
  bool has_decision_ = false;
  // I1：各数据源最后一次**真实数据到达**时刻（在订阅回调处打点，而非巡检周期），
  // 供 WatchdogTick 计算报告新鲜度 —— 上游发布者死亡后时效戳不再被调用周期续命。
  // key 约定："chassis"（/chassis/link_health）、"watchdog"（两条分源心跳）。
  std::unordered_map<std::string, std::chrono::steady_clock::time_point> last_data_arrival_;
};

}  // namespace amr_dispatcher_ros
