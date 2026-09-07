#pragma once

#include <memory>
#include <string>
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
  void WatchdogHeartbeatCallback(const std_msgs::msg::Bool::SharedPtr msg);

  void WatchdogTick();
  void PublishSafetyState(const amr_dispatcher_core::safety::CmdVelGateDecision& decision);

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr raw_cmd_vel_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr teleop_cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr manual_takeover_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr bumper_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr heartbeat_sub_;
  rclcpp::Subscription<amr_dispatcher_interfaces::msg::ChassisLinkHealth>::SharedPtr link_health_sub_;
  rclcpp::Subscription<amr_dispatcher_interfaces::msg::TopologyState>::SharedPtr topology_state_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr safe_cmd_vel_pub_;
  rclcpp::Publisher<amr_dispatcher_interfaces::msg::SafetyState>::SharedPtr safety_state_pub_;

  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  std::unique_ptr<amr_dispatcher_core::safety::CmdVelGate> gate_;
  std::unique_ptr<amr_dispatcher_core::safety::SafetyWatchdog> watchdog_;
  std::unique_ptr<amr_dispatcher_core::safety::FaultSupervisor> fault_supervisor_;

  bool estop_active_ = false;
  bool bumper_active_ = false;
  bool manual_takeover_active_ = false;
  bool watchdog_ok_ = true;
  double chassis_loss_rate_ = 0.0;
  bool chassis_healthy_ = true;
  bool deadlock_active_ = false;
  std::vector<std::string> watched_nodes_;
};

}  // namespace amr_dispatcher_ros
