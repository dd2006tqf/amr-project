#pragma once

#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include "amr_dispatcher_core/safety/cmd_vel_gate.hpp"
#include "amr_dispatcher_core/safety/watchdog.hpp"
#include "amr_dispatcher_interfaces/msg/safety_state.hpp"

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
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr bumper_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr heartbeat_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr safe_cmd_vel_pub_;
  rclcpp::Publisher<amr_dispatcher_interfaces::msg::SafetyState>::SharedPtr safety_state_pub_;

  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  std::unique_ptr<amr_dispatcher_core::safety::CmdVelGate> gate_;
  std::unique_ptr<amr_dispatcher_core::safety::SafetyWatchdog> watchdog_;

  bool estop_active_ = false;
  bool bumper_active_ = false;
  std::vector<std::string> watched_nodes_;
};

}  // namespace amr_dispatcher_ros
