#include "amr_dispatcher_ros/safety_gate_node.hpp"

#include <chrono>

using namespace std::chrono_literals;

namespace amr_dispatcher_ros {

SafetyGateNode::SafetyGateNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("safety_gate_node", options) {
  declare_parameter("max_linear_x_mps", 1.5);
  declare_parameter("max_linear_y_mps", 1.0);
  declare_parameter("max_angular_z_radps", 2.0);
  declare_parameter("heartbeat_timeout_ms", 500);
  declare_parameter("soft_timeout_ms", 2000);

  amr_dispatcher_core::safety::CmdVelGateConfig gate_cfg;
  gate_cfg.max_linear_x_mps = get_parameter("max_linear_x_mps").as_double();
  gate_cfg.max_linear_y_mps = get_parameter("max_linear_y_mps").as_double();
  gate_cfg.max_angular_z_radps = get_parameter("max_angular_z_radps").as_double();
  gate_cfg.source_blocked = [this](const std::string& src) {
    if (src == "estop") return estop_active_;
    if (src == "bumper") return bumper_active_;
    if (src.find("watchdog") != std::string::npos) return true;
    return false;
  };

  gate_ = std::make_unique<amr_dispatcher_core::safety::CmdVelGate>(gate_cfg);

  amr_dispatcher_core::safety::WatchdogConfig wd_cfg;
  wd_cfg.heartbeat_timeout = std::chrono::milliseconds(get_parameter("heartbeat_timeout_ms").as_int());
  wd_cfg.soft_timeout = std::chrono::milliseconds(get_parameter("soft_timeout_ms").as_int());
  watchdog_ = std::make_unique<amr_dispatcher_core::safety::SafetyWatchdog>(wd_cfg);

  watched_nodes_ = {"chassis_driver", "path_tracker"};
  for (const auto& node : watched_nodes_) {
    watchdog_->FeedHeartbeat(node);
  }

  raw_cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel_raw", 10,
      std::bind(&SafetyGateNode::RawCmdVelCallback, this, std::placeholders::_1));

  estop_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/safety/estop", 10,
      std::bind(&SafetyGateNode::EstopCallback, this, std::placeholders::_1));

  bumper_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/safety/bumper", 10,
      std::bind(&SafetyGateNode::BumperCallback, this, std::placeholders::_1));

  heartbeat_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/safety/heartbeat", 10,
      std::bind(&SafetyGateNode::WatchdogHeartbeatCallback, this, std::placeholders::_1));

  safe_cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_safe", 10);
  safety_state_pub_ = create_publisher<amr_dispatcher_interfaces::msg::SafetyState>(
      "/safety/state", 10);

  watchdog_timer_ = create_wall_timer(
      100ms, std::bind(&SafetyGateNode::WatchdogTick, this));

  RCLCPP_INFO(get_logger(), "SafetyGateNode initialized (max_vx=%.2f, max_wz=%.2f)",
              gate_cfg.max_linear_x_mps, gate_cfg.max_angular_z_radps);
}

SafetyGateNode::~SafetyGateNode() = default;

void SafetyGateNode::EstopCallback(const std_msgs::msg::Bool::SharedPtr msg) {
  estop_active_ = msg->data;
  if (estop_active_) {
    amr_dispatcher_core::safety::SafetyEvent evt;
    evt.source = "estop";
    evt.severity = amr_dispatcher_core::safety::SafetySeverity::kCritical;
    evt.message = "Emergency stop button pressed";
    gate_->PushEvent(evt);
    RCLCPP_ERROR(get_logger(), "EMERGENCY STOP ACTIVATED");
  }
}

void SafetyGateNode::BumperCallback(const std_msgs::msg::Bool::SharedPtr msg) {
  bumper_active_ = msg->data;
  if (bumper_active_) {
    amr_dispatcher_core::safety::SafetyEvent evt;
    evt.source = "bumper";
    evt.severity = amr_dispatcher_core::safety::SafetySeverity::kCritical;
    evt.message = "Front bumper triggered collision";
    gate_->PushEvent(evt);
    RCLCPP_ERROR(get_logger(), "BUMPER TRIGGERED");
  }
}

void SafetyGateNode::WatchdogHeartbeatCallback(const std_msgs::msg::Bool::SharedPtr /*msg*/) {
  if (watchdog_) {
    watchdog_->FeedHeartbeat("path_tracker");
    watchdog_->FeedHeartbeat("chassis_driver");
  }
}

void SafetyGateNode::WatchdogTick() {
  if (!watchdog_ || !gate_) return;
  auto events = watchdog_->Inspect(watched_nodes_);
  for (auto& ev : events) {
    gate_->PushEvent(ev);
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "Watchdog event: source=%s msg=%s", ev.source.c_str(), ev.message.c_str());
  }
}

void SafetyGateNode::RawCmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
  if (!gate_) return;

  amr_dispatcher_core::safety::CmdVelCommand cmd;
  cmd.linear_x_mps = msg->linear.x;
  cmd.linear_y_mps = msg->linear.y;
  cmd.angular_z_radps = msg->angular.z;

  auto decision = gate_->Evaluate(cmd);

  geometry_msgs::msg::Twist safe_cmd;
  safe_cmd.linear.x = decision.linear_x_mps;
  safe_cmd.linear.y = decision.linear_y_mps;
  safe_cmd.angular.z = decision.angular_z_radps;
  safe_cmd_vel_pub_->publish(safe_cmd);

  PublishSafetyState(decision);
}

void SafetyGateNode::PublishSafetyState(const amr_dispatcher_core::safety::CmdVelGateDecision& decision) {
  amr_dispatcher_interfaces::msg::SafetyState state_msg;
  state_msg.header.stamp = now();
  state_msg.motion_allowed = decision.allowed;
  state_msg.reason = decision.reason;

  if (decision.severity == amr_dispatcher_core::safety::SafetySeverity::kCritical) {
    state_msg.severity = "CRITICAL";
    state_msg.speed_scale = 0.0;
  } else if (decision.severity == amr_dispatcher_core::safety::SafetySeverity::kWarn) {
    state_msg.severity = "WARN";
    state_msg.speed_scale = 0.3;
  } else {
    state_msg.severity = "INFO";
    state_msg.speed_scale = 1.0;
  }

  if (estop_active_) state_msg.active_sources.push_back("estop");
  if (bumper_active_) state_msg.active_sources.push_back("bumper");
  if (!state_msg.active_sources.empty()) {
    state_msg.active_stop_source = state_msg.active_sources.front();
  }

  safety_state_pub_->publish(state_msg);
}

}  // namespace amr_dispatcher_ros

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<amr_dispatcher_ros::SafetyGateNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
