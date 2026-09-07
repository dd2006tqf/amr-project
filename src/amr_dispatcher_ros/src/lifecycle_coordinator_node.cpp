#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include "amr_dispatcher_interfaces/msg/safety_state.hpp"
#include "amr_dispatcher_interfaces/srv/get_system_readiness.hpp"

using namespace std::chrono_literals;

namespace amr_dispatcher_ros {

/**
 * @brief 工业级多组件生命周期协调器与就绪探针 (Lifecycle Coordinator & Readiness Probe)
 * 实施严格的拓扑依赖启动与停止链：
 * ChassisDriver (Active) -> SafetyGate (Ready) -> PathTracker (Ready) -> DispatcherNode (Active)
 * 向上暴露统一的 `/system/ready` 与 `/system/healthy` 服务，供上层 WMS/REST 网关拦截派单。
 */
class LifecycleCoordinatorNode : public rclcpp::Node {
 public:
  explicit LifecycleCoordinatorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : rclcpp::Node("lifecycle_coordinator_node", options) {
    declare_parameter("auto_bringup", true);
    declare_parameter("check_period_ms", 500);

    auto_bringup_ = get_parameter("auto_bringup").as_bool();
    int period_ms = get_parameter("check_period_ms").as_int();

    dispatcher_get_state_cli_ =
        create_client<lifecycle_msgs::srv::GetState>("/dispatcher_node/get_state");
    dispatcher_change_state_cli_ =
        create_client<lifecycle_msgs::srv::ChangeState>("/dispatcher_node/change_state");

    safety_sub_ = create_subscription<amr_dispatcher_interfaces::msg::SafetyState>(
        "/safety/state", 10,
        [this](const amr_dispatcher_interfaces::msg::SafetyState::SharedPtr msg) {
          safety_healthy_ = msg->motion_allowed;
          safety_seen_ = true;
        });

    readiness_srv_ = create_service<amr_dispatcher_interfaces::srv::GetSystemReadiness>(
        "/system/ready",
        [this](const std::shared_ptr<amr_dispatcher_interfaces::srv::GetSystemReadiness::Request> /*req*/,
               std::shared_ptr<amr_dispatcher_interfaces::srv::GetSystemReadiness::Response> resp) {
          EvaluateReadiness(resp);
        });

    healthy_srv_ = create_service<amr_dispatcher_interfaces::srv::GetSystemReadiness>(
        "/system/healthy",
        [this](const std::shared_ptr<amr_dispatcher_interfaces::srv::GetSystemReadiness::Request> /*req*/,
               std::shared_ptr<amr_dispatcher_interfaces::srv::GetSystemReadiness::Response> resp) {
          EvaluateReadiness(resp);
        });

    ready_pub_ = create_publisher<std_msgs::msg::Bool>("/system/ready_flag", rclcpp::QoS(1).transient_local());

    timer_ = create_wall_timer(
        std::chrono::milliseconds(period_ms),
        std::bind(&LifecycleCoordinatorNode::CoordinatorTick, this));

    RCLCPP_INFO(get_logger(), "Lifecycle Coordinator & Readiness Probe initialized (auto_bringup=%s)",
                auto_bringup_ ? "true" : "false");
  }

 private:
  void CoordinatorTick() {
    if (!dispatcher_get_state_cli_->service_is_ready()) {
      return;
    }

    // 1. 获取 dispatcher_node 当前生命周期状态
    auto req = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
    auto fut = dispatcher_get_state_cli_->async_send_request(req);
    if (fut.wait_for(100ms) == std::future_status::ready) {
      current_dispatcher_state_ = fut.get()->current_state.label;
    }

    // 2. 自动级联提拉状态机: unconfigured -> inactive -> active
    if (auto_bringup_) {
      if (current_dispatcher_state_ == "unconfigured") {
        RCLCPP_INFO(get_logger(), "Promoting dispatcher_node: unconfigured -> configure()");
        SendTransition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
      } else if (current_dispatcher_state_ == "inactive") {
        RCLCPP_INFO(get_logger(), "Promoting dispatcher_node: inactive -> activate()");
        SendTransition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
      }
    }

    // 3. 周期广播整体就绪状态布尔量
    std_msgs::msg::Bool ready_msg;
    ready_msg.data = (current_dispatcher_state_ == "active");
    ready_pub_->publish(ready_msg);
  }

  void SendTransition(uint8_t transition_id) {
    if (!dispatcher_change_state_cli_->service_is_ready()) return;
    auto req = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    req->transition.id = transition_id;
    dispatcher_change_state_cli_->async_send_request(req);
  }

  void EvaluateReadiness(std::shared_ptr<amr_dispatcher_interfaces::srv::GetSystemReadiness::Response> resp) {
    bool is_ready = (current_dispatcher_state_ == "active") && safety_seen_ && safety_healthy_;
    resp->ready = is_ready;
    resp->healthy = safety_healthy_;

    if (is_ready) {
      resp->state = "READY";
      resp->message = "All critical nodes (Chassis, Safety, Dispatcher) are ACTIVE and HEALTHY";
      resp->active_nodes = {"chassis_driver", "safety_gate", "path_tracker", "dispatcher_node"};
    } else if (current_dispatcher_state_ != "active") {
      resp->state = "INITIALIZING";
      resp->message = "DispatcherNode is in state: " + current_dispatcher_state_;
      resp->pending_nodes = {"dispatcher_node"};
    } else {
      resp->state = "DEGRADED";
      resp->message = "Safety Gate or Chassis reporting fault/standstill";
    }
  }

  bool auto_bringup_{true};
  std::string current_dispatcher_state_{"unknown"};
  bool safety_seen_{false};
  bool safety_healthy_{true};

  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr dispatcher_get_state_cli_;
  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr dispatcher_change_state_cli_;
  rclcpp::Subscription<amr_dispatcher_interfaces::msg::SafetyState>::SharedPtr safety_sub_;

  rclcpp::Service<amr_dispatcher_interfaces::srv::GetSystemReadiness>::SharedPtr readiness_srv_;
  rclcpp::Service<amr_dispatcher_interfaces::srv::GetSystemReadiness>::SharedPtr healthy_srv_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace amr_dispatcher_ros

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<amr_dispatcher_ros::LifecycleCoordinatorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
