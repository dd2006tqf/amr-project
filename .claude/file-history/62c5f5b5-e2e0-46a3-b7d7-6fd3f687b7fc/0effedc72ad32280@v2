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
#include "amr_dispatcher_proto_bridge/proto_dual_subscriber.hpp"

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
    // 安全状态新鲜度（秒）：safety_gate 停发后 "motion_allowed" 会永远停在最后一次
    // 的值上 —— 只看值不看时间，等于安全门死了以后就绪还是 true。
    declare_parameter("safety_stale_sec", 2.0);
    // dispatcher 存活判据（秒）。这是"容器 Up 8 days、dispatcher 已死 8 天、
    // 网关照样回 200"那次的直接病根：CoordinatorTick 在 get_state 服务不可用时
    // 直接 return，current_dispatcher_state_ 永远停在最后一次读到的 "active"，
    // 于是 /system/ready 一直报 READY。宽限期比 check_period 略大，
    // 让服务端短暂抖动不至于误判。
    declare_parameter("dispatcher_stale_sec", 3.0);

    auto_bringup_ = get_parameter("auto_bringup").as_bool();
    int period_ms = get_parameter("check_period_ms").as_int();
    safety_stale_sec_ = get_parameter("safety_stale_sec").as_double();
    dispatcher_stale_sec_ = get_parameter("dispatcher_stale_sec").as_double();

    dispatcher_get_state_cli_ =
        create_client<lifecycle_msgs::srv::GetState>("/dispatcher_node/get_state");
    dispatcher_change_state_cli_ =
        create_client<lifecycle_msgs::srv::ChangeState>("/dispatcher_node/change_state");

    // 阶段 3：安全状态可走 proto 话题。默认 false，行为与迁移前逐字相同；
    // 置 true 时订阅 /safety/state_proto 并在适配器内部反投影回 rosidl，
    // 下面这个回调（以及整个就绪判定逻辑）完全不用改。
    declare_parameter("proto_dual_subscribe", false);
    const bool use_proto = get_parameter("proto_dual_subscribe").as_bool();

    safety_sub_ = amr_dispatcher_proto_bridge::ProtoDualSubscriber<
        amr_dispatcher_interfaces::msg::SafetyState,
        amr_dispatcher::proto::SafetyState>(
        this, "/safety/state", 10, use_proto,
        [this](const amr_dispatcher_interfaces::msg::SafetyState::SharedPtr msg) {
          safety_healthy_ = msg->motion_allowed;
          safety_seen_ = true;
          safety_last_seen_ = std::chrono::steady_clock::now();
        });
    if (use_proto) {
      RCLCPP_INFO(get_logger(),
                  "Safety state source: proto topic '%s' (reverse-projected)",
                  safety_sub_.proto_topic().c_str());
    }

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
      // 服务不可用 = dispatcher 已死（或还没起来）。**不能**直接 return：
      // 那样 current_dispatcher_state_ 会一直停在最后的 "active"，
      // 就绪判定与 ready_flag 都会撒谎。这里显式把状态标为不可用，
      // 交给 EvaluateReadiness / ready_flag 报告未就绪。
      MarkDispatcherUnavailable("get_state service unavailable");
      return;
    }

    // 1. 获取 dispatcher_node 当前生命周期状态。
    //    单线程 executor 下 spin 期间不能嵌套 wait_for future（会永久阻塞），
    //    必须异步发送 + 轮询 shared_future 的就绪位。
    if (!pending_get_state_.valid()) {
      auto req = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
      pending_get_state_ = dispatcher_get_state_cli_->async_send_request(req).share();
      return;
    }

    if (pending_get_state_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
      // 尚未返回，下一拍再看；超时则放弃本次请求避免悬挂
      if (++get_state_wait_ticks_ > 10) {
        pending_get_state_ = {};
        get_state_wait_ticks_ = 0;
        // 连续多拍拿不到应答 = DDS 卡死或进程假死。与会话丢失同样处理：
        // 不标记就绪，等下一轮真正拿到状态再恢复。
        MarkDispatcherUnavailable("get_state request timed out");
      }
      return;
    }
    current_dispatcher_state_ = pending_get_state_.get()->current_state.label;
    dispatcher_last_ok_ = std::chrono::steady_clock::now();
    dispatcher_seen_ = true;
    pending_get_state_ = {};
    get_state_wait_ticks_ = 0;

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

  /// dispatcher 状态不可用（进程死掉 / DDS 卡死 / 尚未启动）。
  ///
  /// 语义上"不可用"和"未激活"必须区分：
  ///   * 未激活（inactive/unconfigured）= 正常的启动中间态，等提拉即可；
  ///   * 不可用 = 进程根本不在，提拉也无从谈起，需要人工介入（respawn 兜底）。
  /// 但它俩对上层门禁的结论一致：**不放行**。所以对 /system/ready 用
  /// INITIALIZING（与 lifecycle 契约测试锁定的词汇一致：未就绪 = 等），
  /// 对 /system/healthy 明确报 unhealthy —— 后者才是给"存活探针/告警"看的。
  void MarkDispatcherUnavailable(const std::string& detail) {
    const bool was_available = current_dispatcher_state_ != "unavailable";
    current_dispatcher_state_ = "unavailable";
    dispatcher_unavailable_detail_ = detail;
    if (was_available) {
      RCLCPP_ERROR(get_logger(), "dispatcher_node unavailable (%s); readiness will report NOT READY",
                   detail.c_str());
    }
    // 发布 false：否则 rest_gateway 的 /readyz 会一直读到上一次的 true，
    // 直到它自己的 stale 阈值超时 —— 那是"网关侧再补一道"而非"上游说真话"。
    std_msgs::msg::Bool ready_msg;
    ready_msg.data = false;
    ready_pub_->publish(ready_msg);
  }

  void EvaluateReadiness(std::shared_ptr<amr_dispatcher_interfaces::srv::GetSystemReadiness::Response> resp) {
    // 安全状态新鲜度：safety_gate 停发（崩溃/被 SIGKILL）后 motion_allowed 会永远停在
    // 最后一次的值上。只看值不看时间，安全门死了以后就绪判定仍然为 true —— 这正是
    // "容器 Up 8 days、dispatcher 已死 8 天"那次事故在协调器侧的第二个真实缺陷。
    const bool safety_fresh =
        safety_seen_ &&
        (std::chrono::steady_clock::now() - safety_last_seen_) <=
            std::chrono::duration<double>(safety_stale_sec_);
    // dispatcher 可用性：state 读到 "unavailable" 或最后一次成功读取已超过
    // dispatcher_stale_sec_（例如本节点自己卡住过一阵，状态可信度存疑）。
    const bool dispatcher_fresh =
        dispatcher_seen_ && current_dispatcher_state_ != "unavailable" &&
        (std::chrono::steady_clock::now() - dispatcher_last_ok_) <=
            std::chrono::duration<double>(dispatcher_stale_sec_);
    const bool is_ready =
        dispatcher_fresh && (current_dispatcher_state_ == "active") && safety_fresh &&
        safety_healthy_;
    resp->ready = is_ready;
    // healthy 是"系统活着"的判据，供存活探针/告警用：dispatcher 不可用即不健康。
    // 但 dispatcher 只是"未激活"（还在提拉）时，系统本身仍健康，那由 ready 表达。
    resp->healthy = dispatcher_fresh && safety_fresh && safety_healthy_;

    if (is_ready) {
      resp->state = "READY";
      resp->message = "All critical nodes (Chassis, Safety, Dispatcher) are ACTIVE and HEALTHY";
      resp->active_nodes = {"chassis_driver", "safety_gate", "path_tracker", "dispatcher_node"};
    } else if (!dispatcher_fresh) {
      // 进程不在或状态读不到：门禁结论同样是"不放行"，但 message 必须说清
      // 是哪一种 —— 运维看到 "in state: unavailable" 才知道不是启动慢。
      resp->state = "INITIALIZING";
      resp->message = "DispatcherNode unavailable: " +
                      (dispatcher_seen_ ? dispatcher_unavailable_detail_
                                        : std::string("no state response yet"));
      resp->pending_nodes = {"dispatcher_node"};
    } else if (current_dispatcher_state_ != "active") {
      resp->state = "INITIALIZING";
      resp->message = "DispatcherNode is in state: " + current_dispatcher_state_;
      resp->pending_nodes = {"dispatcher_node"};
    } else if (!safety_fresh) {
      resp->state = "DEGRADED";
      resp->message = safety_seen_
                          ? "Safety Gate state is stale (no /safety/state within " +
                                std::to_string(safety_stale_sec_) + "s); treating as unhealthy"
                          : "Safety Gate has never reported /safety/state";
    } else {
      resp->state = "DEGRADED";
      resp->message = "Safety Gate or Chassis reporting fault/standstill";
    }
  }

  bool auto_bringup_{true};
  std::string current_dispatcher_state_{"unknown"};
  // dispatcher 存活/新鲜度：曾经读到过状态、最近一次成功读取的时刻。
  // 与 safety_* 同一套判据，避免"只记值不记时间"的同一类缺陷。
  bool dispatcher_seen_{false};
  std::chrono::steady_clock::time_point dispatcher_last_ok_{};
  std::string dispatcher_unavailable_detail_;
  double dispatcher_stale_sec_{3.0};
  bool safety_seen_{false};
  bool safety_healthy_{true};
  // 最近一次收到 /safety/state 的时刻（配合 safety_stale_sec_ 判新鲜度）
  std::chrono::steady_clock::time_point safety_last_seen_{};
  double safety_stale_sec_{2.0};
  std::shared_future<lifecycle_msgs::srv::GetState::Response::SharedPtr> pending_get_state_;
  int get_state_wait_ticks_ = 0;

  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr dispatcher_get_state_cli_;
  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr dispatcher_change_state_cli_;
  // 阶段 3：双订阅适配器。对外仍是 rosidl 回调，内部按 proto_dual_subscribe
  // 决定收 rosidl 话题还是 proto 话题（后者反投影后再回调）。
  amr_dispatcher_proto_bridge::ProtoDualSubscriber<
      amr_dispatcher_interfaces::msg::SafetyState,
      amr_dispatcher::proto::SafetyState>
      safety_sub_;

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
