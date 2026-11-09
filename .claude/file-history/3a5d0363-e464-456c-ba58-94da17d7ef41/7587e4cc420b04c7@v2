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
#include "amr_dispatcher_ros/proto_dual_publisher.hpp"
// 阶段 3：链路健康与拓扑状态的双订阅（proto 分支内部反投影回 rosidl）。
#include "amr_dispatcher_proto_bridge/proto_dual_subscriber.hpp"

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
  // P0.6：全局急停/碰撞订阅（per-vehicle 模式下与本车话题并存，全局始终被尊重）
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr global_estop_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr global_bumper_sub_;
  // B1 心跳分源订阅（不再共用单话题）：每条只 Feed 自己的源，避免一侧挂掉拉低另一侧。
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr heartbeat_chassis_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr heartbeat_path_tracker_sub_;
  // 阶段 3：这两条话题支持双订阅（proto 分支内部反投影回 rosidl，
  // 回调与下面所有判定逻辑一字未改）。
  amr_dispatcher_proto_bridge::ProtoDualSubscriber<
      amr_dispatcher_interfaces::msg::ChassisLinkHealth,
      amr_dispatcher::proto::ChassisLinkHealth>
      link_health_sub_;
  amr_dispatcher_proto_bridge::ProtoDualSubscriber<
      amr_dispatcher_interfaces::msg::TopologyState,
      amr_dispatcher::proto::TopologyState>
      topology_state_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr safe_cmd_vel_pub_;
  // 阶段 4b：原先这里还有 safety_state_pub_（节点自建的 rosidl 发布器）。
  // 已删除并收进 safety_state_proto_pub_——留着它就会有两个 rosidl 发布器
  // 指向 /safety/state，而「只发 proto」的开关也就随之失效。
  //
  // 阶段 2 双发：/safety/state_proto，rosidl 侧同样由双发器持有。
  //
  // 安全消息迁移的双发**绝不能改变安全语义**：proto 侧是 rosidl 侧的纯副本，
  // 不参与任何决策。safety_gate 的所有判断仍只读 rosidl 话题（含 self-consume
  // 的链路健康、拓扑死锁），因此即使 proto 侧发生任何异常（序列化失败、无人订阅），
  // 门控行为逐位不变。这一点在阶段 3 消费者迁移之前必须保持。
  std::unique_ptr<ProtoDualPublisher<
      amr_dispatcher_interfaces::msg::SafetyState,
      amr_dispatcher::proto::SafetyState>> safety_state_proto_pub_;

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
  // T9：看门狗降级（丢拍但未到软超时）。只限速，不急停——与 watchdog_ok_
  // （硬超时/从未收到心跳，判不可动）分开，否则 soft_timeout_ms 名存实亡。
  bool watchdog_degraded_ = false;
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
  // M4：构造期缓存的参数（10Hz/50Hz 周期路径不再重复 get_parameter）
  std::chrono::milliseconds heartbeat_timeout_{500};
  // M5：降级限速线速度阈值（kClampSpeed 时生效，可按车型配置）
  double degraded_max_linear_x_mps_ = 0.2;

  // ---- P0.6 / A4：per-vehicle 话题隔离 ----
  // vehicle_id 空 = 单车兼容模式（话题名逐字不变）；非空 = 按 {vid} 派生。
  std::string vehicle_id_;
  // 已解析后的最终话题名（启动期一次性定稿并打印）
  std::string raw_cmd_vel_topic_{"/cmd_vel_raw"};
  std::string safe_cmd_vel_topic_{"/cmd_vel_safe"};
  std::string teleop_cmd_vel_topic_{"/cmd_vel_teleop"};
  std::string estop_topic_{"/safety/estop"};
  std::string bumper_topic_{"/safety/bumper"};
  std::string manual_takeover_topic_{"/safety/manual_takeover"};
  std::string chassis_heartbeat_topic_{"/safety/heartbeat/chassis_driver"};
  std::string path_tracker_heartbeat_topic_{"/safety/heartbeat/path_tracker"};
  std::string link_health_topic_{"/chassis/link_health"};
  std::string safety_state_topic_{"/safety/state"};
  // P0.6：全局急停/碰撞话题（per-vehicle 模式下与本车话题一并订阅）
  bool subscribe_global_estop_ = true;
  bool subscribe_global_bumper_ = true;

  // 阶段 2：proto 双发总开关（参数 proto_dual_publish）
  bool proto_dual_publish_{true};
  // 阶段 4b：rosidl 侧开关（参数 proto_rosidl_publish，默认开）。
  // false = 只发 /safety/state_proto，DDS 里不再有 /safety/state。
  bool proto_rosidl_publish_{true};
  bool proto_dual_subscribe_{false};
};

}  // namespace amr_dispatcher_ros
