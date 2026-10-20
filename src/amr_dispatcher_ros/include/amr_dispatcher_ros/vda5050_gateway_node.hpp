#pragma once

#include <atomic>
#include <cstddef>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

namespace amr_dispatcher_ros {

// Eclipse Paho MQTT C++ 客户端封装（pimpl）：完整定义只在 .cpp 中，
// MQTT 类型不进入本头文件，核心库 amr_dispatcher_core 保持零 MQTT 依赖（Spec §1 铁律）。
class MqttClientWrapper;

/// ROS 2 VDA 5050 MQTT 通信适配网关（薄壳，P2）。
///
/// 职责边界（Spec §3 铁律）：本节点只做 VDA 5050 协议 <-> ROS 2 话题 的转换与转发，
/// 绝不触碰任何路权锁——在线/离线只翻转 /fleet/{vehicle_id}/online。
/// 锁的保留（断链时）与逐段释放（lastNodeId 推进时）属于调度核心/Task 5 的
/// dispatcher_node（Spec §5.2 断链锁定保护 / §5.3 逐段释放）。
///
/// MQTT 侧 Topic（v2 默认 base: uagv/v2，v3: vda5050/v3，经 protocol_version 配置）：
///   SUB state       <- {base}/{manufacturer}/{serialNumber}/state
///   SUB connection  <- {base}/{manufacturer}/{serialNumber}/connection  (LWT 遗嘱)
///   PUB order       -> {base}/{manufacturer}/{serialNumber}/order
///   PUB instantActions -> .../instantActions
///   PUB connection  -> {gateway 自身 .../connection} 心跳 ONLINE + LWT CONNECTIONBROKEN
///
/// ROS 2 侧话题（{vehicle_id} 即 VDA 5050 serialNumber，惰性创建）：
///   PUB /fleet/{vid}/state          std_msgs/String    原始 VDA 5050 state JSON（无损透传）
///   PUB /fleet/{vid}/pose           geometry_msgs/PoseStamped (frame_id=map, 由 x/y/theta 换算)
///   PUB /fleet/{vid}/online         std_msgs/Bool      transient_local（晚加入者可读）
///   SUB /fleet/{vid}/send_order     std_msgs/String    期望 Vda5050OrderBuilder 产出的 Order JSON
///   SUB /fleet/{vid}/instant_actions std_msgs/String   完整 JSON 原样透传，或简单指令
///                                                       cancelOrder/pause/resume/stopPause
///
/// vehicle_id 约定：即 VDA 5050 serialNumber，且必须是合法 ROS 话题段
/// （字母或 '_' 开头，仅含字母/数字/'_'），否则跳过该车 ROS 端点创建并告警
/// （MQTT 订阅解析不受影响，仅该车不出 ROS 话题）。
///
/// 线程模型：Paho 回调跑在 MQTT 内部线程，只把消息推入互斥锁保护的入站队列；
/// 50ms drain 定时器在执行器线程消费队列并创建/发布全部 ROS 实体。
class Vda5050GatewayNode : public rclcpp::Node {
 public:
  explicit Vda5050GatewayNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~Vda5050GatewayNode() override;

 private:
  /// 单台车的 ROS 端点集合（只在执行器线程创建与访问）。
  struct VehicleEndpoints {
    // 从入站 MQTT Topic 记忆的 manufacturer，用于原路回发（保证多厂商车队正确寻址）
    std::string manufacturer;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr online_pub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr send_order_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr instant_actions_sub;
    bool online = false;  // 最近一次连接/状态推断的在线标记
  };

  /// 入站 MQTT Topic 解析结果：{base}/{manufacturer}/{serialNumber}/{leaf}
  struct InboundTopic {
    bool valid = false;
    std::string manufacturer;
    std::string vehicle_id;  // serialNumber
    std::string leaf;        // "state" | "connection"
  };

  // ---- MQTT -> ROS（Paho 回调线程只入队，drain 定时器在执行器线程消费） ----
  void OnMqttMessage(const std::string& topic, std::string payload);
  void DrainInboundQueue();
  void HandleStateMessage(const std::string& vehicle_id, const std::string& payload,
                          VehicleEndpoints& ep);
  void HandleConnectionMessage(const std::string& vehicle_id, const std::string& payload,
                               VehicleEndpoints& ep);

  // ---- ROS -> MQTT（执行器线程） ----
  void SendOrderCallback(const std::string& vehicle_id, const std::string& manufacturer,
                         const std_msgs::msg::String::SharedPtr& msg);
  void InstantActionsCallback(const std::string& vehicle_id, const std::string& manufacturer,
                              const std_msgs::msg::String::SharedPtr& msg);
  void PublishGatewayHeartbeat();
  void TryConnectMqtt();

  // ---- 辅助 ----
  /// 首次发现车辆时于执行器线程内创建其全部 ROS 端点（非法 vehicle_id 只告警一次）。
  VehicleEndpoints& EnsureVehicle(const std::string& vehicle_id, const std::string& manufacturer);
  static InboundTopic ParseInboundTopic(const std::string& topic);
  static bool IsValidVehicleId(const std::string& vehicle_id);
  /// MQTT Topic base：v3 -> "vda5050/v3"，其余按 v2 -> "uagv/v2"。
  std::string TopicBase() const;
  std::string MqttTopic(const std::string& manufacturer, const std::string& vehicle_id,
                        const std::string& leaf) const;
  /// 网关自身 connection 心跳 / LWT 遗嘱 JSON。
  std::string BuildGatewayConnectionJson(const std::string& state) const;

  // ---- 参数 ----
  bool use_vda5050_ = false;
  std::string mqtt_broker_host_ = "127.0.0.1";
  int mqtt_port_ = 1883;
  std::string manufacturer_ = "RobotManufacturer";
  std::string serial_number_ = "default";
  std::string protocol_version_ = "v2";
  int heartbeat_ms_ = 5000;

  // ---- MQTT 客户端（pimpl，仅 .cpp 可见 Paho 类型） ----
  std::unique_ptr<MqttClientWrapper> mqtt_;

  // ---- 入站队列（MQTT 线程 <-> 执行器线程） ----
  static constexpr std::size_t kMaxInboundQueue = 2048;
  static constexpr int kDrainPeriodMs = 50;         // 20Hz 出队，state 1~5Hz 足够
  static constexpr int kMqttRetryPeriodMs = 5000;   // 首连失败后 5s 重试间隔
  std::mutex inbound_mutex_;
  std::deque<std::pair<std::string, std::string>> inbound_queue_;
  std::atomic<std::size_t> dropped_messages_{0};

  // ---- 每车端点（仅执行器线程访问） ----
  std::unordered_map<std::string, VehicleEndpoints> vehicles_;

  // ---- 定时器 ----
  rclcpp::TimerBase::SharedPtr drain_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  rclcpp::TimerBase::SharedPtr mqtt_retry_timer_;
};

}  // namespace amr_dispatcher_ros
