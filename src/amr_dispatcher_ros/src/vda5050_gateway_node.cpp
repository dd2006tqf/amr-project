#include "amr_dispatcher_ros/vda5050_gateway_node.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cctype>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <ctime>
#include <functional>
#include <iomanip>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <mqtt/async_client.h>
#include <mqtt/connect_options.h>
#include <mqtt/message.h>
#include <mqtt/will_options.h>

#include "amr_dispatcher_core/fleet/vda5050_serializer.hpp"

namespace amr_dispatcher_ros {

namespace {

// MQTT QoS 选型：state 高频走 0，链路状态与指令通道走 1（Spec §4.1 典型约定）
constexpr int kQosState = 0;
constexpr int kQosConnection = 1;
constexpr int kQosOrder = 1;
constexpr int kQosInstantActions = 1;

// 网关自用 connection 心跳/LWT 的 headerId（VDA 5050 要求单调递增）
std::atomic<uint64_t> g_connection_header_id{1};
// 简单指令映射为 instantActions 时的 actionId 计数
std::atomic<uint64_t> g_instant_action_seq{1};

// 与 core 序列化器一致的 ISO 8601 UTC 时间戳，例如 2026-10-20T12:00:00.000Z
std::string CurrentIsoTimestampUtc() {
  const auto now = std::chrono::system_clock::now();
  const auto millis =
      std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
  const std::time_t timer = std::chrono::system_clock::to_time_t(now);
  std::tm tm_buf{};
  gmtime_r(&timer, &tm_buf);
  std::ostringstream ss;
  ss << std::put_time(&tm_buf, "%Y-%m-%dT%H:%M:%S") << '.' << std::setfill('0') << std::setw(3)
     << millis.count() << 'Z';
  return ss.str();
}

std::string EscapeJsonString(const std::string& input) {
  std::string out;
  out.reserve(input.size() + 8);
  for (const char c : input) {
    if (c == '"') {
      out += "\\\"";
    } else if (c == '\\') {
      out += "\\\\";
    } else {
      out += c;
    }
  }
  return out;
}

std::string Trim(const std::string& s) {
  const auto not_space = [](unsigned char c) { return !std::isspace(c); };
  auto begin = std::find_if(s.begin(), s.end(), not_space);
  auto end = std::find_if(s.rbegin(), s.rend(), not_space).base();
  if (begin >= end) return {};
  return std::string(begin, end);
}

// 简单指令 -> VDA 5050 instantActions actionType 别名表
struct InstantActionAlias {
  const char* alias;
  const char* action_type;
};
constexpr InstantActionAlias kInstantActionAliases[] = {
    {"cancelOrder", "cancelOrder"}, {"cancel", "cancelOrder"}, {"pause", "pause"},
    {"resume", "resume"},            {"stopPause", "stopPause"},
};

}  // namespace

// ============================================================
// MqttClientWrapper：Eclipse Paho MQTT C++ 的最小封装。
// MQTT 类型与线程完全隔离在本 TU，ROS 层只见 topic/string。
//
// 线程安全设计（防死锁）：Paho 的 receiveThread 既处理 SUBACK 等 token 应答，
// 也派发消息回调。因此消息回调绝不能阻塞在任何被跨阻塞调用持有的锁上，
// 否则会出现「回调等锁 <-> wait() 等 SUBACK」死锁。故：
// - handler_mutex_ 只在拷贝/清空 handler_ 的瞬间持有（微秒级）；
// - client_ 只被执行器线程访问（Connect/Publish/Disconnect 均在执行器线程），无需加锁；
// - Disconnect 先清空 handler（回调变 no-op），再用 in_flight 计数器 + 条件变量
//   等待在途回调退出，最后销毁 client（防止析构期回调触碰已释放的节点成员）。
// ============================================================
class MqttClientWrapper {
 public:
  using MessageHandler = std::function<void(const std::string& topic, const std::string& payload)>;

  ~MqttClientWrapper() { Disconnect(); }

  MqttClientWrapper() = default;
  MqttClientWrapper(const MqttClientWrapper&) = delete;
  MqttClientWrapper& operator=(const MqttClientWrapper&) = delete;

  /// 建立连接并订阅主题。成功后启用 automatic_reconnect 自动重连；
  /// 失败返回 false 并销毁半成品客户端（由调用方定时重试重建）。
  bool Connect(const std::string& broker_uri, const std::string& client_id,
               const std::vector<std::string>& sub_filters, int sub_qos,
               const std::string& lwt_topic, const std::string& lwt_payload, int lwt_qos,
               MessageHandler handler, std::string* error) {
    {
      std::lock_guard<std::mutex> lock(handler_mutex_);
      handler_ = std::move(handler);
    }
    QuiesceCallbacks();  // 等待旧 client 的在途回调退出后再销毁
    DestroyClient();
    try {
      client_ = std::make_unique<mqtt::async_client>(broker_uri, client_id);
      client_->set_message_callback([this](mqtt::const_message_ptr msg) {
        in_flight_.fetch_add(1, std::memory_order_acq_rel);
        MessageHandler handler;
        {
          std::lock_guard<std::mutex> lock(handler_mutex_);
          handler = handler_;
        }
        if (handler && msg) {
          handler(msg->get_topic(), msg->to_string());
        }
        if (in_flight_.fetch_sub(1, std::memory_order_acq_rel) == 1) {
          std::lock_guard<std::mutex> lock(done_mutex_);
          done_cv_.notify_all();
        }
      });
      mqtt::connect_options_builder builder;
      builder.clean_session(true)
          .automatic_reconnect(std::chrono::seconds(1), std::chrono::seconds(60))
          .connect_timeout(std::chrono::seconds(5))
          .keep_alive_interval(std::chrono::seconds(15))
          .will(mqtt::will_options(lwt_topic, lwt_payload, lwt_qos, true /*retained*/));
      if (!client_->connect(builder.finalize())->wait_for(std::chrono::seconds(10))) {
        if (error != nullptr) *error = "连接超时";
        DestroyClient();
        return false;
      }
      for (const auto& filter : sub_filters) {
        if (!client_->subscribe(filter, sub_qos)->wait_for(std::chrono::seconds(5))) {
          if (error != nullptr) *error = "订阅超时: " + filter;
          DestroyClient();
          return false;
        }
      }
      connected_once_.store(true);
      return true;
    } catch (const std::exception& e) {
      if (error != nullptr) *error = e.what();
      DestroyClient();
      return false;
    }
  }

  /// 发布消息。未连接时快速失败（返回 false），绝不阻塞执行器线程。
  /// 仅允许在执行器线程调用（与 Connect/Disconnect 串行）。
  bool Publish(const std::string& topic, const std::string& payload, int qos, bool retained,
               std::string* error) {
    if (client_ == nullptr) {
      if (error != nullptr) *error = "MQTT 客户端未初始化";
      return false;
    }
    if (!client_->is_connected()) {
      if (error != nullptr) *error = "MQTT 未连接";
      return false;
    }
    try {
      client_->publish(mqtt::make_message(topic, payload.data(), payload.size(), qos, retained));
      return true;
    } catch (const std::exception& e) {
      if (error != nullptr) *error = e.what();
      return false;
    }
  }

  bool HasConnectedOnce() const { return connected_once_.load(); }

  /// 优雅断连：不触发 LWT 遗嘱（正常下线不应广播 CONNECTIONBROKEN）。
  void Disconnect() {
    {
      std::lock_guard<std::mutex> lock(handler_mutex_);
      handler_ = nullptr;  // 回调屏障：此后的回调拷贝到空 handler 直接 no-op
    }
    QuiesceCallbacks();
    DestroyClient();
  }

 private:
  /// 等待在途消息回调退出（带 1s 上限，防御异常路径挂死）。
  void QuiesceCallbacks() {
    std::unique_lock<std::mutex> lock(done_mutex_);
    done_cv_.wait_for(lock, std::chrono::seconds(1),
                      [this] { return in_flight_.load(std::memory_order_acquire) == 0; });
  }

  /// 仅允许执行器线程调用；调用前须确保无在途回调（见 QuiesceCallbacks）。
  void DestroyClient() {
    if (client_ == nullptr) return;
    try {
      if (client_->is_connected()) {
        client_->disconnect()->wait_for(std::chrono::seconds(2));
      }
    } catch (...) {
      // 异常路径：连接可能已断，忽略即可
    }
    client_.reset();
  }

  // 只保护 handler_ 的拷贝/清空，绝不在阻塞 MQTT 调用期间持有
  std::mutex handler_mutex_;
  MessageHandler handler_;
  std::unique_ptr<mqtt::async_client> client_;

  // 在途回调计数（回调线程 <-> Disconnect 屏障）
  std::atomic<int> in_flight_{0};
  std::mutex done_mutex_;
  std::condition_variable done_cv_;

  std::atomic<bool> connected_once_{false};
};

// ============================================================
// Vda5050GatewayNode
// ============================================================
Vda5050GatewayNode::Vda5050GatewayNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("vda5050_gateway_node", options) {
  declare_parameter("use_vda5050", false);
  declare_parameter("mqtt_broker_host", "127.0.0.1");
  declare_parameter("mqtt_port", 1883);
  declare_parameter("manufacturer", "RobotManufacturer");
  declare_parameter("serial_number", "default");
  declare_parameter("protocol_version", "v2");
  declare_parameter("heartbeat_ms", 5000);

  use_vda5050_ = get_parameter("use_vda5050").as_bool();
  mqtt_broker_host_ = get_parameter("mqtt_broker_host").as_string();
  mqtt_port_ = static_cast<int>(get_parameter("mqtt_port").as_int());
  manufacturer_ = get_parameter("manufacturer").as_string();
  serial_number_ = get_parameter("serial_number").as_string();
  protocol_version_ = get_parameter("protocol_version").as_string();
  heartbeat_ms_ = static_cast<int>(get_parameter("heartbeat_ms").as_int());

  if (!use_vda5050_) {
    RCLCPP_INFO(get_logger(), "VDA 5050 网关未启用 (use_vda5050=false)，节点保持空转 no-op");
    return;
  }

  if (protocol_version_ != "v2" && protocol_version_ != "v3") {
    RCLCPP_WARN(get_logger(), "未知 protocol_version '%s'，按 v2 处理", protocol_version_.c_str());
    protocol_version_ = "v2";
  }
  if (mqtt_port_ <= 0 || mqtt_port_ > 65535) {
    RCLCPP_WARN(get_logger(), "非法 mqtt_port=%d，回退 1883", mqtt_port_);
    mqtt_port_ = 1883;
  }
  if (manufacturer_.empty()) {
    RCLCPP_WARN(get_logger(), "manufacturer 为空，回退 RobotManufacturer");
    manufacturer_ = "RobotManufacturer";
  }
  if (serial_number_.empty()) {
    RCLCPP_WARN(get_logger(), "serial_number 为空，回退 default");
    serial_number_ = "default";
  }
  if (heartbeat_ms_ <= 0) {
    RCLCPP_WARN(get_logger(), "非法 heartbeat_ms=%d，回退 5000", heartbeat_ms_);
    heartbeat_ms_ = 5000;
  }

  RCLCPP_INFO(get_logger(),
              "VDA 5050 网关启用: broker=%s:%d base=%s gateway_identity=%s/%s heartbeat=%dms",
              mqtt_broker_host_.c_str(), mqtt_port_, TopicBase().c_str(), manufacturer_.c_str(),
              serial_number_.c_str(), heartbeat_ms_);

  try {
    mqtt_ = std::make_unique<MqttClientWrapper>();
    drain_timer_ = create_wall_timer(std::chrono::milliseconds(kDrainPeriodMs),
                                     std::bind(&Vda5050GatewayNode::DrainInboundQueue, this));
    heartbeat_timer_ = create_wall_timer(
        std::chrono::milliseconds(heartbeat_ms_),
        std::bind(&Vda5050GatewayNode::PublishGatewayHeartbeat, this));
    mqtt_retry_timer_ = create_wall_timer(
        std::chrono::milliseconds(kMqttRetryPeriodMs),
        std::bind(&Vda5050GatewayNode::TryConnectMqtt, this));
    TryConnectMqtt();
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "MQTT 子系统初始化失败，网关降级为空转: %s", e.what());
    if (drain_timer_) drain_timer_->cancel();
    if (heartbeat_timer_) heartbeat_timer_->cancel();
    if (mqtt_retry_timer_) mqtt_retry_timer_->cancel();
    mqtt_.reset();
  }
}

Vda5050GatewayNode::~Vda5050GatewayNode() {
  if (drain_timer_) drain_timer_->cancel();
  if (heartbeat_timer_) heartbeat_timer_->cancel();
  if (mqtt_retry_timer_) mqtt_retry_timer_->cancel();
  // 优雅断连：正常关机不触发 LWT 遗嘱
  if (mqtt_) mqtt_->Disconnect();
}

void Vda5050GatewayNode::TryConnectMqtt() {
  if (!mqtt_ || mqtt_->HasConnectedOnce()) {
    return;  // 首连成功后由 automatic_reconnect 接管，无需人工重试
  }
  const std::string uri = "tcp://" + mqtt_broker_host_ + ":" + std::to_string(mqtt_port_);
  const std::string client_id = "amr_dispatcher_gateway_" + serial_number_;
  const std::vector<std::string> filters = {TopicBase() + "/+/+/state",
                                            TopicBase() + "/+/+/connection"};
  const std::string lwt_topic = MqttTopic(manufacturer_, serial_number_, "connection");
  const std::string lwt_payload = BuildGatewayConnectionJson("CONNECTIONBROKEN");

  std::string err;
  const bool ok = mqtt_->Connect(
      uri, client_id, filters, kQosConnection, lwt_topic, lwt_payload, kQosConnection,
      [this](const std::string& topic, const std::string& payload) {
        OnMqttMessage(topic, payload);
      },
      &err);
  if (ok) {
    RCLCPP_INFO(get_logger(), "MQTT 已连接 %s (client_id=%s, base=%s, LWT=%s)", uri.c_str(),
                client_id.c_str(), TopicBase().c_str(), lwt_topic.c_str());
  } else {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                         "MQTT 连接失败 %s: %s（首连前每 5s 重试，首连成功后由自动重连接管）",
                         uri.c_str(), err.c_str());
  }
}

void Vda5050GatewayNode::OnMqttMessage(const std::string& topic, std::string payload) {
  std::lock_guard<std::mutex> lock(inbound_mutex_);
  if (inbound_queue_.size() >= kMaxInboundQueue) {
    inbound_queue_.pop_front();
    dropped_messages_.fetch_add(1, std::memory_order_relaxed);
  }
  inbound_queue_.emplace_back(topic, std::move(payload));
}

void Vda5050GatewayNode::DrainInboundQueue() {
  std::deque<std::pair<std::string, std::string>> batch;
  {
    std::lock_guard<std::mutex> lock(inbound_mutex_);
    batch.swap(inbound_queue_);
  }
  const std::size_t dropped = dropped_messages_.exchange(0, std::memory_order_relaxed);
  if (dropped > 0) {
    RCLCPP_WARN(get_logger(),
                "入站队列溢出，丢弃 %zu 条 MQTT 消息（可调低负载或提高出队频率 kDrainPeriodMs）",
                dropped);
  }
  for (const auto& item : batch) {
    const InboundTopic parsed = ParseInboundTopic(item.first);
    if (!parsed.valid) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "忽略无法解析的 MQTT Topic: %s",
                           item.first.c_str());
      continue;
    }
    // 网关自身 connection 心跳/LWT 不回环进 ROS 侧
    if (parsed.vehicle_id == serial_number_) {
      continue;
    }
    VehicleEndpoints& ep = EnsureVehicle(parsed.vehicle_id, parsed.manufacturer);
    if (parsed.leaf == "state") {
      HandleStateMessage(parsed.vehicle_id, item.second, ep);
    } else {
      HandleConnectionMessage(parsed.vehicle_id, item.second, ep);
    }
  }
}

void Vda5050GatewayNode::HandleStateMessage(const std::string& vehicle_id,
                                            const std::string& payload, VehicleEndpoints& ep) {
  if (!ep.state_pub) {
    return;  // 非法 vehicle_id 已在 EnsureVehicle 告警，该车无 ROS 端点
  }
  const auto report = amr_dispatcher_core::fleet::DeserializeVda5050State(payload);
  if (!report.has_value()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "丢弃非法 VDA 5050 state 报文 (vehicle=%s)", vehicle_id.c_str());
    return;
  }
  // 无损透传原始 JSON，Task 5 调度侧可用 DeserializeVda5050State 按需解析
  std_msgs::msg::String raw;
  raw.data = payload;
  ep.state_pub->publish(raw);

  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = now();
  pose.header.frame_id = "map";
  pose.pose.position.x = report->x;
  pose.pose.position.y = report->y;
  pose.pose.orientation.z = std::sin(report->yaw * 0.5);
  pose.pose.orientation.w = std::cos(report->yaw * 0.5);
  ep.pose_pub->publish(pose);

  if (!ep.online) {
    ep.online = true;
    std_msgs::msg::Bool online_msg;
    online_msg.data = true;
    ep.online_pub->publish(online_msg);
  }
}

void Vda5050GatewayNode::HandleConnectionMessage(const std::string& vehicle_id,
                                                 const std::string& payload,
                                                 VehicleEndpoints& ep) {
  const auto connection = amr_dispatcher_core::fleet::DeserializeVda5050Connection(payload);
  if (!connection.has_value()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "丢弃非法 VDA 5050 connection 报文 (vehicle=%s)", vehicle_id.c_str());
    return;
  }
  if (*connection == "CONNECTIONBROKEN") {
    // 安全铁律（Spec §3 / §5.2）：网关绝不释放任何路权锁——
    // 仅将 /fleet/{vid}/online 置 false；锁保留与逐段释放由调度核心 (Task 5) 负责。
    RCLCPP_WARN(get_logger(),
                "车辆 %s MQTT 连接断开 (LWT CONNECTIONBROKEN)：仅发布 /fleet/%s/online=false，"
                "绝不释放该车持有的路权锁（锁保留归调度核心 Task 5）",
                vehicle_id.c_str(), vehicle_id.c_str());
  }
  if (!ep.online_pub) {
    return;  // 非法 vehicle_id 无 ROS 端点
  }
  const bool online = (*connection == "ONLINE");
  ep.online = online;
  std_msgs::msg::Bool online_msg;
  online_msg.data = online;
  ep.online_pub->publish(online_msg);
}

void Vda5050GatewayNode::SendOrderCallback(const std::string& vehicle_id,
                                           const std::string& manufacturer,
                                           const std_msgs::msg::String::SharedPtr& msg) {
  const std::string payload = Trim(msg->data);
  if (payload.empty()) {
    RCLCPP_WARN(get_logger(), "send_order 收到空报文 (vehicle=%s)，忽略", vehicle_id.c_str());
    return;
  }
  if (payload.front() != '{') {
    RCLCPP_WARN(get_logger(),
                "send_order 期望 Vda5050OrderBuilder 序列化的 Order JSON，实际开头 '%c' (vehicle=%s)，忽略",
                payload.front(), vehicle_id.c_str());
    return;
  }
  const std::string topic = MqttTopic(manufacturer, vehicle_id, "order");
  std::string err;
  if (mqtt_ && mqtt_->Publish(topic, payload, kQosOrder, false, &err)) {
    RCLCPP_INFO(get_logger(), "Order 已下发 MQTT: %s (%zu bytes)", topic.c_str(), payload.size());
  } else {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Order 下发失败 (topic=%s): %s",
                         topic.c_str(),
                         (mqtt_ == nullptr) ? "MQTT 子系统未启用" : err.c_str());
  }
}

void Vda5050GatewayNode::InstantActionsCallback(const std::string& vehicle_id,
                                                const std::string& manufacturer,
                                                const std_msgs::msg::String::SharedPtr& msg) {
  const std::string payload = Trim(msg->data);
  if (payload.empty()) {
    RCLCPP_WARN(get_logger(), "instant_actions 收到空报文 (vehicle=%s)，忽略", vehicle_id.c_str());
    return;
  }

  std::string json;
  if (payload.front() == '{') {
    json = payload;  // 已是完整 VDA 5050 instantActions JSON，原样透传
  } else {
    const char* canonical = nullptr;
    for (const auto& alias : kInstantActionAliases) {
      if (payload == alias.alias) {
        canonical = alias.action_type;
        break;
      }
    }
    if (canonical == nullptr) {
      RCLCPP_WARN(get_logger(),
                  "instant_actions 不支持的简单指令 '%s' (vehicle=%s)，"
                  "支持: cancelOrder/pause/resume/stopPause，或完整 instantActions JSON",
                  payload.c_str(), vehicle_id.c_str());
      return;
    }
    amr_dispatcher_core::fleet::VehicleAction action;
    action.action_type = canonical;
    action.action_id = "ia_" + std::to_string(g_instant_action_seq.fetch_add(1));
    std::vector<amr_dispatcher_core::fleet::VehicleAction> actions{std::move(action)};
    json = amr_dispatcher_core::fleet::SerializeVda5050InstantActions(vehicle_id, actions,
                                                                     protocol_version_, manufacturer);
  }

  const std::string topic = MqttTopic(manufacturer, vehicle_id, "instantActions");
  std::string err;
  if (mqtt_ && mqtt_->Publish(topic, json, kQosInstantActions, false, &err)) {
    RCLCPP_INFO(get_logger(), "InstantActions 已下发 MQTT: %s (%zu bytes)", topic.c_str(),
                json.size());
  } else {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "InstantActions 下发失败 (topic=%s): %s",
                         topic.c_str(),
                         (mqtt_ == nullptr) ? "MQTT 子系统未启用" : err.c_str());
  }
}

void Vda5050GatewayNode::PublishGatewayHeartbeat() {
  if (!mqtt_) return;
  const std::string topic = MqttTopic(manufacturer_, serial_number_, "connection");
  const std::string json = BuildGatewayConnectionJson("ONLINE");
  std::string err;
  if (!mqtt_->Publish(topic, json, kQosConnection, true /*retained*/, &err)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                         "网关 connection 心跳发布失败 (topic=%s): %s", topic.c_str(), err.c_str());
  }
}

Vda5050GatewayNode::VehicleEndpoints& Vda5050GatewayNode::EnsureVehicle(
    const std::string& vehicle_id, const std::string& manufacturer) {
  auto it = vehicles_.find(vehicle_id);
  if (it != vehicles_.end()) {
    return it->second;
  }
  VehicleEndpoints ep;
  ep.manufacturer = manufacturer;  // manufacturer 以首见为准，保证后续原路回发
  if (!IsValidVehicleId(vehicle_id)) {
    RCLCPP_ERROR(get_logger(),
                 "serialNumber '%s' 不是合法 ROS 话题段（须字母或 '_' 开头，仅含字母/数字/'_'），"
                 "跳过该车全部 ROS 端点创建（MQTT 订阅解析不受影响）",
                 vehicle_id.c_str());
  } else {
    try {
      const std::string base = "/fleet/" + vehicle_id;
      ep.state_pub = create_publisher<std_msgs::msg::String>(base + "/state", 10);
      ep.pose_pub = create_publisher<geometry_msgs::msg::PoseStamped>(base + "/pose", 10);
      ep.online_pub = create_publisher<std_msgs::msg::Bool>(
          base + "/online", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());
      ep.send_order_sub = create_subscription<std_msgs::msg::String>(
          base + "/send_order", 10,
          [this, vehicle_id, manufacturer](const std_msgs::msg::String::SharedPtr m) {
            SendOrderCallback(vehicle_id, manufacturer, m);
          });
      ep.instant_actions_sub = create_subscription<std_msgs::msg::String>(
          base + "/instant_actions", 10,
          [this, vehicle_id, manufacturer](const std_msgs::msg::String::SharedPtr m) {
            InstantActionsCallback(vehicle_id, manufacturer, m);
          });
      RCLCPP_INFO(get_logger(),
                  "发现车辆 %s (manufacturer=%s)，已创建 ROS 端点 %s/{state,pose,online,send_order,instant_actions}",
                  vehicle_id.c_str(), manufacturer.c_str(), base.c_str());
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "创建车辆 %s 的 ROS 端点失败: %s", vehicle_id.c_str(), e.what());
      ep = VehicleEndpoints{};  // 部分创建失败则整体放弃，避免半残端点
      ep.manufacturer = manufacturer;
    }
  }
  return vehicles_.emplace(vehicle_id, std::move(ep)).first->second;
}

Vda5050GatewayNode::InboundTopic Vda5050GatewayNode::ParseInboundTopic(const std::string& topic) {
  InboundTopic out;
  std::vector<std::string> parts;
  std::size_t start = 0;
  while (start <= topic.size()) {
    const std::size_t pos = topic.find('/', start);
    if (pos == std::string::npos) {
      parts.push_back(topic.substr(start));
      break;
    }
    parts.push_back(topic.substr(start, pos - start));
    start = pos + 1;
  }
  if (parts.size() != 5) {
    return out;  // base/manufacturer/serial/leaf 才是 4 段；此处 topic 含首段，共 5
  }
  if (parts[4] != "state" && parts[4] != "connection") {
    return out;
  }
  if (parts[2].empty() || parts[3].empty()) {
    return out;
  }
  out.valid = true;
  out.manufacturer = parts[2];
  out.vehicle_id = parts[3];
  out.leaf = parts[4];
  return out;
}

bool Vda5050GatewayNode::IsValidVehicleId(const std::string& vehicle_id) {
  if (vehicle_id.empty()) return false;
  const auto ok_first = [](char c) {
    return std::isalpha(static_cast<unsigned char>(c)) != 0 || c == '_';
  };
  if (!ok_first(vehicle_id[0])) return false;
  for (const char c : vehicle_id) {
    if (std::isalnum(static_cast<unsigned char>(c)) == 0 && c != '_') return false;
  }
  return true;
}

std::string Vda5050GatewayNode::TopicBase() const {
  return (protocol_version_ == "v3") ? "vda5050/v3" : "uagv/v2";
}

std::string Vda5050GatewayNode::MqttTopic(const std::string& manufacturer,
                                          const std::string& vehicle_id,
                                          const std::string& leaf) const {
  return TopicBase() + "/" + manufacturer + "/" + vehicle_id + "/" + leaf;
}

std::string Vda5050GatewayNode::BuildGatewayConnectionJson(const std::string& state) const {
  const std::string vda_version = (protocol_version_ == "v3") ? "3.0.0" : "2.0.0";
  std::ostringstream ss;
  ss << "{\"headerId\":" << g_connection_header_id.fetch_add(1, std::memory_order_relaxed)
     << ",\"timestamp\":\"" << CurrentIsoTimestampUtc() << "\","
     << "\"version\":\"" << vda_version << "\","
     << "\"manufacturer\":\"" << EscapeJsonString(manufacturer_) << "\","
     << "\"serialNumber\":\"" << EscapeJsonString(serial_number_) << "\","
     << "\"connectionState\":\"" << EscapeJsonString(state) << "\"}";
  return ss.str();
}

}  // namespace amr_dispatcher_ros

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<amr_dispatcher_ros::Vda5050GatewayNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
