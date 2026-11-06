#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "amr_dispatcher_core/chassis/link_quality.hpp"
#include "amr_dispatcher_interfaces/msg/chassis_link_health.hpp"
#include "amr_dispatcher_proto_bridge/proto_dual_subscriber.hpp"
#include "amr_dispatcher_ros/proto_dual_publisher.hpp"

using namespace std::chrono_literals;

namespace amr_dispatcher_ros {

class LinkHealthPublisherNode : public rclcpp::Node {
 public:
  explicit LinkHealthPublisherNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : rclcpp::Node("link_health_publisher_node", options) {
    declare_parameter("publish_rate_hz", 2.0);
    declare_parameter("watched_topics", std::vector<std::string>{"/chassis/link_health"});
    // 阶段 2 双发总开关（默认开）。关闭只停 proto 侧。
    declare_parameter("proto_dual_publish", true);
    // 阶段 3 双订阅总开关（默认关）。开启后 watched_topics 的每一项都改从
    // "<topic>_proto" 读取，适配器内部反投影回 rosidl 后交给同一段回调。
    //
    // 每项必须是**已解析的最终话题名**：发布端是 ResolveVehicleTopic 之后再
    // 追加 _proto 后缀，订阅端若传原始模板名就会派生出错误的话题名，
    // 表现为「订阅成功但永远收不到」。
    declare_parameter("proto_dual_subscribe", false);
    const bool proto_dual_publish = get_parameter("proto_dual_publish").as_bool();
    const bool proto_dual_subscribe = get_parameter("proto_dual_subscribe").as_bool();

    auto topics = get_parameter("watched_topics").as_string_array();
    subs_.reserve(topics.size());
    for (const auto& t : topics) {
      // 阶段 3：每项都可切到 proto 话题。用 make_unique 存放：适配器持有
      // 订阅句柄，放进 unique_ptr 后 vector 扩容不会搬动对象本身，
      // 订阅生命周期与 vector 同寿命。
      subs_.push_back(std::make_unique<amr_dispatcher_proto_bridge::ProtoDualSubscriber<
          amr_dispatcher_interfaces::msg::ChassisLinkHealth,
          amr_dispatcher::proto::ChassisLinkHealth>>(
          this, t, 10, proto_dual_subscribe,
          [this, t](const amr_dispatcher_interfaces::msg::ChassisLinkHealth::SharedPtr msg) {
            latest_health_[t] = *msg;
          }));
    }
    if (proto_dual_subscribe && !subs_.empty()) {
      RCLCPP_INFO(get_logger(),
                  "link health source: proto topics (*_proto), first='%s'",
                  subs_.front()->proto_topic().c_str());
    }

    aggregated_pub_ = create_publisher<amr_dispatcher_interfaces::msg::ChassisLinkHealth>(
        "/system/aggregated_link_health", 10);
    // 阶段 2 双发：/system/aggregated_link_health_proto。
    //
    // 注意这条话题**不是** per-vehicle 派生的（它是全局聚合视图，由本节点把
    // 多车健康合成一条），所以这里直接传字面量，不经 ResolveVehicleTopic。
    aggregated_proto_pub_ = std::make_unique<ProtoDualPublisher<
        amr_dispatcher_interfaces::msg::ChassisLinkHealth,
        amr_dispatcher::proto::ChassisLinkHealth>>(
        this, "/system/aggregated_link_health", 10, proto_dual_publish);
    if (aggregated_proto_pub_->enabled()) {
      RCLCPP_INFO(get_logger(), "proto dual-publish enabled: %s",
                  aggregated_proto_pub_->proto_topic().c_str());
    }

    const double rate = get_parameter("publish_rate_hz").as_double();
    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / rate),
        std::bind(&LinkHealthPublisherNode::PublishAggregated, this));

    RCLCPP_INFO(get_logger(), "LinkHealthPublisherNode started, monitoring %zu topics", topics.size());
  }

 private:
  void PublishAggregated() {
    if (latest_health_.empty()) return;

    amr_dispatcher_interfaces::msg::ChassisLinkHealth agg;
    agg.header.stamp = now();
    agg.backend_name = "aggregated";
    agg.is_healthy = true;

    double max_loss = 0.0;
    double max_lat = 0.0;
    double sum_avg_lat = 0.0;
    // T3：命令送达率取**最差**的一路（任一台车的命令在丢，都该被看见），
    // 只要有一路适用就适用；全部为 -1（文本协议/未回显）时保持 -1。
    double worst_delivery = -1.0;
    std::uint64_t sum_cmd_sent = 0;
    std::uint64_t sum_cmd_delivered = 0;

    for (const auto& [name, h] : latest_health_) {
      agg.frames_total += h.frames_total;
      agg.frames_success += h.frames_success;
      agg.frames_lost += h.frames_lost;
      agg.frames_crc_error += h.frames_crc_error;
      if (h.loss_rate > max_loss) max_loss = h.loss_rate;
      if (h.max_latency_ms > max_lat) max_lat = h.max_latency_ms;
      sum_avg_lat += h.avg_latency_ms;
      if (!h.is_healthy) agg.is_healthy = false;
      if (h.command_delivery_rate >= 0.0) {
        if (worst_delivery < 0.0 || h.command_delivery_rate < worst_delivery) {
          worst_delivery = h.command_delivery_rate;
        }
      }
      sum_cmd_sent += h.commands_sent;
      sum_cmd_delivered += h.commands_delivered;
    }
    agg.command_delivery_rate = worst_delivery;
    agg.commands_sent = sum_cmd_sent;
    agg.commands_delivered = sum_cmd_delivered;

    agg.loss_rate = agg.frames_total > 0
                        ? static_cast<double>(agg.frames_lost) / static_cast<double>(agg.frames_total)
                        : 0.0;
    agg.avg_latency_ms = sum_avg_lat / static_cast<double>(latest_health_.size());
    agg.max_latency_ms = max_lat;

    if (agg.loss_rate >= 0.3) {
      agg.tier = "CRITICAL";
    } else if (agg.loss_rate >= 0.15) {
      agg.tier = "DEGRADED";
    } else if (agg.is_healthy) {
      agg.tier = "HEALTHY";
    } else {
      agg.tier = "UNKNOWN";
    }

    aggregated_pub_->publish(agg);
    // 阶段 2 双发：从同一个 agg 投影，因此聚合逻辑（最差送达率、求和计数器等）
    // 只有一份实现，不会出现「rosidl 侧改了聚合口径、proto 侧没跟上」的漂移。
    if (aggregated_proto_pub_) {
      aggregated_proto_pub_->Publish(agg);
    }
  }

  // 阶段 3：双订阅容器。vector 可以容纳移动-only 元素，正好匹配
  // 「watched_topics 运行时长度不定」这个既有约束。
  std::vector<std::unique_ptr<amr_dispatcher_proto_bridge::ProtoDualSubscriber<
      amr_dispatcher_interfaces::msg::ChassisLinkHealth,
      amr_dispatcher::proto::ChassisLinkHealth>>> subs_;
  rclcpp::Publisher<amr_dispatcher_interfaces::msg::ChassisLinkHealth>::SharedPtr aggregated_pub_;
  std::unique_ptr<ProtoDualPublisher<
      amr_dispatcher_interfaces::msg::ChassisLinkHealth,
      amr_dispatcher::proto::ChassisLinkHealth>> aggregated_proto_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unordered_map<std::string, amr_dispatcher_interfaces::msg::ChassisLinkHealth> latest_health_;
};

}  // namespace amr_dispatcher_ros

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<amr_dispatcher_ros::LinkHealthPublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
