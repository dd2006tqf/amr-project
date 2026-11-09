#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "amr_dispatcher_core/chassis/link_quality.hpp"
#include "amr_dispatcher_interfaces/msg/chassis_link_health.hpp"

using namespace std::chrono_literals;

namespace amr_dispatcher_ros {

class LinkHealthPublisherNode : public rclcpp::Node {
 public:
  explicit LinkHealthPublisherNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : rclcpp::Node("link_health_publisher_node", options) {
    declare_parameter("publish_rate_hz", 2.0);
    declare_parameter("watched_topics", std::vector<std::string>{"/chassis/link_health"});

    auto topics = get_parameter("watched_topics").as_string_array();
    for (const auto& t : topics) {
      auto sub = create_subscription<amr_dispatcher_interfaces::msg::ChassisLinkHealth>(
          t, 10,
          [this, t](const amr_dispatcher_interfaces::msg::ChassisLinkHealth::SharedPtr msg) {
            latest_health_[t] = *msg;
          });
      subs_.push_back(sub);
    }

    aggregated_pub_ = create_publisher<amr_dispatcher_interfaces::msg::ChassisLinkHealth>(
        "/system/aggregated_link_health", 10);

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

    for (const auto& [name, h] : latest_health_) {
      agg.frames_total += h.frames_total;
      agg.frames_success += h.frames_success;
      agg.frames_lost += h.frames_lost;
      agg.frames_crc_error += h.frames_crc_error;
      if (h.loss_rate > max_loss) max_loss = h.loss_rate;
      if (h.max_latency_ms > max_lat) max_lat = h.max_latency_ms;
      sum_avg_lat += h.avg_latency_ms;
      if (!h.is_healthy) agg.is_healthy = false;
    }

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
  }

  std::vector<rclcpp::Subscription<amr_dispatcher_interfaces::msg::ChassisLinkHealth>::SharedPtr> subs_;
  rclcpp::Publisher<amr_dispatcher_interfaces::msg::ChassisLinkHealth>::SharedPtr aggregated_pub_;
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
