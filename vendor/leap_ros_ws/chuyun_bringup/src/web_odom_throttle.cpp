#include <algorithm>
#include <chrono>
#include <memory>
#include <utility>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

class WebOdomThrottle : public rclcpp::Node
{
public:
  WebOdomThrottle()
  : Node("web_odom_throttle")
  {
    const double publish_rate = declare_parameter<double>("publish_rate", 10.0);
    const auto best_effort_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

    publisher_ = create_publisher<nav_msgs::msg::Odometry>(
      "/web/odometry/filtered", best_effort_qos);
    subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/filtered", best_effort_qos,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {
        latest_ = std::move(message);
      });

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() {
        if (latest_ && publisher_->get_subscription_count() > 0U) {
          publisher_->publish(*latest_);
        }
      });
  }

private:
  nav_msgs::msg::Odometry::ConstSharedPtr latest_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WebOdomThrottle>());
  rclcpp::shutdown();
  return 0;
}
