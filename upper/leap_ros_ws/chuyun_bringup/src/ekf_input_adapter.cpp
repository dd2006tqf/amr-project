#include <algorithm>
#include <array>
#include <memory>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace
{
template<typename Covariance>
void set_diagonal(Covariance & covariance, const std::array<double, 6> & diagonal)
{
  std::fill(covariance.begin(), covariance.end(), 0.0);
  for (std::size_t index = 0; index < diagonal.size(); ++index) {
    covariance[index * 6 + index] = diagonal[index];
  }
}

template<typename Covariance>
void set_diagonal_3d(Covariance & covariance, const std::array<double, 3> & diagonal)
{
  std::fill(covariance.begin(), covariance.end(), 0.0);
  for (std::size_t index = 0; index < diagonal.size(); ++index) {
    covariance[index * 3 + index] = diagonal[index];
  }
}
}  // namespace

class EkfInputAdapter : public rclcpp::Node
{
public:
  EkfInputAdapter()
  : Node("ekf_input_adapter")
  {
    const auto input_qos = rclcpp::SensorDataQoS().keep_last(20);
    const auto output_qos = rclcpp::QoS(rclcpp::KeepLast(20)).reliable();

    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>(
      "/odom/ekf_input", output_qos);
    imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>(
      "/imu/ekf_input", output_qos);

    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", input_qos,
      [this](const nav_msgs::msg::Odometry::SharedPtr source) {
        auto message = *source;
        message.child_frame_id = "base_footprint";

        // Wheel odometry: x/y and yaw pose, forward/lateral velocity.
        set_diagonal(
          message.pose.covariance,
          {0.01, 0.01, 1.0e6, 1.0e6, 1.0e6, 0.04});
        set_diagonal(
          message.twist.covariance,
          {0.0025, 0.0004, 1.0e6, 1.0e6, 1.0e6, 0.04});
        odom_publisher_->publish(message);
      });

    imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu", input_qos,
      [this](const sensor_msgs::msg::Imu::SharedPtr source) {
        auto message = *source;
        if (message.header.frame_id.empty()) {
          message.header.frame_id = "imu_link";
        }

        // The MCU currently marks all IMU covariances as unknown (-1).
        // Supply conservative values so robot_localization can use gyro Z.
        set_diagonal_3d(message.orientation_covariance, {0.10, 0.10, 0.04});
        set_diagonal_3d(message.angular_velocity_covariance, {0.04, 0.04, 0.0025});
        set_diagonal_3d(message.linear_acceleration_covariance, {0.10, 0.10, 0.10});
        imu_publisher_->publish(message);
      });

    RCLCPP_INFO(
      get_logger(),
      "EKF inputs ready: /odom -> /odom/ekf_input, /imu -> /imu/ekf_input");
  }

private:
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EkfInputAdapter>());
  rclcpp::shutdown();
  return 0;
}
