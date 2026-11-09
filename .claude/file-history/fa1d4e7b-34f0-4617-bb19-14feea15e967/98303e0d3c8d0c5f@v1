#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "amr_dispatcher_core/chassis/backend_degradation.hpp"
#include "amr_dispatcher_core/chassis/chassis_packet_stream.hpp"
#include "amr_dispatcher_interfaces/msg/chassis_link_health.hpp"

namespace amr_dispatcher_ros {

class ChassisDriverNode : public rclcpp::Node {
 public:
  explicit ChassisDriverNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~ChassisDriverNode() override;

 private:
  void CmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void ReadPollLoop();
  void PublishHealth();
  // 按当前 protocol 选择文本行流或 Mick 二进制帧流，二者产出同一套 ChassisPacket。
  std::vector<amr_dispatcher_core::chassis::ChassisPacket> IngestBytes(const std::string& data);
  void EvaluateLinkLiveness();
  void HandleOdometry(const amr_dispatcher_core::chassis::ChassisOdometryPacket& odometry,
                      const rclcpp::Time& stamp);
  void HandleState(const amr_dispatcher_core::chassis::ChassisStatePacket& state);
  void PublishBatteryState(
      const amr_dispatcher_core::chassis::ChassisOdometryPacket& odometry,
      const rclcpp::Time& stamp);

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
  rclcpp::Publisher<amr_dispatcher_interfaces::msg::ChassisLinkHealth>::SharedPtr health_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr heartbeat_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::TimerBase::SharedPtr poll_timer_;
  rclcpp::TimerBase::SharedPtr health_timer_;

  std::unique_ptr<amr_dispatcher_core::chassis::BackendDegradationController> backend_ctrl_;
  amr_dispatcher_core::chassis::ChassisPacketStream text_stream_;
  amr_dispatcher_core::chassis::MickBinaryPacketStream binary_stream_;

  // 链路协议：text（ODOM/STATE 文本行）| mick_binary（0xAE 0xEA ... 0xEF 0xFE 帧）。
  // 必须与 ChassisBackendConfig.protocol 一致，否则下行编码与上行解析不匹配。
  std::string protocol_ = "text";
  bool binary_protocol_ = false;

  // 上行帧超时：串口句柄还在但 MCU 已停发时，链路属于"假活"。
  // PublishHealth 不再无条件置心跳，改为以此为依据，see ReadPollLoop。
  std::chrono::milliseconds odom_timeout_{300};
  std::chrono::steady_clock::time_point last_odom_stamp_{};
  bool has_odom_packet_ = false;
  bool link_alive_ = false;
  // 最近一次上报电压（V），0 表示从未收到；用于 battery_state 变化率节流。
  double last_battery_voltage_ = 0.0;
  // 保底发布节拍：电压长期不变时仍定期上报，避免电池话题静默。
  std::chrono::steady_clock::time_point last_battery_stamp_{};
  // 12V 台架默认值：满=12.6V（6 串 4.2V 折算区间不对时按单体配置调整），空=11.0V。
  // 24V 系统改用 25.2 / 21.0 即可，两个值都可从参数覆盖。
  double battery_full_voltage_ = 12.6;
  double battery_empty_voltage_ = 11.0;
  bool fault_reported_ = false;

  std::string odom_frame_id_ = "odom";
  std::string base_frame_id_ = "base_link";
  bool publish_tf_ = true;
  // B1：最近一次收到 /cmd_vel_safe 的时刻，用于指令龄超时零速兜底
  std::chrono::steady_clock::time_point last_cmd_stamp_{};
  bool has_cmd_stamp_ = false;
  // B1：指令龄超时阈值（毫秒），ReadPollLoop 超时后主动写零帧
  std::chrono::milliseconds cmd_stale_timeout_{500};
  // B1：本轮失联零速是否已告警/已写帧（恢复后重置，避免每次 poll 刷屏）
  bool stale_zero_active_ = false;

  // ---- P0.6 / A4：per-vehicle 话题与 TF 隔离 ----
  // vehicle_id 空 = 单车兼容模式（话题名与 frame 名逐字不变）；
  // 非空 = 话题按 {vid} 派生，TF frame 按 {vid}/odom、{vid}/base_link 派生。
  std::string vehicle_id_;
  std::string cmd_vel_topic_{"/cmd_vel_safe"};
  std::string odom_topic_{"/odom"};
  std::string link_health_topic_{"/chassis/link_health"};
  std::string battery_topic_{"/battery_state"};
  std::string heartbeat_topic_{"/safety/heartbeat/chassis_driver"};
};

}  // namespace amr_dispatcher_ros
