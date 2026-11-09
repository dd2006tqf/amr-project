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
#include "amr_dispatcher_core/chassis/command_delivery_tracker.hpp"
#include "amr_dispatcher_interfaces/msg/chassis_link_health.hpp"
#include "amr_dispatcher_ros/proto_dual_publisher.hpp"

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
  // 0xA1 与 0xA2 共用的电压发布路径（共用节流状态，见实现处的注释）。
  void PublishBatteryVoltage(double voltage, const rclcpp::Time& stamp);
  // T2：故障位进安全链。锁存语义：置位后链路判不可信，收到无故障帧才解除。
  void UpdateChassisFaultState(bool fault, const char* source);

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
  // 阶段 4b：原先这里还有 health_pub_（节点自建的 rosidl 发布器）。
  // 已删除并收进 health_proto_pub_——留着它就会出现两个 rosidl 发布器
  // 指向同一话题，而「只发 proto」的开关也因此不能真正生效。
  //
  // 阶段 2 双发：与 rosidl 侧同源同帧的 protobuf 副本（话题 /chassis/link_health_proto）。
  // 用 unique_ptr 是因为 ProtoDualPublisher 必须以「解析后的最终话题名」构造，
  // 而该名字在构造函数体内才算出（依赖 vehicle_id 与覆盖参数）。
  std::unique_ptr<ProtoDualPublisher<
      amr_dispatcher_interfaces::msg::ChassisLinkHealth,
      amr_dispatcher::proto::ChassisLinkHealth>> health_proto_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr heartbeat_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::TimerBase::SharedPtr poll_timer_;
  rclcpp::TimerBase::SharedPtr health_timer_;

  std::unique_ptr<amr_dispatcher_core::chassis::BackendDegradationController> backend_ctrl_;
  amr_dispatcher_core::chassis::ChassisPacketStream text_stream_;
  amr_dispatcher_core::chassis::MickBinaryPacketStream binary_stream_;
  // T3：下行序号 + 命令送达率。只在 mick_binary 协议下推进 seq；
  // 文本协议的 MCU 不回显，统计保持“无回显”状态（送达率 0、不参与健康判定）。
  amr_dispatcher_core::chassis::CommandDeliveryTracker delivery_tracker_;

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
  // T2：底盘上报的故障锁存。置位后链路判为不可信（心跳下沉），直到
  // 收到一帧无故障的上行帧才自动解除。锁存是必需的：0xA2 只在状态变化时
  // 发送，若只在“收到 fault 帧的那一拍”降级，下一拍就恢复了。
  bool chassis_fault_latched_ = false;

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

  // 阶段 2：proto 双发总开关（参数 proto_dual_publish）。
  // 在构造函数最前面声明并读出，后续所有双发器都引用它。
  bool proto_dual_publish_{true};
  // 阶段 4b：rosidl 侧开关（参数 proto_rosidl_publish，默认开）。
  // false = 只发 *_proto，DDS 里不再有 /chassis/link_health。
  bool proto_rosidl_publish_{true};
};

}  // namespace amr_dispatcher_ros
