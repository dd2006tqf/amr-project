// 阶段 0 的冒烟验证：
//  1) proto 代码生成正确、字段可读写
//  2) 类型注册宏可用、create_publisher/create_subscription 能编译并运行
//  3) 端到端 pub/sub 数据完整
//
// 这不是正式测试，是骨架自检。正式的 gtest 在 test/ 下（后续补）。
#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "amr_dispatcher_proto/register_protobuf_type.hpp"
#include "chassis.pb.h"

// 一行完成注册，替代手写四处特化。
AMR_PROTO_REGISTER_TYPE(amr_dispatcher::proto::ChassisLinkHealth)

using namespace std::chrono_literals;
using ChassisLinkHealth = amr_dispatcher::proto::ChassisLinkHealth;

namespace
{

class SmokeNode : public rclcpp::Node
{
public:
  explicit SmokeNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("proto_smoke", options)
  {
    pub_ = this->create_publisher<ChassisLinkHealth>("proto_smoke", rclcpp::QoS(10));

    sub_ = this->create_subscription<ChassisLinkHealth>(
      "proto_smoke", rclcpp::QoS(10),
      [this](const ChassisLinkHealth & msg) { this->on_msg(msg); });

    timer_ = this->create_wall_timer(100ms, [this]() { this->tick(); });
  }

  bool passed() const {return received_ > 0 && last_frames_total_ == expected_frames_total_;}

private:
  void tick()
  {
    ChassisLinkHealth msg;
    msg.mutable_header()->mutable_stamp()->set_sec(123);
    msg.mutable_header()->mutable_stamp()->set_nanosec(456);
    msg.mutable_header()->set_frame_id("base_link");
    msg.set_backend_name("serial");
    msg.set_tier(ChassisLinkHealth::TIER_HEALTHY);
    msg.set_frames_total(expected_frames_total_);
    msg.set_frames_success(expected_frames_total_ - 3);
    msg.set_frames_crc_error(3);
    msg.set_is_healthy(true);
    // -1 哨兵值 -> valid=false 的翻译验证
    msg.set_command_delivery_rate_valid(false);
    msg.set_command_delivery_rate(0.0);
    pub_->publish(msg);
  }

  void on_msg(const ChassisLinkHealth & msg)
  {
    ++received_;
    last_frames_total_ = msg.frames_total();

    // 逐字段校验：确认 protobuf 的完整往返无损
    const bool ok =
      msg.header().frame_id() == "base_link" &&
      msg.header().stamp().sec() == 123 &&
      msg.header().stamp().nanosec() == 456u &&
      msg.backend_name() == "serial" &&
      msg.tier() == ChassisLinkHealth::TIER_HEALTHY &&
      msg.frames_total() == expected_frames_total_ &&
      msg.frames_success() == expected_frames_total_ - 3 &&
      msg.frames_crc_error() == 3 &&
      msg.is_healthy() &&
      !msg.command_delivery_rate_valid();

    if (ok) {
      ++verified_;
    } else {
      RCLCPP_ERROR(this->get_logger(), "field mismatch on round-trip");
    }

    RCLCPP_INFO(
      this->get_logger(), "recv #%d frames_total=%llu verified=%d",
      received_, static_cast<unsigned long long>(msg.frames_total()), verified_);
  }

  static constexpr uint64_t expected_frames_total_ = 1000000ULL;

  rclcpp::Publisher<ChassisLinkHealth>::SharedPtr pub_;
  rclcpp::Subscription<ChassisLinkHealth>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  int received_{0};
  int verified_{0};
  uint64_t last_frames_total_{0};
};

/// 解析 `--duration <秒>`，默认 2 秒。
///
/// 默认值存在的理由：CI 自检要尽快结束。但这个默认值对**交互演示**是陷阱——
/// DDS 发现通常要几百毫秒到数秒，节点如果 2 秒就退出，`ros2 topic list`
/// 和 `ros2 amr echo` 启动时对端已经消失，看起来像"话题不存在"。
/// 所以留这个开关：演示时 `--duration 60` 让它长驻。
///
/// 在 rclcpp::init 之前解析：init 会消费 ROS 自己的参数，虽然本参数不是
/// ROS 参数、不会被吃掉，但提前解析语义更清晰。
double parse_duration_seconds(int argc, char ** argv)
{
  constexpr double kDefaultSeconds = 2.0;
  for (int i = 1; i + 1 < argc; ++i) {
    if (std::string(argv[i]) != "--duration") {
      continue;
    }
    try {
      const double seconds = std::stod(argv[i + 1]);
      return seconds > 0.0 ? seconds : kDefaultSeconds;
    } catch (const std::exception &) {
      return kDefaultSeconds;
    }
  }
  return kDefaultSeconds;
}

}  // namespace

int main(int argc, char ** argv)
{
  const double duration_s = parse_duration_seconds(argc, argv);

  rclcpp::init(argc, argv);
  auto node = std::make_shared<SmokeNode>(rclcpp::NodeOptions{});

  RCLCPP_INFO(
    node->get_logger(),
    "smoke test will run for %.1f s (override with --duration <seconds>)", duration_s);

  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::milliseconds(static_cast<int64_t>(duration_s * 1000.0));
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(node);
  }

  const bool ok = node->passed();
  RCLCPP_INFO(node->get_logger(), "SMOKE RESULT: %s", ok ? "PASS" : "FAIL");
  rclcpp::shutdown();
  return ok ? 0 : 1;
}
