#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

#include "amr_dispatcher_core/chassis/backend_degradation.hpp"
#include "amr_dispatcher_core/chassis/chassis_kinematics.hpp"
#include "amr_dispatcher_core/chassis/chassis_packet_stream.hpp"

namespace amr_dispatcher_ros {

class ChassisHardwareInterface : public hardware_interface::SystemInterface {
 public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
      return CallbackReturn::ERROR;
    }

    // 默认双轮差速参数
    wheel_separation_ = std::stod(info_.hardware_parameters.count("wheel_separation")
                                      ? info_.hardware_parameters.at("wheel_separation")
                                      : "0.45");
    wheel_radius_ = std::stod(info_.hardware_parameters.count("wheel_radius")
                                  ? info_.hardware_parameters.at("wheel_radius")
                                  : "0.08");

    amr_dispatcher_core::chassis::ChassisBackendConfig cfg;
    cfg.serial_device = info_.hardware_parameters.count("serial_device")
                            ? info_.hardware_parameters.at("serial_device")
                            : "/dev/ttyUSB0";
    cfg.serial_baud = std::stoi(info_.hardware_parameters.count("serial_baud")
                                    ? info_.hardware_parameters.at("serial_baud")
                                    : "115200");

    backend_ctrl_ = std::make_unique<amr_dispatcher_core::chassis::BackendDegradationController>(
        amr_dispatcher_core::chassis::DefaultBackendDescriptors(),
        std::vector<amr_dispatcher_core::chassis::ChassisBackendConfig>{cfg, cfg, cfg});

    hw_positions_.resize(info_.joints.size(), 0.0);
    hw_velocities_.resize(info_.joints.size(), 0.0);
    hw_commands_.resize(info_.joints.size(), 0.0);

    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (std::size_t i = 0; i < info_.joints.size(); ++i) {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    }
    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (std::size_t i = 0; i < info_.joints.size(); ++i) {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
    }
    return command_interfaces;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State& /*previous_state*/) override {
    for (std::size_t i = 0; i < hw_commands_.size(); ++i) {
      hw_commands_[i] = 0.0;
    }
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) override {
    if (backend_ctrl_ && backend_ctrl_->current()) {
      amr_dispatcher_core::chassis::ChassisCommand stop_cmd;
      std::string err;
      backend_ctrl_->current()->WriteCommand(stop_cmd, &err);
    }
    return CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type read(
      const rclcpp::Time& /*time*/, const rclcpp::Duration& period) override {
    if (!backend_ctrl_ || !backend_ctrl_->current() || !backend_ctrl_->current()->IsOpen()) {
      return hardware_interface::return_type::OK;
    }

    std::string err;
    auto data = backend_ctrl_->current()->Read(&err);
    if (data && !data->empty()) {
      text_stream_.Append(*data);
      auto packets = text_stream_.DrainPackets();
      for (const auto& pkt : packets) {
        if (pkt.kind == amr_dispatcher_core::chassis::ChassisPacketKind::kOdometry) {
          const auto& o = pkt.odometry;
          // 差速轮速逆解
          if (hw_velocities_.size() >= 2) {
            double v_l = (o.linear_x_mps - o.angular_z_radps * wheel_separation_ / 2.0) / wheel_radius_;
            double v_r = (o.linear_x_mps + o.angular_z_radps * wheel_separation_ / 2.0) / wheel_radius_;
            hw_velocities_[0] = v_l;
            hw_velocities_[1] = v_r;
            hw_positions_[0] += v_l * period.seconds();
            hw_positions_[1] += v_r * period.seconds();
          }
        }
      }
    }
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write(
      const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) override {
    if (!backend_ctrl_ || !backend_ctrl_->current() || !backend_ctrl_->current()->IsOpen()) {
      return hardware_interface::return_type::OK;
    }

    if (hw_commands_.size() >= 2) {
      double w_l = hw_commands_[0];
      double w_r = hw_commands_[1];
      double linear_x = (w_r + w_l) * wheel_radius_ / 2.0;
      double angular_z = (w_r - w_l) * wheel_radius_ / wheel_separation_;

      amr_dispatcher_core::chassis::ChassisCommand cmd;
      cmd.linear_x_mps = linear_x;
      cmd.angular_z_radps = angular_z;

      std::string err;
      backend_ctrl_->current()->WriteCommand(cmd, &err);
    }
    return hardware_interface::return_type::OK;
  }

 private:
  double wheel_separation_ = 0.45;
  double wheel_radius_ = 0.08;

  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_commands_;

  std::unique_ptr<amr_dispatcher_core::chassis::BackendDegradationController> backend_ctrl_;
  amr_dispatcher_core::chassis::ChassisPacketStream text_stream_;
};

}  // namespace amr_dispatcher_ros

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(amr_dispatcher_ros::ChassisHardwareInterface, hardware_interface::SystemInterface)
