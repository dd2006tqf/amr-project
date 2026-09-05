#pragma once

#include <array>
#include <string>

#include "amr_dispatcher_core/chassis/chassis_packet.hpp"

namespace amr_dispatcher_core::chassis {

struct PlanarPose {
  double x_m = 0.0;
  double y_m = 0.0;
  double yaw_rad = 0.0;
};

struct WheelSpeeds {
  std::array<double, 4> rpm{0.0, 0.0, 0.0, 0.0};
};

std::string NormalizeKinematicsModel(const std::string& model);
PlanarPose IntegratePlanarMotion(
    const PlanarPose& pose, const ChassisCommand& command, double dt_seconds);
WheelSpeeds EstimateWheelSpeeds(
    const ChassisCommand& command, const std::string& model, double wheel_diameter_m,
    double wheel_base_m, double track_width_m);
ChassisCommand EstimateCommandFromWheelSpeeds(
    const WheelSpeeds& speeds, const std::string& model, double wheel_diameter_m,
    double wheel_base_m, double track_width_m);

}  // namespace amr_dispatcher_core::chassis
