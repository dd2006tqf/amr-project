#pragma once

#include <cmath>
#include <string>
#include <vector>

#include "amr_dispatcher_core/path_tracking/pure_pursuit.hpp"

namespace amr_dispatcher_core::dispatcher {

struct CostEstimate {
  double distance_m = 0.0;
  double eta_seconds = 0.0;
  double estimated_battery_drop_v = 0.0;
  double projected_battery_voltage = 0.0;
  bool battery_sufficient = true;
};

struct CostEstimatorConfig {
  double nominal_linear_speed_mps = 0.5;
  double battery_drop_per_meter_v = 0.002;  // 每米消耗电压
  double minimum_safe_voltage_v = 21.0;     // 最低允许作业电压（例如 24V 电池系统）
  double station_stop_penalty_sec = 5.0;     // 每个中间停靠站点耗时惩罚
};

class MissionCostEstimator {
 public:
  explicit MissionCostEstimator(CostEstimatorConfig config = {});

  CostEstimate Estimate(
      double distance_m, int waypoint_count, double current_battery_voltage) const;

  CostEstimate EstimateFromPath(
      const std::vector<amr_dispatcher_core::path_tracking::Pose2D>& path,
      double current_battery_voltage) const;

  const CostEstimatorConfig& config() const { return config_; }

 private:
  CostEstimatorConfig config_;
};

}  // namespace amr_dispatcher_core::dispatcher
