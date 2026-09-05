#include "amr_dispatcher_core/dispatcher/mission_cost_estimator.hpp"

#include "amr_dispatcher_core/path_tracking/geometry_utils.hpp"

namespace amr_dispatcher_core::dispatcher {

MissionCostEstimator::MissionCostEstimator(CostEstimatorConfig config)
    : config_(config) {}

CostEstimate MissionCostEstimator::Estimate(
    double distance_m, int waypoint_count, double current_battery_voltage) const {
  CostEstimate est;
  est.distance_m = distance_m;

  double speed = config_.nominal_linear_speed_mps > 0.05
                     ? config_.nominal_linear_speed_mps
                     : 0.5;
  double travel_time = distance_m / speed;
  double penalty = waypoint_count > 1
                       ? (waypoint_count - 1) * config_.station_stop_penalty_sec
                       : 0.0;
  est.eta_seconds = travel_time + penalty;

  est.estimated_battery_drop_v = distance_m * config_.battery_drop_per_meter_v;
  est.projected_battery_voltage = current_battery_voltage - est.estimated_battery_drop_v;
  est.battery_sufficient = (est.projected_battery_voltage >= config_.minimum_safe_voltage_v);

  return est;
}

CostEstimate MissionCostEstimator::EstimateFromPath(
    const std::vector<amr_dispatcher_core::path_tracking::Pose2D>& path,
    double current_battery_voltage) const {
  if (path.size() < 2) {
    return Estimate(0.0, static_cast<int>(path.size()), current_battery_voltage);
  }

  double total_dist = 0.0;
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    total_dist += amr_dispatcher_core::path_tracking::EuclideanDistance(
        path[i].x, path[i].y, path[i + 1].x, path[i + 1].y);
  }

  return Estimate(total_dist, static_cast<int>(path.size()), current_battery_voltage);
}

}  // namespace amr_dispatcher_core::dispatcher
