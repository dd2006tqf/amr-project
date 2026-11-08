// 派发时的起终点解析（纯函数）。
//
// 原实现内联在 ScheduleTick 尾段，混了三种信息源：
//   1. 目录里的 pickup 站点      -> 起点 = 该站位姿
//   2. 目录里的 dropoff 站点     -> 终点 = 该站位姿
//   3. 没有 pickup 但有里程计    -> 起点 = 车辆当前位姿
// 其中 3 是"车辆已就位、直接在原地接单"的语义；dropoff 解析不出来则必须判负，
// 且失败文案（`unknown dropoff station: <id>`）被 e2e 与事件消费者依赖。
//
// 抽成纯函数后可以单测"没有 pickup 时是否用了里程计位姿"、"目录未加载时是否
// 仍然失败"、"站点存在但 dropoff 为空"等边界，不必起 ROS 图。
#pragma once

#include <string>

#include "amr_dispatcher_core/catalog/station_catalog.hpp"
#include "amr_dispatcher_core/path_tracking/pure_pursuit.hpp"

namespace amr_dispatcher_core::workflow {

struct DispatchEndpointRequest {
  std::string pickup_station;
  std::string dropoff_station;
  bool catalog_loaded = false;
  const catalog::StationCatalog* catalog = nullptr;
  // 无 pickup 时的起点来源：车辆当前位姿
  bool has_odom = false;
  path_tracking::Pose2D current_pose;
};

struct DispatchEndpointResolution {
  enum class Kind {
    kResolved,          // 起终点齐备，可以规划
    kUnknownDropoff,    // dropoff 为空或不在目录里 -> 判负
  };
  Kind kind = Kind::kUnknownDropoff;
  path_tracking::Pose2D start;  // 仅在 kResolved 时有意义
  path_tracking::Pose2D goal;
  bool start_from_odom = false;  // 起点取的是车辆当前位姿（审计/日志用）
  // 判负原因，与旧实现逐字一致（e2e 与事件消费者依赖该文案）
  std::string reason;
};

inline DispatchEndpointResolution ResolveDispatchEndpoints(
    const DispatchEndpointRequest& request) {
  DispatchEndpointResolution resolution;
  const catalog::Station* from_st = nullptr;
  const catalog::Station* to_st = nullptr;
  if (request.catalog_loaded && request.catalog != nullptr) {
    if (!request.pickup_station.empty()) {
      from_st = catalog::FindStation(*request.catalog, request.pickup_station);
    }
    if (!request.dropoff_station.empty()) {
      to_st = catalog::FindStation(*request.catalog, request.dropoff_station);
    }
  }
  if (request.dropoff_station.empty() || to_st == nullptr) {
    resolution.kind = DispatchEndpointResolution::Kind::kUnknownDropoff;
    resolution.reason = "unknown dropoff station: " + request.dropoff_station;
    return resolution;
  }
  if (from_st != nullptr) {
    resolution.start = {from_st->x, from_st->y, from_st->yaw};
  } else if (request.has_odom) {
    resolution.start = request.current_pose;
    resolution.start_from_odom = true;
  }
  resolution.goal = {to_st->x, to_st->y, to_st->yaw};
  resolution.kind = DispatchEndpointResolution::Kind::kResolved;
  return resolution;
}

}  // namespace amr_dispatcher_core::workflow
