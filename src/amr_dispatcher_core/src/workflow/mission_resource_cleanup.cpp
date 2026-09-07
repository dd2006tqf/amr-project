#include "amr_dispatcher_core/workflow/mission_resource_cleanup.hpp"

namespace amr_dispatcher_core::workflow {

using amr_dispatcher_core::facility::FacilityCatalog;
using amr_dispatcher_core::facility::FacilityResource;
using amr_dispatcher_core::facility::FacilityReservationMap;
using amr_dispatcher_core::facility::MissionResourceMap;
using amr_dispatcher_core::facility::ResourceReservation;

MissionResourceCleanupResult ReleaseMissionResources(
    FacilityReservationMap& facility_reservations, MissionResourceMap& facility_resource_by_mission,
    RouteLockMap& route_locks, RouteLockReservationMap& route_locks_by_mission,
    const std::string& mission_id) {
  MissionResourceCleanupResult result;
  result.facility_resource_released =
      ReleaseResourceForMission(facility_reservations, facility_resource_by_mission, mission_id);
  result.route_locks_released =
      ReleaseRouteLocksForMission(route_locks, route_locks_by_mission, mission_id);
  return result;
}

}  // namespace amr_dispatcher_core::workflow
