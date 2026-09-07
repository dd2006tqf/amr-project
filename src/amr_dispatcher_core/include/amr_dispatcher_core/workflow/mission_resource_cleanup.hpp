#pragma once

#include <string>

#include "amr_dispatcher_core/facility/facility_reservation.hpp"
#include "amr_dispatcher_core/workflow/traffic_planning.hpp"

namespace amr_dispatcher_core::workflow {

using amr_dispatcher_core::facility::FacilityCatalog;
using amr_dispatcher_core::facility::FacilityResource;
using amr_dispatcher_core::facility::FacilityReservationMap;
using amr_dispatcher_core::facility::MissionResourceMap;
using amr_dispatcher_core::facility::ResourceReservation;

struct MissionResourceCleanupResult {
  bool facility_resource_released = false;
  bool route_locks_released = false;
};

MissionResourceCleanupResult ReleaseMissionResources(
    FacilityReservationMap& facility_reservations, MissionResourceMap& facility_resource_by_mission,
    RouteLockMap& route_locks, RouteLockReservationMap& route_locks_by_mission,
    const std::string& mission_id);

}  // namespace amr_dispatcher_core::workflow
