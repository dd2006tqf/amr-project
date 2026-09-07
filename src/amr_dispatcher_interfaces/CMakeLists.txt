cmake_minimum_required(VERSION 3.16)
project(amr_dispatcher_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

set(msg_files
  "msg/DispatcherState.msg"
  "msg/ChassisLinkHealth.msg"
  "msg/MissionEvent.msg"
  "msg/TrackingError.msg"
  "msg/SafetyState.msg"
  "msg/TopologyState.msg"
)

set(srv_files
  "srv/SubmitOrder.srv"
  "srv/CancelOrder.srv"
  "srv/PauseMission.srv"
  "srv/ResumeMission.srv"
  "srv/ReserveResource.srv"
  "srv/ReleaseResource.srv"
  "srv/GetSystemReadiness.srv"
  "srv/EnqueueMission.srv"
  "srv/CancelQueuedMission.srv"
  "srv/PreemptMission.srv"
  "srv/ReprioritizeQueuedMission.srv"
  "srv/EstimateMissionCost.srv"
  "srv/BlockStationRoute.srv"
  "srv/DetectTrafficDeadlock.srv"
  "srv/ListStations.srv"
  "srv/ListTrafficReservations.srv"
  "srv/ReserveFacilityResource.srv"
  "srv/GetOperatorSnapshot.srv"
  "srv/ValidateSiteConfig.srv"
)

set(action_files
  "action/NavigateSequence.action"
  "action/ExecuteMission.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES
    std_msgs
    geometry_msgs
    builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
