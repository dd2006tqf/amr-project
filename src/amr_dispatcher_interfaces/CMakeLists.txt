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
)

set(srv_files
  "srv/SubmitOrder.srv"
  "srv/CancelOrder.srv"
  "srv/PauseMission.srv"
  "srv/ResumeMission.srv"
  "srv/ReserveResource.srv"
  "srv/ReleaseResource.srv"
)

set(action_files
  "action/NavigateSequence.action"
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
