cmake_minimum_required(VERSION 3.16)
project(amr_dispatcher_ros)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(amr_dispatcher_core REQUIRED)
find_package(amr_dispatcher_interfaces REQUIRED)

# 1. chassis_driver_node
add_executable(chassis_driver_node
  src/chassis_driver_node.cpp
)
target_include_directories(chassis_driver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(chassis_driver_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  amr_dispatcher_core
  amr_dispatcher_interfaces
)

# 2. dispatcher_node
add_executable(dispatcher_node
  src/dispatcher_node.cpp
)
target_include_directories(dispatcher_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(dispatcher_node
  rclcpp
  rclcpp_lifecycle
  rclcpp_action
  amr_dispatcher_core
  amr_dispatcher_interfaces
)

# 3. safety_gate_node
add_executable(safety_gate_node
  src/safety_gate_node.cpp
)
target_include_directories(safety_gate_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(safety_gate_node
  rclcpp
  geometry_msgs
  std_msgs
  amr_dispatcher_core
  amr_dispatcher_interfaces
)

# 4. path_tracker_node
add_executable(path_tracker_node
  src/path_tracker_node.cpp
)
target_include_directories(path_tracker_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(path_tracker_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
  amr_dispatcher_core
  amr_dispatcher_interfaces
)

# 5. link_health_publisher
add_executable(link_health_publisher
  src/link_health_publisher.cpp
)
target_include_directories(link_health_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(link_health_publisher
  rclcpp
  amr_dispatcher_core
  amr_dispatcher_interfaces
)

# 6. dispatcher_visualizer_node
add_executable(dispatcher_visualizer_node
  src/dispatcher_visualizer_node.cpp
)
target_include_directories(dispatcher_visualizer_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(dispatcher_visualizer_node
  rclcpp
  geometry_msgs
  std_msgs
  visualization_msgs
  amr_dispatcher_core
  amr_dispatcher_interfaces
)
target_link_libraries(dispatcher_visualizer_node yaml-cpp)

# 7. lifecycle_coordinator_node
add_executable(lifecycle_coordinator_node
  src/lifecycle_coordinator_node.cpp
)
target_include_directories(lifecycle_coordinator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(lifecycle_coordinator_node
  rclcpp
  lifecycle_msgs
  std_msgs
  amr_dispatcher_interfaces
)

# 6. ros2_control hardware plugin
add_library(amr_chassis_hardware_interface SHARED
  src/chassis_hardware_interface.cpp
)
target_include_directories(amr_chassis_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(amr_chassis_hardware_interface
  rclcpp
  hardware_interface
  pluginlib
  amr_dispatcher_core
  amr_dispatcher_interfaces
)
pluginlib_export_plugin_description_file(hardware_interface chassis_hardware_interface.xml)

# 安装
install(TARGETS
  chassis_driver_node
  dispatcher_node
  safety_gate_node
  path_tracker_node
  link_health_publisher
  dispatcher_visualizer_node
  lifecycle_coordinator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  amr_chassis_hardware_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES chassis_hardware_interface.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
