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
find_package(amr_dispatcher_bt REQUIRED)
# nav2_msgs 可选：镜像缺包时 QUIET 跳过，桥接节点 Nav2 分支条件编译，容器构建永不因 Nav2 挂掉
find_package(nav2_msgs QUIET)

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
  nav_msgs
  geometry_msgs
  amr_dispatcher_core
  amr_dispatcher_interfaces
)
# 行为树编排层：订单路由、出队派发门控、恢复与结果决策
target_link_libraries(dispatcher_node amr_dispatcher_bt::amr_dispatcher_bt)

# 2.1 navigate_sequence_bridge_node
# /navigate_sequence action server：fallback 转 /plan（复用 path_tracker 链），
# use_nav2:=true 时条件编译走 Nav2 navigate_to_pose（镜像缺 nav2_msgs 则自动降级）
add_executable(navigate_sequence_bridge_node
  src/navigate_sequence_bridge_node.cpp
)
target_include_directories(navigate_sequence_bridge_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(navigate_sequence_bridge_node
  rclcpp
  rclcpp_action
  nav_msgs
  geometry_msgs
  amr_dispatcher_interfaces
)
if(nav2_msgs_FOUND)
  target_compile_definitions(navigate_sequence_bridge_node PRIVATE AMR_HAVE_NAV2_MSGS)
  ament_target_dependencies(navigate_sequence_bridge_node nav2_msgs)
endif()

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

# 单元测试：可抽离的纯逻辑（B1 周期发布判定 / B3 航点位姿 / B4 任务戳记）
# 以 header-only 库 + gtest 单测落地，模式与 amr_dispatcher_core 的 BUILD_TESTING 分支一致。
if(BUILD_TESTING)
  find_package(GTest REQUIRED)
  include(GoogleTest)
  enable_testing()

  add_library(amr_dispatcher_ros_helpers INTERFACE)
  target_include_directories(amr_dispatcher_ros_helpers INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(amr_dispatcher_ros_helpers INTERFACE
    geometry_msgs
    amr_dispatcher_core
  )

  set(ROS_UNIT_TESTS
    test_safety_publish_decision
    test_nav_pose_builder
    test_mission_identity
  )
  foreach(test_name IN LISTS ROS_UNIT_TESTS)
    set(test_src "${CMAKE_CURRENT_SOURCE_DIR}/test/${test_name}.cpp")
    if(EXISTS "${test_src}")
      add_executable(${test_name} "${test_src}")
      target_link_libraries(${test_name} PRIVATE
        amr_dispatcher_ros_helpers GTest::gtest GTest::gtest_main)
      gtest_discover_tests(${test_name})
    endif()
  endforeach()
endif()

# 安装
install(TARGETS
  chassis_driver_node
  dispatcher_node
  navigate_sequence_bridge_node
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

# B7：站点/节点参数文件与 launch 安装到 share，ros2 launch 用 get_package_share_directory 引用，
# 避免依赖 colcon 工作区相对路径（修复 stations_file 在容器外运行时找不到的问题）。
install(DIRECTORY
  ${CMAKE_CURRENT_SOURCE_DIR}/../../config
  ${CMAKE_CURRENT_SOURCE_DIR}/../../launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
