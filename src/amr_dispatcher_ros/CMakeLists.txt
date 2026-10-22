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
# Eclipse Paho MQTT C++ 可选（Task 4 vda5050_gateway_node）：镜像缺包时跳过该目标编译，
# full_system.launch.py 默认 use_vda5050=false 不引用该可执行文件，构建永不因缺 Paho 挂掉。
# I1：C++ 头/库与 C 库三者齐备才启用——C 库是条件链接（pp3 依赖 MQTTAsync_* 符号，
# 半装环境会编译通过再在链接期报 undefined reference）；NAMES 只列异步变体
# （同步 3c/3cs 无 MQTTAsync_*，链上也无法满足 pp3 的依赖）。
find_path(PAHO_MQTTPP_INCLUDE_DIR NAMES mqtt/async_client.h)
find_library(PAHO_MQTTPP_LIBRARY NAMES paho-mqttpp3)
find_library(PAHO_MQTT_LIBRARY NAMES paho-mqtt3as paho-mqtt3a)
if(PAHO_MQTTPP_INCLUDE_DIR AND PAHO_MQTTPP_LIBRARY AND PAHO_MQTT_LIBRARY)
  set(AMR_HAVE_PAHO_MQTT TRUE)
else()
  set(AMR_HAVE_PAHO_MQTT FALSE)
  message(WARNING "未找到 Eclipse Paho MQTT C++ (libpaho-mqttpp-dev) 或异步 C 客户端库 (libpaho-mqtt-dev)，跳过 vda5050_gateway_node 编译")
endif()

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

# 8. map_ingest_node（Task 8）：/map -> 核心库算法流水线（安全空间腐蚀、
# 骨架细化、受保护剪枝、拓扑构建）-> 原子写出 stations.yaml + 拓扑 Marker。
add_executable(map_ingest_node
  src/map_ingest_node.cpp
)
target_include_directories(map_ingest_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(map_ingest_node
  rclcpp
  nav_msgs
  geometry_msgs
  std_msgs
  visualization_msgs
  amr_dispatcher_core
)
# yaml-cpp：解析 markers_file 的 anchors 段（与 dispatcher_visualizer 同一链接方式）
target_link_libraries(map_ingest_node yaml-cpp)

# 8.1 vda5050_gateway_node（Task 4）：VDA 5050 <-> ROS 2 MQTT 薄壳网关。
# MQTT 类型封装在 .cpp（pimpl），核心库 amr_dispatcher_core 保持零 MQTT 依赖（Spec §1 铁律）；
# 仅在检测到 Paho 时编译（见文件头 AMR_HAVE_PAHO_MQTT 判定）
if(AMR_HAVE_PAHO_MQTT)
  add_executable(vda5050_gateway_node
    src/vda5050_gateway_node.cpp
  )
  target_include_directories(vda5050_gateway_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(vda5050_gateway_node
    rclcpp
    std_msgs
    geometry_msgs
    amr_dispatcher_core
  )
  # 三元守卫已保证 C 异步库存在；显式链接 C 客户端，
  # 防止 C++ 封装静态链接场景缺 MQTTAsync_* 符号
  target_link_libraries(vda5050_gateway_node
    ${PAHO_MQTTPP_LIBRARY}
    ${PAHO_MQTT_LIBRARY}
  )
endif()

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
    test_report_freshness
    test_assignment_dispatch_decision
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
  map_ingest_node
  DESTINATION lib/${PROJECT_NAME}
)

# vda5050_gateway_node 仅在 Paho 可用时生成，条件安装
if(TARGET vda5050_gateway_node)
  install(TARGETS vda5050_gateway_node
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

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
