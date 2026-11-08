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
find_package(sensor_msgs REQUIRED)
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
# 阶段 2：protobuf 双发（见 PROTOBUF_MIGRATION_PLAN.md §3.3）。
#
# 这里是**硬依赖**，不学 nav2_msgs / Paho 那样 QUIET 跳过。理由：
# 双发是阶段 2 的正式功能，静默降级会得到一个「构建成功但 proto 话题根本不存在」
# 的镜像，而这种缺失只有在运维真的去订阅时才暴露——比构建期直接失败更贵。
# 部署侧依赖已在 docker/Dockerfile.jazzy 与 CI 中显式安装。
find_package(amr_dispatcher_proto REQUIRED)
# 阶段 3：反向投影（proto -> rosidl）与双订阅适配器。与上面同理，是硬依赖。
find_package(amr_dispatcher_proto_bridge REQUIRED)
# 显式 find Protobuf：下游目标的 pb.h 会 include <google/protobuf/*>，
# 需要 Protobuf_INCLUDE_DIRS。见下面 amr_dispatcher_proto_dual 的注释。
find_package(Protobuf REQUIRED)
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

# 2.0 proto 投影层（阶段 2 双发的地基）
#
# 做成共享库而不是 header-only，原因有两条：
#   1. 投影实现在 .cpp 里（见 proto_projection.cpp）。7 个消息 × 每字段一行，
#      放头文件会被 8 个可执行目标各编译一次，构建时间线性膨胀。
#   2. 单测（test_proto_projection）需要链接同一份实现来验证字段完整性，
#      共享库让「测的就是用的」成立；header-only 也能做到，但容易在
#      不同 TU 里出现 ODR 违规的弱符号。
add_library(amr_dispatcher_proto_projection SHARED
  src/proto_projection.cpp
)
target_include_directories(amr_dispatcher_proto_projection PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(amr_dispatcher_proto_projection
  rclcpp
  std_msgs
  amr_dispatcher_interfaces
  amr_dispatcher_proto
)

# 双发所需的**全部**编译期依赖，打包成一个 INTERFACE 目标。
#
# 为什么需要这一层：调用 create_publisher<ProtoT> 的节点必须同时拿到
#   - amr_dispatcher_proto 的 include（register_all_types.hpp、protobuf_typesupport.hpp）
#   - 生成的 pb.h 所在目录（由 amr_dispatcher_proto 的 include 导出覆盖）
#   - Protobuf 自身的头文件（pb.h 内部 include <google/protobuf/*>）
#   - fastcdr（protobuf_typesupport.hpp include <fastcdr/Cdr.h>）
# 后两项 amr_dispatcher_proto 通过 ament_export_dependencies 导出，但
# ament_export_dependencies 只负责让 find_package 成功，不会自动把
# Protobuf_INCLUDE_DIRS 塞进下游目标。漏掉它会在下游报
# "google/protobuf/message.h: No such file or directory"，而错误位置在
# proto 包的头文件里，指向性很差。
add_library(amr_dispatcher_proto_dual INTERFACE)
target_link_libraries(amr_dispatcher_proto_dual INTERFACE
  amr_dispatcher_proto_projection
  # 阶段 3：把反向投影库一并带给消费者目标。凡是用 ProtoDualSubscriber 的
  # 可执行目标都已在用本 INTERFACE 目标（双发端），订阅端因此零额外改动即可
  # 拿到 amr_dispatcher_proto_bridge 的头与库。
  amr_dispatcher_proto_bridge::amr_dispatcher_proto_bridge
  ${Protobuf_LIBRARIES}
)
target_include_directories(amr_dispatcher_proto_dual INTERFACE
  ${Protobuf_INCLUDE_DIRS}
)

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
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  amr_dispatcher_core
  amr_dispatcher_interfaces
)
# 阶段 2 双发：ChassisLinkHealth -&gt; /chassis/link_health_proto
target_link_libraries(chassis_driver_node amr_dispatcher_proto_dual)

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
  sensor_msgs
  amr_dispatcher_core
  amr_dispatcher_interfaces
)
# 行为树编排层：订单路由、出队派发门控、恢复与结果决策
target_link_libraries(dispatcher_node amr_dispatcher_bt::amr_dispatcher_bt)
# 阶段 2 双发：DispatcherState / MissionEvent / TopologyState -> *_proto
# 注意：dispatcher_node 是 lifecycle 节点，双发器用的是 ProtoLifecycleDualPublisher
# （proto 侧跟随 on_activate/on_deactivate），编译期依赖与普通版完全相同。
target_link_libraries(dispatcher_node amr_dispatcher_proto_dual)


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
# 阶段 3：/tracking/error 双订阅（proto 分支反投影回 rosidl）
target_link_libraries(navigate_sequence_bridge_node
  amr_dispatcher_proto_bridge::amr_dispatcher_proto_bridge)

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
# 阶段 2 双发：SafetyState -> /safety/state_proto
target_link_libraries(safety_gate_node amr_dispatcher_proto_dual)

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
# 阶段 2 双发：TrackingError -> /tracking/error_proto（全项目频率最高的双发点）
target_link_libraries(path_tracker_node amr_dispatcher_proto_dual)

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
# 阶段 2 双发：/system/aggregated_link_health_proto
target_link_libraries(link_health_publisher amr_dispatcher_proto_dual)

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
# 阶段 3：/dispatcher/topology_state 双订阅
target_link_libraries(dispatcher_visualizer_node
  amr_dispatcher_proto_bridge::amr_dispatcher_proto_bridge)

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
# 阶段 3：/safety/state 双订阅
target_link_libraries(lifecycle_coordinator_node
  amr_dispatcher_proto_bridge::amr_dispatcher_proto_bridge)

# 7.1 proto_dual_benchmark：双发开销基准（不是测试，不注册 ctest 断言）。
#
# 为什么放在这里而不是 proto 包：它测的是**投影层**（proto_projection）的成本，
# 而投影层在本包内（proto 包只提供 typesupport 与消息定义，不含 rosidl 依赖）。
#
# 为什么不做成 ctest：计时断言在共享机器上必然抖动，见源文件头注释。
#
# 装到 lib/${PROJECT_NAME} 与其它节点一致，用 `ros2 run amr_dispatcher_ros
# proto_dual_benchmark` 即可执行；它不读 ROS 参数、不需要 rclcpp::init。
add_executable(proto_dual_benchmark
  src/proto_dual_benchmark.cpp
)
target_link_libraries(proto_dual_benchmark amr_dispatcher_proto_dual)

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

  # 纯逻辑单测：只依赖 amr_dispatcher_ros_helpers（rclcpp/几何/cored 的薄封装）
  set(ROS_UNIT_TESTS
    test_safety_publish_decision
    test_nav_pose_builder
    test_mission_identity
    test_report_freshness
    test_assignment_dispatch_decision
    test_chassis_fault
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

  # 阶段 2 双发的投影层单测：需要链接**同一份** amr_dispatcher_proto_projection
  # 实现（而不是把投影再写一遍），才能验证「测的就是用的」。
  #
  # 单独成组而不并入上面那个循环，是因为它需要 amr_dispatcher_proto_dual 的
  # 全部编译期依赖（protobuf 头文件、pb.h、typesupport）；把它塞进通用循环会
  # 让 6 个纯逻辑测试平白多出 protobuf 依赖。
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/test_proto_projection.cpp")
    add_executable(test_proto_projection
      "${CMAKE_CURRENT_SOURCE_DIR}/test/test_proto_projection.cpp")
    target_link_libraries(test_proto_projection PRIVATE
      amr_dispatcher_proto_dual GTest::gtest GTest::gtest_main)
    gtest_discover_tests(test_proto_projection)
  endif()

  # 阶段 3 往返恒等测试：rosidl -> proto -> rosidl。
  #
  # 必须同时链接正向投影（amr_dispatcher_proto_dual，本包内）与反向投影
  # （amr_dispatcher_proto_bridge）。两个投影层各自的逐字段单测都通过，
  # 并不能推出二者互逆——只有首尾相接跑一遍才能证明阶段 3 切换后消费者
  # 看到的消息与切换前逐字相同，这是阶段 3 的验收标准本身。
  #
  # 本目标额外带了一个话题名 parity 用例：ros 侧与 bridge 侧各有一份
  # kProtoTopicSuffix（bridge 不能依赖本包），此处是唯一的漂移守卫。
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/test_proto_round_trip.cpp")
    add_executable(test_proto_round_trip
      "${CMAKE_CURRENT_SOURCE_DIR}/test/test_proto_round_trip.cpp")
    target_link_libraries(test_proto_round_trip PRIVATE
      amr_dispatcher_proto_dual GTest::gtest GTest::gtest_main)
    gtest_discover_tests(test_proto_round_trip)
  endif()

  # L2 契约测试：真进程 + 真服务调用（不再是 assertTrue(True)）。
  # 只在 launch_testing 可用时注册——宿主机的纯 CMake 构建没有它。
  find_package(launch_testing_ament_cmake QUIET)
  if(launch_testing_ament_cmake_FOUND)
    add_launch_test(
      "${CMAKE_CURRENT_SOURCE_DIR}/../../test/l2_launch_contracts/test_lifecycle_contracts.py"
      TARGET test_lifecycle_contracts
      TIMEOUT 120
    )
    # 守护契约：respawn 起新进程 + 协调器重新提拉 + on_configure 服务重建。
    # 域 94（与上面 91 域分开）：两套夹具的节点互相看见会让断言不可信。
    # 预算 60s + 重生延迟/提拉/重建服务，给慢 CI 留余量。
    add_launch_test(
      "${CMAKE_CURRENT_SOURCE_DIR}/../../test/l2_launch_contracts/test_supervision_contracts.py"
      TARGET test_supervision_contracts
      TIMEOUT 240
    )
  endif()
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
  proto_dual_benchmark
  DESTINATION lib/${PROJECT_NAME}
)

# vda5050_gateway_node 仅在 Paho 可用时生成，条件安装
if(TARGET vda5050_gateway_node)
  install(TARGETS vda5050_gateway_node
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

install(TARGETS
  amr_dispatcher_proto_projection
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
