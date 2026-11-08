cmake_minimum_required(VERSION 3.16)
project(amr_dispatcher_proto_bridge)

# C++17：与 amr_dispatcher_proto 一致（if constexpr 类型分派）。
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(amr_dispatcher_interfaces REQUIRED)
find_package(amr_dispatcher_proto REQUIRED)
find_package(Protobuf REQUIRED)

# 反向投影做成共享库，理由与 amr_dispatcher_ros 的正向投影库相同：
#   1. 实现集中在 .cpp：7 个消息 × 每字段一行，放头文件会被每个消费者
#      TU 各编译一次；
#   2. 单测链接同一份实现，「测的就是用的」。
add_library(${PROJECT_NAME} SHARED
  src/proto_reverse_projection.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  # Protobuf 头文件必须 PUBLIC 传播：本包头文件 include 了 pb.h，下游编译时
  # 会继续 include <google/protobuf/*>。ament_export_dependencies 只保证
  # find_package 成功，不会把 Protobuf_INCLUDE_DIRS 塞进下游目标——与
  # amr_dispatcher_ros 的 amr_dispatcher_proto_dual 是同一个坑。
  ${Protobuf_INCLUDE_DIRS}
)

# 注意签名：Jazzy 的 ament_target_dependencies **内部用的是 plain 签名**，
# 因此下面追加 protobuf 库时也必须是 plain（不带 PUBLIC 关键字），
# 否则报 "All uses of target_link_libraries with a target must be either
# all-keyword or all-plain"（已实测两次，两种混法各撞一次）。
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  amr_dispatcher_interfaces
  amr_dispatcher_proto
)
target_link_libraries(${PROJECT_NAME} ${Protobuf_LIBRARIES})

# 单测：反向投影的逐字段覆盖。
#
# 往返测试不在这里：它需要**正向**投影（rosidl -> proto），而正向投影在
# amr_dispatcher_ros 包里。放进本包会让 bridge 反向依赖 ros，正是本包
# 存在的意义所在（tools 必须能不依赖 ros）。往返测试因此落在
# amr_dispatcher_ros/test/test_proto_round_trip.cpp。
if(BUILD_TESTING)
  find_package(GTest REQUIRED)
  include(GoogleTest)
  enable_testing()

  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/test_proto_reverse_projection.cpp")
    add_executable(test_proto_reverse_projection
      "${CMAKE_CURRENT_SOURCE_DIR}/test/test_proto_reverse_projection.cpp")
    target_link_libraries(test_proto_reverse_projection PRIVATE
      ${PROJECT_NAME} GTest::gtest GTest::gtest_main)
    gtest_discover_tests(test_proto_reverse_projection)
  endif()
endif()

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)

# 导出成**命名空间目标**而不是经典变量，与仓库内 amr_dispatcher_bt 一致。
#
# 理由：下游有两个消费者包（amr_dispatcher_ros 的 INTERFACE 目标、
# amr_dispatcher_tools 的可执行目标），用 target 形式才能自动携带
# INTERFACE_INCLUDE_DIRECTORIES 与 INTERFACE_LINK_LIBRARIES（含 Protobuf），
# 不必让每个下游自己再拼一遍 Protobuf_INCLUDE_DIRS。
#
# HAS_LIBRARY_TARGET：本包导出的是真实共享库（不是 header-only），
# 让 ament 在 target 上挂 IMPORTED_LOCATION。
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  std_msgs
  amr_dispatcher_interfaces
  amr_dispatcher_proto
  Protobuf
)

ament_package()
