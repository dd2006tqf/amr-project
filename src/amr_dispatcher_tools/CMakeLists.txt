cmake_minimum_required(VERSION 3.16)
project(amr_dispatcher_tools)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(amr_dispatcher_core REQUIRED)
find_package(amr_dispatcher_interfaces REQUIRED)

# 1. dispatcher_cli
add_executable(dispatcher_cli
  src/dispatcher_cli.cpp
)
target_include_directories(dispatcher_cli PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(dispatcher_cli
  rclcpp
  amr_dispatcher_core
  amr_dispatcher_interfaces
)

# 2. rest_gateway
add_executable(rest_gateway
  src/rest_gateway.cpp
)
target_include_directories(rest_gateway PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(rest_gateway
  rclcpp
  ament_index_cpp
  std_msgs
  amr_dispatcher_core
  amr_dispatcher_interfaces
)

install(TARGETS
  dispatcher_cli
  rest_gateway
  DESTINATION lib/${PROJECT_NAME}
)

# 单元测试：把网关里"会打死进程"的解析逻辑抽成纯函数后单测覆盖。
# 模式与 amr_dispatcher_ros 的 ROS_UNIT_TESTS 一致：header-only 接口库 + gtest。
if(BUILD_TESTING)
  find_package(GTest REQUIRED)
  include(GoogleTest)
  enable_testing()

  add_library(amr_dispatcher_tools_helpers INTERFACE)
  target_include_directories(amr_dispatcher_tools_helpers INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

  set(TOOLS_UNIT_TESTS
    test_http_limits
    test_bearer_auth
    test_idempotency_store
    test_json_escape
  )
  foreach(test_name IN LISTS TOOLS_UNIT_TESTS)
    set(test_src "${CMAKE_CURRENT_SOURCE_DIR}/test/${test_name}.cpp")
    if(EXISTS "${test_src}")
      add_executable(${test_name} "${test_src}")
      target_link_libraries(${test_name} PRIVATE
        amr_dispatcher_tools_helpers GTest::gtest GTest::gtest_main)
      gtest_discover_tests(${test_name})
    endif()
  endforeach()
endif()

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
