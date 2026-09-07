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

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
