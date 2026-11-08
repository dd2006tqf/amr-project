cmake_minimum_required(VERSION 3.5)
project(chuyun_navigation2)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(lite_navigation_core
  src/a_star_planner.cpp
  src/correlative_scan_matcher.cpp
  src/grid_map.cpp
  src/local_trajectory_planner.cpp
  src/path_follower.cpp
)
target_include_directories(lite_navigation_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(lite_navigation_core
  geometry_msgs
  nav_msgs
)

add_executable(lite_navigation_node
  src/lite_navigation_node.cpp
)
target_include_directories(lite_navigation_node PRIVATE include)
target_link_libraries(lite_navigation_node lite_navigation_core)
ament_target_dependencies(lite_navigation_node
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)

install(TARGETS lite_navigation_core lite_navigation_node
  EXPORT export_lite_navigation_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_lite_navigation_core HAS_LIBRARY_TARGET)
ament_export_dependencies(geometry_msgs nav_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(core_smoke_test test/core_smoke_test.cpp)
  target_link_libraries(core_smoke_test lite_navigation_core)
  ament_target_dependencies(core_smoke_test geometry_msgs nav_msgs)
endif()

install(
  DIRECTORY launch config param maps
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
