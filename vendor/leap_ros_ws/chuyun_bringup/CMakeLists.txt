cmake_minimum_required(VERSION 3.8)
project(chuyun_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

set(dependencies
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_msgs
  tf2_ros
)

add_executable(chuyun_bringup src/chuyun_bringup.cpp)
target_include_directories(chuyun_bringup PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(chuyun_bringup PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(chuyun_bringup
  ${dependencies}
)

add_executable(ekf_input_adapter src/ekf_input_adapter.cpp)
target_compile_features(ekf_input_adapter PUBLIC cxx_std_17)
ament_target_dependencies(ekf_input_adapter
  rclcpp
  nav_msgs
  sensor_msgs
)

add_executable(web_tf_aggregator src/web_tf_aggregator.cpp)
target_compile_features(web_tf_aggregator PUBLIC cxx_std_17)
ament_target_dependencies(web_tf_aggregator
  geometry_msgs
  rclcpp
  tf2_msgs
)

add_executable(web_odom_throttle src/web_odom_throttle.cpp)
target_compile_features(web_odom_throttle PUBLIC cxx_std_17)
ament_target_dependencies(web_odom_throttle
  nav_msgs
  rclcpp
)

install(TARGETS chuyun_bringup ekf_input_adapter web_tf_aggregator web_odom_throttle
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
