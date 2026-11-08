cmake_minimum_required(VERSION 3.8)
project(chuyun_opencv)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)

add_executable(opencv_demo_node src/opencv_demo_node.cpp)
target_compile_features(opencv_demo_node PUBLIC cxx_std_17)
target_include_directories(opencv_demo_node PRIVATE ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(opencv_demo_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
)
target_link_libraries(opencv_demo_node ${OpenCV_LIBRARIES})

install(TARGETS opencv_demo_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config README.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
