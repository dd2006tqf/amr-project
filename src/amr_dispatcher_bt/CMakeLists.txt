cmake_minimum_required(VERSION 3.16)
project(amr_dispatcher_bt VERSION 1.0.0 LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(amr_dispatcher_core REQUIRED)
find_package(behaviortree_cpp REQUIRED)

add_library(${PROJECT_NAME}
  src/docking_behavior_tree.cpp
  src/facility_behavior_tree.cpp
  src/mission_queue_behavior_tree.cpp
  src/mission_recovery_behavior_tree.cpp
  src/mission_resource_cleanup_behavior_tree.cpp
  src/mission_result_behavior_tree.cpp
  src/mission_runtime_behavior_tree.cpp
  src/queued_mission_start_behavior_tree.cpp
  src/station_sequence_behavior_tree.cpp
  src/submit_order_behavior_tree.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  ${amr_dispatcher_core_LIBRARIES}
  amr_dispatcher_core::amr_dispatcher_core
  behaviortree_cpp::behaviortree_cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)

if(BUILD_TESTING)
  find_package(ament_lint_auto QUIET)
  find_package(GTest REQUIRED)
  include(GoogleTest)

  set(BT_TESTS
    docking_behavior_tree_test
    facility_behavior_tree_test
    mission_queue_behavior_tree_test
    mission_recovery_behavior_tree_test
    mission_resource_cleanup_behavior_tree_test
    mission_result_behavior_tree_test
    mission_runtime_behavior_tree_test
    queued_mission_start_behavior_tree_test
    station_sequence_behavior_tree_test
    submit_order_behavior_tree_test
  )

  foreach(test_name IN LISTS BT_TESTS)
    set(test_src "${CMAKE_CURRENT_SOURCE_DIR}/test/${test_name}.cpp")
    if(EXISTS "${test_src}")
      add_executable(${test_name} "${test_src}")
      target_link_libraries(${test_name} PRIVATE ${PROJECT_NAME} GTest::gtest GTest::gtest_main)
      gtest_discover_tests(${test_name})
    endif()
  endforeach()
endif()

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(amr_dispatcher_core behaviortree_cpp)
ament_package()
