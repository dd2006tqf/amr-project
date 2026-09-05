cmake_minimum_required(VERSION 3.16)
project(amr_dispatcher_core VERSION 1.0.0 LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake QUIET)
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)

add_library(${PROJECT_NAME}
  src/chassis/chassis_packet.cpp
  src/chassis/chassis_packet_stream.cpp
  src/chassis/chassis_kinematics.cpp
  src/chassis/chassis_backend_factory.cpp
  src/chassis/mock_backend.cpp
  src/chassis/serial_backend.cpp
  src/chassis/udp_backend.cpp
  src/chassis/link_quality.cpp
  src/chassis/link_recovery.cpp
  src/chassis/backend_degradation.cpp
  src/path_tracking/pure_pursuit.cpp
  src/path_tracking/stanley.cpp
  src/dispatcher/mission_queue.cpp
  src/dispatcher/mission_state.cpp
  src/dispatcher/mission_comparator.cpp
  src/dispatcher/traffic_reservation.cpp
  src/dispatcher/mission_event_log.cpp
  src/dispatcher/deadlock_detector.cpp
  src/dispatcher/recovery_policy.cpp
  src/dispatcher/station_topology.cpp
  src/dispatcher/mission_cost_estimator.cpp
  src/dispatcher/dock_catalog.cpp
  src/safety/cmd_vel_gate.cpp
  src/safety/watchdog.cpp
  src/safety/safety_sources.cpp
  src/safety/fault_supervisor.cpp
  src/config/config_loader.cpp
  src/logging/async_logger.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME} PUBLIC yaml-cpp::yaml-cpp PRIVATE Threads::Threads)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)
add_library(amr::core ALIAS ${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(GTest REQUIRED)
  include(GoogleTest)
  enable_testing()
  set(UNIT_TESTS
    test_chassis_packet
    test_chassis_packet_stream
    test_chassis_kinematics
    test_pure_pursuit
    test_stanley
    test_mission_queue
    test_mission_state
    test_traffic_reservation
    test_mission_event_log
    test_link_quality
    test_link_recovery
    test_backend_degradation
    test_deadlock_detector
    test_recovery_policy
    test_cmd_vel_gate
    test_watchdog
    test_safety_sources
    test_config_loader
    test_logger
    test_station_topology
    test_transport_and_diagnostics
    test_end_to_end_pipeline
  )
  foreach(test_name IN LISTS UNIT_TESTS)
    if(test_name STREQUAL "test_end_to_end_pipeline")
      set(test_src "${CMAKE_CURRENT_SOURCE_DIR}/../../test/integration/${test_name}.cpp")
    else()
      set(test_src "${CMAKE_CURRENT_SOURCE_DIR}/../../test/unit/${test_name}.cpp")
    endif()
    if(EXISTS "${test_src}")
      add_executable(${test_name} "${test_src}")
      target_link_libraries(${test_name} PRIVATE ${PROJECT_NAME} GTest::gtest GTest::gtest_main)
      gtest_discover_tests(${test_name})
    endif()
  endforeach()
endif()

if(ament_cmake_FOUND)
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
  ament_export_dependencies(yaml-cpp Threads)
  ament_package()
endif()
