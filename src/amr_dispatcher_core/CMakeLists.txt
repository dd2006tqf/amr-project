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
  src/catalog/station_catalog.cpp
  src/catalog/scenario_catalog.cpp
  src/catalog/business_order_catalog.cpp
  src/catalog/zone_catalog.cpp
  src/fleet/fleet_catalog.cpp
  src/fleet/fleet_runtime_state.cpp
  src/facility/facility_catalog.cpp
  src/facility/facility_reservation.cpp
  src/facility/facility_workflow.cpp
  src/workflow/mission_profile.cpp
  src/workflow/order_payload.cpp
  src/workflow/fulfillment_workflow.cpp
  src/workflow/payload_workflow.cpp
  src/workflow/mission_runtime_workflow.cpp
  src/workflow/mission_result_workflow.cpp
  src/workflow/mission_safety_workflow.cpp
  src/workflow/mission_recovery_policy.cpp
  src/workflow/mission_event_log.cpp
  src/workflow/traffic_planning.cpp
  src/workflow/workflow_control.cpp
  src/workflow/mission_estimator.cpp
  src/workflow/mission_preflight.cpp
  src/workflow/station_transport_workflow.cpp
  src/workflow/station_sequence_workflow.cpp
  src/workflow/docking_workflow.cpp
  src/workflow/mission_charging_workflow.cpp
  src/workflow/mission_resource_cleanup.cpp
  src/workflow/queued_mission.cpp
  src/workflow/mission_profile_builders.cpp
  src/workflow/operator_snapshot.cpp
  src/workflow/dock_catalog.cpp
  src/workflow/submit_order_router.cpp
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
    test_mock_linear_planner
    test_mission_profile
    test_mission_profile_builders
    test_queued_mission
    test_order_payload
    test_fulfillment_workflow
    test_payload_workflow
    test_mission_runtime_workflow
    test_mission_result_workflow
    test_mission_safety_workflow
    test_mission_recovery_policy
    test_workflow_mission_event_log
    test_traffic_planning
    test_workflow_control
    test_station_transport_workflow
    test_station_sequence_workflow
    test_docking_workflow
    test_mission_charging_workflow
    test_mission_resource_cleanup
    test_mission_preflight
    test_operator_snapshot
    test_dock_catalog
    test_station_catalog
    test_scenario_catalog
    test_business_order_catalog
    test_zone_catalog
    test_fleet_catalog
    test_fleet_runtime_state
    test_facility_catalog
    test_facility_reservation
    test_facility_workflow
    test_submit_order_router
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
