set(SUBMITTER_IDS "211781141_325049575")

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    cmake_minimum_required(VERSION 3.15)
    project(simulator_${SUBMITTER_IDS} LANGUAGES CXX)
    set(CMAKE_CXX_STANDARD 20)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
    find_package(mp-units CONFIG REQUIRED)
    function(drone_warnings target)
        target_compile_options(${target} PRIVATE -Wall -Wextra -Werror -pedantic)
    endfunction()
endif()

find_package(tinynpy CONFIG REQUIRED)
find_package(yaml-cpp REQUIRED)

if(NOT TARGET common::common)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../common ${CMAKE_CURRENT_BINARY_DIR}/_common)
endif()

add_executable(simulator_${SUBMITTER_IDS}
    src/Map3DImpl.cpp
    src/MockGPS.cpp
    src/MockLidar.cpp
    src/MockMovement.cpp
    src/MapsComparison.cpp
    src/YamlConfigParser.cpp
    src/ScoreReportWriter.cpp
    src/SimulationManager.cpp
    src/SimulationRunImpl.cpp
    src/SimulationRunFactoryImpl.cpp
    src/CliArgs.cpp
    src/AggregateReport.cpp
    src/plugin/PluginLoader.cpp
    src/plugin/MappingAlgorithmRegistration.cpp
    src/plugin/MissionControlRegistration.cpp
    src/main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../UserCommon/src/ErrorLogger.cpp
)
target_include_directories(simulator_${SUBMITTER_IDS} PRIVATE
    common_simulator/include
    include
    src
    ${CMAKE_CURRENT_SOURCE_DIR}/../UserCommon/include
)
target_link_libraries(simulator_${SUBMITTER_IDS} PRIVATE
    common::common
    TinyNPY::TinyNPY
    yaml-cpp::yaml-cpp
    ${CMAKE_DL_LIBS}
)
set_target_properties(simulator_${SUBMITTER_IDS} PROPERTIES ENABLE_EXPORTS ON)
target_link_options(simulator_${SUBMITTER_IDS} PRIVATE -rdynamic)
drone_warnings(simulator_${SUBMITTER_IDS})

# Optional dev-only test target -- see Algorithm/CMakeLists.txt for the same
# pattern. No registration stub needed here: nothing in Simulator's own test
# code calls the REGISTER_* macros (those only appear in the Algorithm/
# MissionControl projects), so main.cpp and plugin/* are simply excluded.
find_package(GTest CONFIG)
if(GTest_FOUND)
    enable_testing()
    add_executable(simulator_test
        src/Map3DImpl.cpp
        src/MockGPS.cpp
        src/MockLidar.cpp
        src/MockMovement.cpp
        src/MapsComparison.cpp
        src/YamlConfigParser.cpp
        src/ScoreReportWriter.cpp
        src/SimulationManager.cpp
        src/SimulationRunImpl.cpp
        src/SimulationRunFactoryImpl.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../UserCommon/src/ErrorLogger.cpp
        tests/test_mock_lidar.cpp
        tests/test_maps_comparison.cpp
        tests/test_simulation_manager.cpp
        tests/test_simulation_run.cpp
        tests/test_score_report_writer.cpp
        tests/test_integration_pipeline.cpp
    )
    target_include_directories(simulator_test PRIVATE
        common_simulator/include
        include
        tests
        ${CMAKE_CURRENT_SOURCE_DIR}/../UserCommon/include
    )
    target_link_libraries(simulator_test PRIVATE
        common::common TinyNPY::TinyNPY yaml-cpp::yaml-cpp
        GTest::gtest GTest::gtest_main GTest::gmock
    )
    include(GoogleTest)
    gtest_discover_tests(simulator_test WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/tests)
endif()
