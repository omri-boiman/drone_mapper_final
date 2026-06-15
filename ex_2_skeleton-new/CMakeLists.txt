cmake_minimum_required(VERSION 3.15)
project(drone_mapper LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)


find_package(mp-units CONFIG REQUIRED)
find_package(tinynpy CONFIG REQUIRED)
find_package(yaml-cpp REQUIRED) # new!

add_library(drone_mapper
    src/DroneControlImpl.cpp
    src/MappingAlgorithmImpl.cpp
    src/MapsComparison.cpp
    src/MissionControlImpl.cpp
    src/Map3DImpl.cpp
    src/MockGPS.cpp
    src/MockLidar.cpp
    src/MockMovement.cpp
    src/ScanResultToVoxels.cpp
    src/SimulationManager.cpp
    src/SimulationRunFactoryImpl.cpp
    src/SimulationRunImpl.cpp
    src/ErrorLogger.cpp
    src/ScoreReportWriter.cpp
    src/YamlConfigParser.cpp
)

target_compile_features(drone_mapper PUBLIC cxx_std_20)
target_compile_options(drone_mapper
    PRIVATE
        -Wall
        -Wextra
        -Werror
        -pedantic
)
target_include_directories(drone_mapper
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(drone_mapper
    PUBLIC
        mp-units::mp-units
        TinyNPY::TinyNPY
        yaml-cpp::yaml-cpp
)

add_executable(drone_mapper_simulation src/drone_mapper_simulation_main.cpp)
target_link_libraries(drone_mapper_simulation PRIVATE drone_mapper)
#Compile the maps_comparison util
add_executable(maps_comparison src/maps_comparison_main.cpp)
target_link_libraries(maps_comparison PRIVATE drone_mapper)

find_package(GTest CONFIG REQUIRED)
enable_testing()
add_executable(drone_mapper_simulation_test
    tests/components/test_simulation_manager.cpp
    tests/components/test_simulation_run.cpp
    tests/components/test_mission_control.cpp
    tests/components/test_drone_control.cpp
    tests/components/test_mapping_algorithm.cpp
    tests/components/test_mock_lidar.cpp
    tests/components/test_maps_comparison.cpp
    tests/integration/test_integration_real.cpp
    tests/integration/test_integration_mock_algo.cpp
)
target_link_libraries(drone_mapper_simulation_test PRIVATE drone_mapper GTest::gtest GTest::gtest_main GTest::gmock)
include(GoogleTest)
gtest_discover_tests(drone_mapper_simulation_test)

# Compile the yaml example file
add_executable(example_yml cpp_yaml_example/main.cpp)
target_link_libraries(example_yml PRIVATE yaml-cpp::yaml-cpp)
