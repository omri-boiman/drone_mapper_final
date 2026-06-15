cmake_minimum_required(VERSION 3.20)

project(cpp_course LANGUAGES CXX)

find_package(mp-units CONFIG REQUIRED)
find_package(tinynpy CONFIG REQUIRED)
find_package(GTest CONFIG REQUIRED)

add_library(cpp_course
    src/MockLidarSensor.cpp
    src/VoxelGrid.cpp
)

target_compile_features(cpp_course PUBLIC cxx_std_20)

target_include_directories(cpp_course
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(cpp_course
    PUBLIC
        mp-units::mp-units
        TinyNPY::TinyNPY
)

add_executable(mock_lidar_test examples/mock_lidar_test.cpp)
target_link_libraries(mock_lidar_test PRIVATE cpp_course)

enable_testing()

add_executable(cpp_course_tests
    tests/MockLidarSensorTests.cpp
    tests/VoxelGridTests.cpp
)
target_link_libraries(cpp_course_tests
    PRIVATE
        cpp_course
        GTest::gtest
        GTest::gtest_main
)
target_compile_definitions(cpp_course_tests
    PRIVATE
        CPP_COURSE_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)

include(GoogleTest)
gtest_discover_tests(cpp_course_tests)
