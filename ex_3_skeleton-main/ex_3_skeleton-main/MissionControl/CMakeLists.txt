set(SUBMITTER_IDS "211781141_325049575")

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    cmake_minimum_required(VERSION 3.15)
    project(MissionControl_${SUBMITTER_IDS} LANGUAGES CXX)
    if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
        set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
    endif()
    set(CMAKE_CXX_STANDARD 20)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
    find_package(mp-units CONFIG REQUIRED)
    function(drone_warnings target)
        target_compile_options(${target} PRIVATE -Wall -Wextra -Werror -pedantic)
    endfunction()
endif()

if(NOT TARGET common::common)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../common ${CMAKE_CURRENT_BINARY_DIR}/_common)
endif()

add_library(MissionControl_${SUBMITTER_IDS} SHARED
    src/DroneControlImpl.cpp
    src/ScanResultToVoxels.cpp
    src/MissionControlImpl.cpp
)
set_target_properties(MissionControl_${SUBMITTER_IDS} PROPERTIES
    PREFIX ""
    OUTPUT_NAME "MissionControl_${SUBMITTER_IDS}"
)
target_include_directories(MissionControl_${SUBMITTER_IDS} PRIVATE
    common_mission_control/include
    include
    ${CMAKE_CURRENT_SOURCE_DIR}/../UserCommon/include
)
target_link_libraries(MissionControl_${SUBMITTER_IDS} PRIVATE common::common)
drone_warnings(MissionControl_${SUBMITTER_IDS})

# Optional dev-only test target -- see Algorithm/CMakeLists.txt for the same
# pattern and rationale (skipped if gtest unavailable, stub registration ctor).
find_package(GTest CONFIG)
if(GTest_FOUND)
    enable_testing()
    add_executable(mission_control_test
        src/DroneControlImpl.cpp
        src/ScanResultToVoxels.cpp
        src/MissionControlImpl.cpp
        tests/RegistrationStub.cpp
        tests/test_drone_control.cpp
        tests/test_mission_control.cpp
    )
    target_include_directories(mission_control_test PRIVATE
        common_mission_control/include
        include
        ${CMAKE_CURRENT_SOURCE_DIR}/../UserCommon/include
    )
    target_link_libraries(mission_control_test PRIVATE common::common GTest::gtest GTest::gtest_main GTest::gmock)
    include(GoogleTest)
    gtest_discover_tests(mission_control_test)
endif()
