set(SUBMITTER_IDS "211781141_325049575")

# Standalone-buildable: only set up project()/toolchain when this is the actual
# CMake source root (not when add_subdirectory'd from the repo root, which already
# did this setup and already has common::common available).
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    cmake_minimum_required(VERSION 3.15)
    project(Algorithm_${SUBMITTER_IDS} LANGUAGES CXX)
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

add_library(Algorithm_${SUBMITTER_IDS} SHARED
    src/MappingAlgorithmImpl.cpp
)
set_target_properties(Algorithm_${SUBMITTER_IDS} PROPERTIES
    PREFIX ""
    OUTPUT_NAME "Algorithm_${SUBMITTER_IDS}"
)
target_include_directories(Algorithm_${SUBMITTER_IDS} PUBLIC include)
target_include_directories(Algorithm_${SUBMITTER_IDS} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../UserCommon/include
)
target_link_libraries(Algorithm_${SUBMITTER_IDS} PRIVATE common::common)
drone_warnings(Algorithm_${SUBMITTER_IDS})

# Optional dev-only test target -- not part of the required Algorithm.so
# deliverable, skipped entirely if gtest isn't available so this project still
# builds standalone for anyone (e.g. another team's Simulator) who just wants
# the .so. Recompiles MappingAlgorithmImpl.cpp into the test binary alongside
# a stub MappingAlgorithmRegistration constructor (the real definition only
# exists in the Simulator project) so the REGISTER_MAPPING_ALGORITHM call
# still links.
find_package(GTest CONFIG)
if(GTest_FOUND)
    enable_testing()
    add_executable(algorithm_test
        src/MappingAlgorithmImpl.cpp
        tests/RegistrationStub.cpp
        tests/test_mapping_algorithm.cpp
    )
    target_include_directories(algorithm_test PRIVATE include
        ${CMAKE_CURRENT_SOURCE_DIR}/../UserCommon/include
    )
    target_link_libraries(algorithm_test PRIVATE common::common GTest::gtest GTest::gtest_main GTest::gmock)
    include(GoogleTest)
    gtest_discover_tests(algorithm_test)
endif()
