cmake_minimum_required(VERSION 3.15)
project(assignment_3 LANGUAGES CXX)

# Default to an optimized build when none is requested -- CMake's own default
# (empty CMAKE_BUILD_TYPE) compiles at -O0, which is misleadingly slow for a
# per-step simulation loop and made an unrelated wall-clock test budget
# fragile (54s vs a 60s cap) before dropping to ~7s once optimized.
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

add_subdirectory(common)
add_subdirectory(Algorithm)
add_subdirectory(MissionControl)
add_subdirectory(Simulator)
