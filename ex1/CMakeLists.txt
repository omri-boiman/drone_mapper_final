cmake_minimum_required(VERSION 3.25)
project(drone_mapper LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Generate compile_commands.json so VS Code IntelliSense finds mp-units headers
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ---------------------------------------------------------------------------
# Bootstrap Conan 2.x if not present, then run conan install
# cmake_layout places generators under <source>/build/build/<BuildType>/generators/
# ---------------------------------------------------------------------------
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(_conan_generators "${CMAKE_SOURCE_DIR}/build/${CMAKE_BUILD_TYPE}/generators")
set(_conan_toolchain  "${_conan_generators}/conan_toolchain.cmake")

if(NOT EXISTS "${_conan_toolchain}")
    # Ensure Conan is installed – install python3-pip via apt, then conan via pip
    find_program(CONAN conan)
    if(NOT CONAN)
        message(STATUS "Conan not found – bootstrapping python3-pip and Conan...")
        find_program(APT apt-get REQUIRED)

        execute_process(COMMAND sudo ${APT} update -qq)
        execute_process(
            COMMAND sudo ${APT} install -y python3-pip python3-venv
            RESULT_VARIABLE _apt_exit
        )
        if(NOT _apt_exit EQUAL 0)
            message(FATAL_ERROR "apt-get install python3-pip python3-venv failed (exit ${_apt_exit})")
        endif()

        find_program(PYTHON3 python3 REQUIRED)
        execute_process(
            COMMAND ${PYTHON3} -m pip install --break-system-packages conan
            RESULT_VARIABLE _pip_exit
        )
        if(NOT _pip_exit EQUAL 0)
            message(FATAL_ERROR "pip install conan failed (exit ${_pip_exit})")
        endif()

        # pip may install to ~/.local/bin – search that explicitly
        unset(CONAN CACHE)
        find_program(CONAN conan
            PATHS "$ENV{HOME}/.local/bin" /usr/local/bin
            REQUIRED
        )
    endif()

    # Create the default Conan profile if it does not exist yet
    if(NOT EXISTS "$ENV{HOME}/.conan2/profiles/default")
        message(STATUS "conan profile detect – creating default build profile...")
        execute_process(
            COMMAND ${CONAN} profile detect
            RESULT_VARIABLE _profile_exit
        )
        if(NOT _profile_exit EQUAL 0)
            message(FATAL_ERROR "conan profile detect failed (exit ${_profile_exit})")
        endif()
    endif()

    message(STATUS "conan install – fetching mp-units and dependencies...")
    execute_process(
        COMMAND ${CONAN} install ${CMAKE_SOURCE_DIR}
                --build=missing
                -s build_type=${CMAKE_BUILD_TYPE}
                -s compiler.cppstd=20
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        RESULT_VARIABLE _conan_exit
    )
    if(NOT _conan_exit EQUAL 0)
        message(FATAL_ERROR "conan install failed (exit ${_conan_exit})")
    endif()
endif()

list(PREPEND CMAKE_PREFIX_PATH "${_conan_generators}")
include("${_conan_toolchain}")

# ---------------------------------------------------------------------------
# Dependencies
# ---------------------------------------------------------------------------
find_package(mp-units REQUIRED)

# ---------------------------------------------------------------------------
# Shared sources (everything except the two entry-point files)
# ---------------------------------------------------------------------------
file(GLOB_RECURSE DRONE_SOURCES
    "src/io/*.cpp"
    "src/simulation/*.cpp"
    "src/drone/*.cpp"
    "src/scoring/*.cpp"
)

# ---------------------------------------------------------------------------
# drone_mapper – main simulator
# ---------------------------------------------------------------------------
add_executable(drone_mapper
    src/main.cpp
    ${DRONE_SOURCES}
)

target_include_directories(drone_mapper PRIVATE include)
target_compile_options(drone_mapper PRIVATE -Wall -Wextra -Werror -pedantic)
target_link_libraries(drone_mapper PRIVATE mp-units::mp-units)

# ---------------------------------------------------------------------------
# sensor_test – quick smoke test for Phase 2 & 3 mocks (no GTest required)
# ---------------------------------------------------------------------------
add_executable(sensor_test
    src/test_sensors.cpp
    ${DRONE_SOURCES}
)

target_include_directories(sensor_test PRIVATE include)
target_compile_options(sensor_test PRIVATE -Wall -Wextra -Werror -pedantic)
target_link_libraries(sensor_test PRIVATE mp-units::mp-units)
