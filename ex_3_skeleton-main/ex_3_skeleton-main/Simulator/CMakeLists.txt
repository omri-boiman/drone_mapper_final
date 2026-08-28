set(SUBMITTER_IDS "211781141_325049575")

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    cmake_minimum_required(VERSION 3.15)
    project(simulator_${SUBMITTER_IDS} LANGUAGES CXX)
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
    src/RunModes.cpp
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
# MissionControl projects), so main.cpp and plugin/* are simply excluded --
# EXCEPT plugin/PluginLoader.cpp, which RunModes.cpp needs at link time
# (RunModes.o references loadAlgorithmPlugin/loadMissionControlPlugin even
# though this target's own tests only call the pure helpers in the same file,
# e.g. partitionWork -- PluginLoader.cpp itself needs no ENABLE_EXPORTS/
# -rdynamic, since nothing here actually dlopens a real .so).
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
        src/CliArgs.cpp
        src/AggregateReport.cpp
        src/RunModes.cpp
        src/plugin/PluginLoader.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../UserCommon/src/ErrorLogger.cpp
        tests/test_mock_lidar.cpp
        tests/test_maps_comparison.cpp
        tests/test_simulation_manager.cpp
        tests/test_simulation_run.cpp
        tests/test_score_report_writer.cpp
        tests/test_integration_pipeline.cpp
        tests/test_cli_args.cpp
        tests/test_aggregate_report.cpp
        tests/test_partition_work.cpp
        tests/test_user_common.cpp
        tests/test_yaml_config_parser.cpp
    )
    target_include_directories(simulator_test PRIVATE
        common_simulator/include
        include
        src
        tests
        ${CMAKE_CURRENT_SOURCE_DIR}/../UserCommon/include
    )
    target_link_libraries(simulator_test PRIVATE
        common::common TinyNPY::TinyNPY yaml-cpp::yaml-cpp
        GTest::gtest GTest::gtest_main GTest::gmock
        ${CMAKE_DL_LIBS}
    )
    include(GoogleTest)
    gtest_discover_tests(simulator_test WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/tests)

    # Optional second dev-only test target: real-plugin integration tests that
    # dlopen the ACTUAL Algorithm_${SUBMITTER_IDS}.so / MissionControl_
    # ${SUBMITTER_IDS}.so (plus purpose-built fixture .so's for controllable
    # multi-plugin scenarios -- see tests/fixtures/), same as a real
    # simulator_${SUBMITTER_IDS} run -- unlike simulator_test above, which
    # only exercises the wiring with stub factories (see tests/StubPlugins.h).
    # Covers: real-algorithm benchmark runs (test_integration_benchmark.cpp),
    # multi-plugin comparative/competition orchestration, plugin-load-failure
    # handling, threading correctness, and a dlclose stress test
    # (test_multiplugin.cpp), and real end-to-end CLI black-box runs of the
    # built executable itself (test_cli_end_to_end.cpp). Only buildable when
    # this project is built via the root CMakeLists.txt alongside its sibling
    # Algorithm/MissionControl projects (guarded on their targets existing),
    # since a standalone Simulator build has no Algorithm/MissionControl
    # source to produce those .so's from.
    if(TARGET Algorithm_${SUBMITTER_IDS} AND TARGET MissionControl_${SUBMITTER_IDS})
        # Small, deterministic, purpose-built plugins for the multi-plugin
        # tests -- see tests/fixtures/FixtureMissionControl.cpp and
        # FixtureAlgorithm.cpp for why these produce controllable totals
        # without needing real navigation/scan geometry. Dev-only, never
        # part of the 3 required deliverables.
        function(add_fixture_plugin_so target_name source_file steps class_name)
            add_library(${target_name} SHARED ${source_file})
            set_target_properties(${target_name} PROPERTIES
                PREFIX ""
                OUTPUT_NAME "${target_name}"
            )
            target_compile_definitions(${target_name} PRIVATE
                FIXTURE_STEPS=${steps}
                FIXTURE_CLASS_NAME=${class_name}
            )
            target_link_libraries(${target_name} PRIVATE common::common)
        endfunction()

        add_fixture_plugin_so(fixture_mc_a tests/fixtures/FixtureMissionControl.cpp 5 FixtureMcA)
        add_fixture_plugin_so(fixture_mc_b tests/fixtures/FixtureMissionControl.cpp 5 FixtureMcB)
        add_fixture_plugin_so(fixture_mc_c tests/fixtures/FixtureMissionControl.cpp 9 FixtureMcC)
        add_fixture_plugin_so(fixture_algo_a tests/fixtures/FixtureAlgorithm.cpp 2 FixtureAlgoA)
        add_fixture_plugin_so(fixture_algo_b tests/fixtures/FixtureAlgorithm.cpp 4 FixtureAlgoB)
        add_fixture_plugin_so(fixture_algo_c tests/fixtures/FixtureAlgorithm.cpp 6 FixtureAlgoC)

        add_executable(simulator_integration_real_test
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
            src/RunModes.cpp
            src/plugin/PluginLoader.cpp
            src/plugin/MappingAlgorithmRegistration.cpp
            src/plugin/MissionControlRegistration.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/../UserCommon/src/ErrorLogger.cpp
            tests/test_integration_benchmark.cpp
            tests/test_multiplugin.cpp
            tests/test_cli_end_to_end.cpp
        )
        target_include_directories(simulator_integration_real_test PRIVATE
            common_simulator/include
            include
            src
            tests
            ${CMAKE_CURRENT_SOURCE_DIR}/../UserCommon/include
        )
        target_link_libraries(simulator_integration_real_test PRIVATE
            common::common TinyNPY::TinyNPY yaml-cpp::yaml-cpp
            GTest::gtest GTest::gtest_main GTest::gmock
            ${CMAKE_DL_LIBS}
        )
        # Needs the same dlopen-symbol-resolution setup as the real executable:
        # the registration constructor definitions linked into this binary must
        # be visible to dlopen() when it resolves the .so's undefined symbol.
        set_target_properties(simulator_integration_real_test PROPERTIES ENABLE_EXPORTS ON)
        target_link_options(simulator_integration_real_test PRIVATE -rdynamic)
        target_compile_definitions(simulator_integration_real_test PRIVATE
            REAL_ALGORITHM_SO_PATH="$<TARGET_FILE:Algorithm_${SUBMITTER_IDS}>"
            REAL_MISSION_CONTROL_SO_PATH="$<TARGET_FILE:MissionControl_${SUBMITTER_IDS}>"
            REAL_SIMULATOR_EXE_PATH="$<TARGET_FILE:simulator_${SUBMITTER_IDS}>"
            FIXTURE_MC_A_SO_PATH="$<TARGET_FILE:fixture_mc_a>"
            FIXTURE_MC_B_SO_PATH="$<TARGET_FILE:fixture_mc_b>"
            FIXTURE_MC_C_SO_PATH="$<TARGET_FILE:fixture_mc_c>"
            FIXTURE_ALGO_A_SO_PATH="$<TARGET_FILE:fixture_algo_a>"
            FIXTURE_ALGO_B_SO_PATH="$<TARGET_FILE:fixture_algo_b>"
            FIXTURE_ALGO_C_SO_PATH="$<TARGET_FILE:fixture_algo_c>"
        )
        # target_compile_definitions with $<TARGET_FILE:...> does not itself
        # order the build -- make it explicit so every referenced binary
        # exists before this test binary tries to dlopen/exec it.
        add_dependencies(simulator_integration_real_test
            Algorithm_${SUBMITTER_IDS} MissionControl_${SUBMITTER_IDS}
            simulator_${SUBMITTER_IDS}
            fixture_mc_a fixture_mc_b fixture_mc_c
            fixture_algo_a fixture_algo_b fixture_algo_c)
        gtest_discover_tests(simulator_integration_real_test
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/tests)
    endif()
endif()
