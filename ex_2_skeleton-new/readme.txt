Drone Mapper - Assignment 2
===========================
TAU Advanced Topics in Programming 2026B
Student: omri-boiman  (boiman.omri@gmail.com)

--------------------------------------------------------------------------------
BUILD INSTRUCTIONS
--------------------------------------------------------------------------------

Prerequisites: CMake >= 3.25, a C++23-capable compiler, vcpkg.

  cmake --preset default
  cmake --build --preset default

Build targets produced under build/:
  drone_mapper_simulation       – main simulator executable
  drone_mapper_simulation_test  – GTest test suite
  maps_comparison               – standalone map-comparison utility

--------------------------------------------------------------------------------
RUNNING THE SIMULATION
--------------------------------------------------------------------------------

  ./build/drone_mapper_simulation [<simulation.yaml>] [<output_path>]

  <simulation.yaml>
    Path to the simulation composition YAML file.
    Defaults to ./simulation.yaml in the current working directory.
    Supports: no argument, filename only, relative path, or absolute path.

  <output_path>
    Directory where all output files are written.
    Defaults to the current working directory.
    Created automatically if it does not exist.

IMPORTANT — relative map paths:
  The simulation YAML files reference map .npy files with paths relative to the
  directory from which the program is run.  When using the professor's inputs/
  directory, run from inside it:

    cd inputs
    ../build/drone_mapper_simulation sim_compose.yaml ../output_results/

  For the built-in test scenarios (sc1, sc2, sc3, benchmark) run from the
  project root:

    ./build/drone_mapper_simulation scenarios/scenario1/simulation.yaml sc1_out/
    ./build/drone_mapper_simulation scenarios/benchmark/simulation.yaml benchmark_out/

--------------------------------------------------------------------------------
RUNNING THE MAPS COMPARISON UTILITY
--------------------------------------------------------------------------------

  ./build/maps_comparison <origin_map> <target_map> [comparison_config=<path>]

  Prints a single floating-point number (0–100) to stdout — nothing else.
  Prints -1 to stdout and a descriptive message to stderr on error.

  <origin_map> and <target_map> are .npy file paths (relative or absolute).
  comparison_config is an optional path to a YAML file with resolution/offset
  overrides.  If omitted, both maps are assumed to share the same resolution
  and offset.

--------------------------------------------------------------------------------
RUNNING THE TESTS
--------------------------------------------------------------------------------

Run all tests:
  ./build/drone_mapper_simulation_test

Run specific component/integration suites (--gtest_filter):
  ./build/drone_mapper_simulation_test --gtest_filter=Integration.*
  ./build/drone_mapper_simulation_test --gtest_filter=SimulationManager.*
  ./build/drone_mapper_simulation_test --gtest_filter=SimulationRun.*
  ./build/drone_mapper_simulation_test --gtest_filter=MissionControl.*
  ./build/drone_mapper_simulation_test --gtest_filter=DroneControl.*
  ./build/drone_mapper_simulation_test --gtest_filter=MappingAlgorithm.*
  ./build/drone_mapper_simulation_test --gtest_filter=MockLidar.*
  ./build/drone_mapper_simulation_test --gtest_filter=MapsComparison.*

All 78 tests pass on a clean build.

--------------------------------------------------------------------------------
simulation_output.yaml FORMAT
--------------------------------------------------------------------------------

Written to <output_path>/simulation_output.yaml.

score_report:
  composition_file: "path/to/composition.yaml"
  generated_at_utc: "2026-06-10T12:00:00Z"
  metric: "output_map_accuracy"
  score_range:
    min: 0
    max: 100
  error_score: -1
  summary:
    total_runs: 12
    scored_runs: 10
    error_runs: 2
    average_score: 87.4
    min_score: 61.2
    max_score: 98.9
  simulations:
    - simulation_config: "path/to/simulation.yaml"
      missions:
        - mission_config: "path/to/mission.yaml"
          resolution_cm: 10        # actual output resolution used
          resolution_request_status: ACCEPTED   # ACCEPTED / IGNORED / IGNORED TOO SMALL
          runs:
            - drone_config: "path/to/drone.yaml"
              lidar_config: "path/to/lidar.yaml"
              status: "completed"    # completed / max_steps / error
              steps: 1231
              score: 93.5
            - drone_config: "..."
              lidar_config: "..."
              status: "error"
              steps: 42
              score: -1
              error_ref:
                code: "DRONE_HITS_OBSTACLE"

resolution_request_status values:
  ACCEPTED         – output_mapping_resolution_factor was valid (>= 1) and applied.
  IGNORED TOO SMALL – factor was < 1; output fell back to gps_resolution_cm.
  IGNORED          – factor field absent; output uses gps_resolution_cm.

--------------------------------------------------------------------------------
output_results/ FOLDER FORMAT
--------------------------------------------------------------------------------

Located at <output_path>/output_results/

  error.log
    Timestamped log of all errors during the run.
    Each error is appended immediately when it occurs (never deferred).
    Format: [UTC timestamp] <error message>

  <map_stem>_<res_cm>cm_<drone_r_cm>r_<lidar_zmax_cm>lz.npy
    Output occupancy map for each (simulation × mission × drone × lidar) run.
    map_stem     – base name of the hidden map file (without path or .npy)
    res_cm       – actual output map resolution in centimetres
    drone_r_cm   – drone radius in centimetres (integer)
    lidar_zmax_cm – lidar z_max in centimetres (integer)
    Example: scenario_small_10cm_4r_150lz.npy

--------------------------------------------------------------------------------
NOTES ON THE inputs/ DIRECTORY
--------------------------------------------------------------------------------

The inputs/ directory contains the professor's provided scenarios.  We added one
line to each of the four main mission configs:

  output_mapping_resolution_factor: 2

This makes the output map resolution (5 cm × 2 = 10 cm) match the hidden-map
resolution (10 cm), which is required for correct voxel-alignment during scoring.
Without this setting, the 5 cm output grid misaligns with the 10 cm hidden-map
grid and produces artificially low F1 scores (0% for lidar_long scenarios).

The house scenarios are unchanged and score 0% by design — the house map starts
at world z = 150 cm and the missions cap at max z = 60–150 cm, making the
interior physically inaccessible within the mission bounds.

--------------------------------------------------------------------------------
ALGORITHM IMPLEMENTATION NOTES
--------------------------------------------------------------------------------

Only MappingAlgorithmImpl.cpp and MappingAlgorithmImpl.h were modified from the
skeleton.  All changes are bracketed with [CHANGE: Fix N] / [END CHANGE: Fix N]
comments for easy review.

Fix 1 — Bootstrap navigation:
  When the drone starts outside the output-map array bounds (as in the house
  scenario, where the drone begins 140 cm below the map), the algorithm issues
  Elevate/Advance commands to navigate into range before the first scan sweep.

Fix 2 — Physical map size threshold for rotation step:
  Maps whose largest physical dimension is < 200 cm use a fine 5.625° rotation
  step (64 rotations per 360° sweep) for higher angular scan density.  Larger
  maps use 11.25° (32 rotations) to fit more frontier visits within the fixed
  step budget.
