Drone Mapper - Assignment 2
===========================

## Running the simulation

./drone_mapper_simulation [<simulation.yaml>] [<output_path>]

  simulation.yaml  – path to the simulation composition YAML (default: ./simulation.yaml)
  output_path      – directory for all output files (default: current working directory)

## Running the maps comparison utility

./maps_comparison <origin_map> <target_map> [comparison_config=<path>]

  Prints a single float (0–100) to stdout. Prints -1 on error (with message to stderr).

## Running tests

./drone_mapper_simulation_test
./drone_mapper_simulation_test --gtest_filter=Integration.*
./drone_mapper_simulation_test --gtest_filter=SimulationManager.*
./drone_mapper_simulation_test --gtest_filter=SimulationRun.*
./drone_mapper_simulation_test --gtest_filter=MissionControl.*
./drone_mapper_simulation_test --gtest_filter=DroneControl.*
./drone_mapper_simulation_test --gtest_filter=MappingAlgorithm.*
./drone_mapper_simulation_test --gtest_filter=MockLidar.*
./drone_mapper_simulation_test --gtest_filter=MapsComparison.*

## simulation_output.yaml format

Written to <output_path>/simulation_output.yaml. Top-level key is score_report:

score_report:
  composition_file: "path/to/composition.yaml"
  generated_at_utc: "2026-06-10T12:00:00Z"
  metric: "output_map_accuracy"
  score_range:
    min: 0
    max: 100
  error_score: -1
  summary:
    total_runs: N
    scored_runs: N
    error_runs: N
    average_score: 87.4
    min_score: 61.2
    max_score: 98.9
  simulations:
    - simulation_config: "path/to/sim.yaml"
      missions:
        - mission_config: "path/to/mission.yaml"
          resolution_cm: 10
          resolution_request_status: ACCEPTED   # or IGNORED / IGNORED TOO SMALL
          runs:
            - drone_config: "path/to/drone.yaml"
              lidar_config: "path/to/lidar.yaml"
              status: "completed"               # completed / max_steps / error
              steps: 1231
              score: 93.5
            - drone_config: ...
              lidar_config: ...
              status: "error"
              steps: 42
              score: -1
              error_ref:
                code: "DRONE_HITS_OBSTACLE"

## output_results/ folder format

Located at <output_path>/output_results/

  error.log
    – timestamped log of all errors during the run; appended immediately when each
      error occurs (never deferred).

  <map_stem>_<gps_res_cm>cm_<drone_dim_cm>d_<lidar_zmax_cm>lz.npy
    – output occupancy map for each (simulation × mission × drone × lidar) run.
      Example: scenario1_map_10cm_30d_120lz.npy
