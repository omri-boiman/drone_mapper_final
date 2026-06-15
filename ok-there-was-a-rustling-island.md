# Migration Plan: Old Skeleton → New Skeleton (Assignment 2 Updated)

## Context

The course published a new skeleton (`ex_2_skeleton-new/`) on 9.6.26 with breaking API changes:
- `IMap3D` refactored: `get()` → `atVoxel()`, `resolution()` → `getMapConfig()` returning new `MapConfig`
- `MissionConfigData` lost `boundaries` (moved to `MapConfig`)
- `MissionRunResult` lost `score`/`output_map_file` (moved to new `SimulationResult`)
- `SimulationReport` replaced by `SimulationManagerReport` + flat `SimulationResult` list
- `SimulationConfigData` gained `map_offset`
- `SimulationRunImpl::run()` now returns `SimulationResult` not `MissionRunResult`
- `MapsComparison::compare()` now takes `(origin, vector<targets>)` → `vector<double>`
- yaml-cpp added to the skeleton (replaces our hand-rolled YAML parser)
- New skeleton CMakeLists has no test target, no ErrorLogger/ScoreReportWriter, no yaml-cpp linked to main lib

**Working directory**: All work is done inside `ex_2_skeleton-new/` (new skeleton becomes our submission).
Our old implementation lives in `ex_2_skeleton-main/ex_2_skeleton-main/` — source to copy from.

---

## Phase 1 — CMakeLists.txt

**File**: `ex_2_skeleton-new/CMakeLists.txt`

Changes from the new stub:
1. Add `src/ErrorLogger.cpp` and `src/ScoreReportWriter.cpp` to `drone_mapper` library sources
2. Uncomment `yaml-cpp::yaml-cpp` in `target_link_libraries(drone_mapper ...)`
3. Add GTest block (copy from old CMakeLists):
   ```cmake
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
   ```
4. Keep `example_yml` target as-is.

---

## Phase 2 — Type Headers (copy new skeleton versions verbatim, they are already correct)

These headers are **already correct** in the new skeleton — just verify they match the spec. Do NOT copy old versions.
**CRITICAL: Do NOT modify any skeleton type header or interface header. The grader compiles against the exact skeleton API.**

- `include/drone_mapper/types/MapTypes.h` — has `MappingBounds` + `MapConfig` ✓
- `include/drone_mapper/types/MissionTypes.h` — `MissionConfigData` without boundaries, `MissionRunResult` with `vector<ErrorRef>` ✓
- `include/drone_mapper/types/SimulationTypes.h` — `SimulationResult`, `SimulationManagerReport` ✓
- `include/drone_mapper/IMap3D.h` — `atVoxel()`, `getMapConfig()` ✓
- All other interface headers (unchanged from old) ✓

### Config file path tracking (without touching skeleton types)

The output YAML needs `simulation_config: "path"`, `mission_config: "path"`, etc., but the skeleton structs have no `config_file` field. Solution:

1. Define an **internal** struct in `YamlConfigParser.h` (NOT a skeleton type):
```cpp
struct CompositionWithPaths {
    types::SimulationCompositionData data;
    std::vector<std::filesystem::path> sim_paths;
    std::vector<std::filesystem::path> mission_paths;
    std::vector<std::filesystem::path> drone_paths;
    std::vector<std::filesystem::path> lidar_paths;
};
```
2. `YamlConfigParser::parseCompositionWithPaths()` returns this.
3. `ScoreReportWriter::write()` signature becomes:
```cpp
static void write(const types::SimulationManagerReport& report,
                  const std::filesystem::path& composition_file,
                  const CompositionWithPaths& paths,
                  const std::filesystem::path& output_path);
```
4. The writer recovers per-run file paths using the deterministic iteration order:
   `run[i]` → sim_idx = `i / (n_missions * n_drones * n_lidars)`, etc.
5. `SimulationManagerReport` does NOT get a `composition_file` field — it is passed separately.

---

## Phase 3 — Implementation Class Headers

### 3a. `include/drone_mapper/Map3DImpl.h`
Take the new skeleton version verbatim. It already has:
- 3 constructors (path+res, path+res+offset, bounds+res+offset)
- `atVoxel()`, `getMapConfig()`, `set()`, `save()`
- Private: `shared_ptr<NpyArray> map_`, `MapConfig config_`

BUT also add private members needed for our implementation:
```cpp
private:
    std::shared_ptr<NpyArray> map_;
    types::MapConfig config_;
    // output map storage (bounds-based construction)
    std::vector<int8_t> output_data_;
    bool is_output_map_ = false;
    std::size_t x_size_ = 0, y_size_ = 0, z_size_ = 0;
    [[nodiscard]] std::size_t flatIndex(std::size_t xi, std::size_t yi, std::size_t zi) const noexcept;
    [[nodiscard]] bool toIndex(const Position3D& pos, std::size_t& xi, std::size_t& yi, std::size_t& zi) const noexcept;
```

### 3b. `include/drone_mapper/MockMovement.h`
Keep our 3-arg constructor (collision detection must stay in MockMovement per test requirements):
```cpp
MockMovement(MockGPS& gps, const IMap3D& hidden_map, const types::DroneConfigData& config);
```
Private members: `gps_`, `hidden_map_`, `config_` (reference + const ref + const ref).

### 3c. `include/drone_mapper/MappingAlgorithmImpl.h`
Keep the full header from old skeleton (with all private members for the BFS algorithm), but change the constructor signature to:
```cpp
explicit MappingAlgorithmImpl(types::MissionConfigData mission,
                               types::DroneConfigData drone = {},
                               types::MappingBounds bounds = {});
```
This allows `MappingAlgorithmImpl(mission)` calls from simple test code while the factory always passes all three.

### 3d. `include/drone_mapper/MapsComparison.h`
Take new skeleton version verbatim — already has correct API:
```cpp
static std::vector<double> compare(const IMap3D& origin, const std::vector<IMap3D*> targets);
```
Remove the old `ResolutionRatio` struct.

### 3e. `include/drone_mapper/SimulationRunImpl.h`
Take new skeleton version but add `types::ResolutionRequestStatus resolution_status_` private member and add it to the constructor:
```cpp
SimulationRunImpl(...all 8 unique_ptrs...,
                  types::SimulationConfigData simulation_config,
                  types::MissionConfigData mission_config,
                  std::filesystem::path output_map_file,
                  types::ResolutionRequestStatus resolution_status);
```

### 3f. New files to add (from old, unchanged):
- `include/drone_mapper/ErrorLogger.h` — copy from old skeleton unchanged
- `include/drone_mapper/ScoreReportWriter.h` — update for `SimulationManagerReport`
- `include/drone_mapper/YamlConfigParser.h` — update for new types

---

## Phase 4 — Implementation CPP Files

### 4a. `src/Map3DImpl.cpp`
Base on old implementation. Key changes:
- Hidden map constructor: load NPY, set `config_.offset = offset`, compute `config_.boundaries` from NPY shape × resolution, set `is_output_map_ = false`
- Output map constructor: set `config_ = {bounds, offset, resolution}`, allocate `output_data_`, set `is_output_map_ = true`
- Rename `get()` → `atVoxel()`, remove `resolution()` method, add `getMapConfig() { return config_; }`
- `toIndex()` must account for offset: `rx = (pos.x - config_.offset.x - config_.boundaries.min_x) / res`
- `save()`: keep TinyNPY logic from old implementation

**Critical coordinate system**: Hidden map world coord = (voxel_idx × resolution) + offset + boundaries.min. Must update `toIndex()` to subtract offset AND boundaries.min from world position.

### 4b. `src/MockMovement.cpp`
Copy from old skeleton. Update one call: `hidden_map_.get(sample)` → `hidden_map_.atVoxel(sample)`.

### 4c. `src/MockLidar.cpp`
Copy from old skeleton. Update: `map_.get(pos)` → `map_.atVoxel(pos)`. Resolution access: `map_.resolution()` → `map_.getMapConfig().resolution`.

### 4d. `src/MappingAlgorithmImpl.cpp`
Copy from old skeleton. Update `initialize()`: instead of `mission_.boundaries.min_x`, use the `bounds_` member (passed via constructor from factory). All other BFS/A* logic unchanged.

### 4e. `src/MapsComparison.cpp`
Rewrite to new API. Key logic:
- Scan range determined by `origin.getMapConfig()`: offset + boundaries define world-space scan range
- For each position in that range, call `origin.atVoxel(pos)` and `target->atVoxel(pos)`
- Compute F1 score same as before
- Repeat for each target in `targets`, return vector of scores
- Handle case where origin and target have different boundaries: use origin boundaries as scan range

```cpp
std::vector<double> MapsComparison::compare(const IMap3D& origin, const std::vector<IMap3D*> targets) {
    // scan over origin's boundary+offset range
    // for each target, compute F1 vs origin
    // return one score per target
}
```

### 4f. `src/MissionControlImpl.cpp`
Copy from old skeleton. Remove the `MapsComparison::compare()` call and score computation entirely. New `runMission()` returns:
```cpp
return types::MissionRunResult{status, steps, errors_vector};
```
Where errors is empty on success/maxsteps, has one `ErrorRef` on error (use old `error` variable).

### 4g. `src/SimulationRunImpl.cpp`
New constructor stores `resolution_status_`. New `run()` method:
```cpp
types::SimulationResult SimulationRunImpl::run() {
    types::MissionRunResult mission_result = mission_control_->runMission();
    
    double score = -1.0;
    if (mission_result.status != types::MissionRunStatus::Error) {
        auto scores = MapsComparison::compare(*hidden_map_, {output_map_.get()});
        score = scores.empty() ? -1.0 : scores[0];
    }
    
    return types::SimulationResult{
        simulation_config_,
        mission_config_,
        resolution_status_,
        {mission_result},          // vector<MissionRunResult>
        output_map_file_,
        output_map_->getMapConfig(),
        score,
    };
}
```

### 4h. `src/SimulationManager.cpp`
Update return type: `types::SimulationManagerReport`. Key changes:
- Collect `SimulationResult` (not `MissionRunResult`) into flat `runs` vector
- Exception handling: build an error `SimulationResult` (all fields default/error)
- Generate `generated_at_utc` (copy `utcNow()` from old)
- Set metric = "output_map_accuracy", score_range = {0.0, 100.0}, error_score = -1
- Remove `MissionScoreGroup` grouping logic

```cpp
types::SimulationManagerReport result;
result.generated_at_utc = utcNow();
result.metric = "output_map_accuracy";
result.score_range = {0.0, 100.0};
result.error_score = -1;

for each (sim, mission, drone, lidar) combo:
    try:
        auto run = run_factory_->create(sim, mission, drone, lidar, output_path);
        result.runs.push_back(run->run());
    catch:
        result.runs.push_back(SimulationResult{sim, mission, ..., error_status});

return result;
```

### 4i. `src/SimulationRunFactoryImpl.cpp`
Update the factory. Key changes:
1. Create hidden map with offset: `Map3DImpl(sim.map_filename, sim.map_resolution, sim.map_offset)`
2. Get hidden map config: `auto cfg = hidden_map->getMapConfig()`
3. Create output map from hidden map's boundaries + output resolution + offset: `Map3DImpl(cfg.boundaries, out_res, cfg.offset)`
4. Compute `ResolutionRequestStatus` and pass it to `SimulationRunImpl`
5. `MockMovement` still takes 3 args: `MockMovement(*gps, *hidden_map, drone)`
6. `MappingAlgorithmImpl` takes 3 args: `MappingAlgorithmImpl(mission, drone, cfg.boundaries)`
7. Pass `resolution_status` to `SimulationRunImpl` constructor

### 4j. `src/YamlConfigParser.cpp` + header
**Rewrite using yaml-cpp** (yaml-cpp is now available).

Key YAML formats to parse:
- `drone_config:` → `DroneConfigData`
- `lidar_config:` → `LidarConfigData`
- `mission_config:` (max_steps, gps_resolution_cm, output_mapping_resolution_factor) → `MissionConfigData`. Parse but ignore `boundaries` (map file drives bounds now).
- `simulation_config:` (map_filename, map_resolution_cm, map_axes_offset, initial_drone_position, initial_angle_deg) → `SimulationConfigData`
- `simulation_compositions:` (list of sims, global missions/drones/lidars) → `CompositionWithPaths` (internal struct defined in YamlConfigParser.h — NOT a skeleton type)
- `comparison_config:` (original/target with map_res_cm, map_offset, map_boundaries) → new `ComparisonConfigData` struct for maps_comparison

Use `YAML::LoadFile()` and node access. Reference: `ex_2_skeleton-new/cpp_yaml_example/main.cpp`.

The `CompositionWithPaths` struct lives in `YamlConfigParser.h` alongside the parser class. The `parseCompositionWithPaths()` method fills both `data` (the skeleton `SimulationCompositionData`) and the parallel `*_paths` vectors (one entry per config, in the same order as the data vectors).

### 4k. `src/ScoreReportWriter.cpp` + header
Update to write `SimulationManagerReport` in the required YAML format. Use yaml-cpp for writing (cleaner than manual string building).

**Do NOT modify `SimulationManagerReport`** — it has no `composition_file` field by design. Pass `composition_file` and the `CompositionWithPaths` separately.

Signature:
```cpp
static void write(const types::SimulationManagerReport& report,
                  const std::filesystem::path& composition_file,
                  const YamlConfigParser::CompositionWithPaths& paths,
                  const std::filesystem::path& output_path);
```

The writer recovers per-run file paths by exploiting the deterministic iteration order from `SimulationManager::run()` (sim × mission × drone × lidar):
```cpp
std::size_t n_s = paths.sim_paths.size();
std::size_t n_m = paths.mission_paths.size();
std::size_t n_d = paths.drone_paths.size();
std::size_t n_l = paths.lidar_paths.size();
// run[i]: sim = i/(n_m*n_d*n_l), mission = (i/(n_d*n_l))%n_m, drone = (i/n_l)%n_d, lidar = i%n_l
```

Output YAML format (from assignment spec p.6-7):
```yaml
score_report:
  composition_file: ...
  generated_at_utc: ...
  metric: "output_map_accuracy"
  score_range: {min: 0, max: 100}
  error_score: -1
  summary: {total_runs, scored_runs, error_runs, average_score, min_score, max_score}
  simulations: [grouped by sim_config → missions → runs]
```

### 4l. `src/ErrorLogger.cpp`
Copy from old skeleton unchanged.

### 4m. `src/drone_mapper_simulation_main.cpp`
Update to use `SimulationManagerReport` (was `SimulationReport`). Use `ScoreReportWriter` with new type.
YAML parsing now uses yaml-cpp based parser. Keep arg handling (composition path, output path).

### 4n. `src/maps_comparison_main.cpp`
Implement fully:
```
Usage: ./maps_comparison <origin_map> <target_map> [comparison_config=<path>]
```
- Load both maps as `Map3DImpl`
- If comparison_config provided: parse it with yaml-cpp to get MapConfig for each map
- Call `MapsComparison::compare(origin, {&target})`
- Print score (or -1 on error to stdout, error message to stderr)

---

## Phase 5 — Tests Migration

### Create `tests/` subdirectory structure:
```
tests/components/   (already exists in old skeleton)
tests/integration/  (already exists in old skeleton)
```

### Per-file changes:

**`test_simulation_manager.cpp`**:
- `MockSimRun::run()` returns `types::SimulationResult` not `MissionRunResult`
- `makeComposition()`: remove `MappingBounds b`, remove boundaries from `MissionConfigData` constructor (use `{100, 10.0*cm, 1}` aggregate init)
- `SimulationConfigData` constructor: add `Position3D{}` as 3rd arg (map_offset)
- Check `report.runs.size()` not `report.simulations.size()`
- Access score via `report.runs[0].mission_score` not `runs[0].score`
- Error check: `report.runs[0].mission_results[0].status == Error`

**`test_simulation_run.cpp`**:
- `MockMissionCtrl::runMission()` returns `MissionRunResult{status, steps, {}}` (no score)
- `SimulationRunImpl` constructor takes extra params: `simulation_cfg, mission_cfg, output_path, resolution_status`
- `MockMovement` still uses 3-arg constructor (we kept it)
- `result.steps` → `result.mission_results[0].steps`
- `result.score` → `result.mission_score`
- `AdvanceFailsWhenObstacleInPath`: still works (collision detection kept in MockMovement)
- `outputMap()`: remove `MappingBounds` from `Map3DImpl` constructor (use bounds from hidden map OR use default output bounds)

**`test_mission_control.cpp`**:
- `MissionRunResult` access: no `.score`, no `.output_map_file`; check `.status`, `.steps`, `.errors`
- `ReturnsErrorScoreMinusOneOnError`: remove score check (score is now in SimulationRunImpl)
- `MissionConfigData` construction: remove boundaries arg

**`test_drone_control.cpp`**:
- `IMap3D::get()` mock → `IMap3D::atVoxel()` (rename in MOCK_METHOD)
- `IMap3D::resolution()` mock → `IMap3D::getMapConfig()` returning MapConfig
- `MissionConfigData` construction: remove boundaries
- `Map3DImpl` bounds-based constructor: add offset param

**`test_mapping_algorithm.cpp`**:
- `MappingAlgorithmImpl` construction: update to `(mission, drone, bounds)` or just `(mission)` with defaults
- `MissionConfigData` construction: remove boundaries

**`test_mock_lidar.cpp`**:
- `MockLidar` construction uses `IMap3D&` — if using `Map3DImpl`, update constructor for new signature
- `Map3DImpl` hidden map constructor: same as before (path + resolution)
- Mock `IMap3D` (if used): rename `get` → `atVoxel`, `resolution` → `getMapConfig`

**`test_maps_comparison.cpp`**:
- Change `MapsComparison::compare(map1, map2, ratio)` → `MapsComparison::compare(map1, {&map2})[0]`
- Remove `ResolutionRatio` usage
- `Map3DImpl` output map: update constructor (add offset param `Position3D{}`)
- Hidden map used for comparison: can use `Map3DImpl(path, res)` unchanged
- `IMap3D::get()` → `IMap3D::atVoxel()` in any mock usage

**`test_integration_real.cpp`** and **`test_integration_mock_algo.cpp`**:
- `MissionRunResult` access → use `.mission_results[0].status`, `.mission_score`
- `SimulationRunImpl` constructor: add extra params
- `SimulationCompositionData` construction: remove MappingBounds, add map_offset
- `MissionConfigData`: remove boundaries

---

## Phase 6 — Data Maps & Scenario Files

Copy from old skeleton:
- `data_maps/` directory (NPY test files)
- `scenarios/` directory (YAML configs)

Update scenario YAML files: add `map_axes_offset:` block to each `simulation_config.yaml` (add zero offsets if not present):
```yaml
map_axes_offset:
  x_offset: 0
  y_offset: 0
  height_offset: 0
```

---

## Phase 7 — Missing Items from Assignment Spec

### 7a. `readme.txt`
Required by assignment (p.10): "You should document in your readme.txt the format that you have used for the simulation_output.yaml and for the output_results folder."

Create `readme.txt` in root of `ex_2_skeleton-new/` documenting:
- Structure of `simulation_output.yaml` (top-level `score_report:` with summary + hierarchical simulations)
- Structure of `output_results/` folder:
  - `error.log` — all errors during the run
  - `<map_stem>_<gps_res>cm_<drone_dim>d_<lidar_zmax>lz.npy` — output map per run

### 7b. YAML Composition: per-simulation missions vs flat list

The assignment YAML format shows **per-simulation** mission_configs. But `SimulationCompositionData.missions` is a **flat global list**. Resolution:

The YAML parser collects ALL mission file paths referenced under any simulation entry and loads them into the flat `missions` vector. `simulations` contains all simulation configs. `SimulationManager::run()` then does full cartesian product: every sim × every mission × every drone × every lidar.

This is consistent with the new skeleton's `SimulationManager::run()` four-nested-loop structure. Per-simulation mission scoping is no longer supported by the new architecture.

### 7c. `simulation_output.yaml` filename

The output file name is `simulation_output.yaml` (assignment p.10), NOT `score_report.yaml`. Update `drone_mapper_simulation_main.cpp` to write to `output_path / "simulation_output.yaml"`.

The YAML content format (assignment p.6-7) has `score_report:` as the TOP-LEVEL KEY. The `ScoreReportWriter` must group the flat `runs` vector back into a hierarchical structure by `simulation_config`:
```yaml
score_report:
  composition_file: ...
  generated_at_utc: ...
  ...
  simulations:
    - simulation_config: "..."
      missions:
        - mission_config: "..."
          resolution_cm: 10
          resolution_request_status: ACCEPTED
          runs:
            - drone_config: "..."
              lidar_config: "..."
              status: "completed"
              steps: 1231
              score: 93.5
```
The writer iterates `report.runs`, groups by `simulation_config` → `mission_config`, emits per-run data.

### 7d. `maps_comparison_main.cpp`: argument format

The third argument uses `=` syntax (assignment p.8): `comparison_config=<path>`. Parse with:
```cpp
if (argc == 4) {
    std::string arg3 = argv[3];
    if (arg3.starts_with("comparison_config=")) {
        config_path = arg3.substr(18); // after "comparison_config="
    }
}
```

### 7e. Map3DImpl: hidden map boundaries from NPY shape

When loading the hidden map with offset, compute `config_.boundaries` from NPY shape + resolution + offset:
```cpp
config_.boundaries.min_x  = offset.x;
config_.boundaries.max_x  = offset.x + shape[0] * resolution;
config_.boundaries.min_y  = offset.y;
config_.boundaries.max_y  = offset.y + shape[1] * resolution;
config_.boundaries.min_height = offset.z;
config_.boundaries.max_height = offset.z + shape[2] * resolution;
```

`toIndex()` formula becomes:  
`rx = (pos.x - config_.boundaries.min_x) / res`  
(same as old formula since old `min_x_cm_ = offset.x` now, previously hardcoded to 0).

---

## Verification

Build sequence:
```bash
cd /workspaces/drone_mapper/ex_2_skeleton-new
cmake -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build --parallel
```

Run all tests:
```bash
./build/drone_mapper_simulation_test
```

Expected: all test fixture suites pass:
- `SimulationManager.*`, `SimulationRun.*`, `MissionControl.*`, `DroneControl.*`
- `MappingAlgorithm.*`, `MockLidar.*`, `MapsComparison.*`
- `Integration.*`

Run maps_comparison:
```bash
./build/maps_comparison data_maps/single_voxel_x2_y4_z2.npy data_maps/single_voxel_x2_y4_z2.npy
# Expected output: 100
```

Run simulation:
```bash
./build/drone_mapper_simulation scenarios/scenario1/simulation.yaml /tmp/test_output
# Expected: produces simulation_output.yaml and output_results/ folder
```

---

## Key Design Decisions

1. **MockMovement keeps collision detection** — 3-arg constructor preserved. The new skeleton's stub removes it but the assignment requires SimulationRun tests to test MockMovement obstacle detection. Our MockMovement is an implementation class we own.

2. **MappingAlgorithmImpl gets 3-arg constructor** — `(mission, drone, bounds={})`. Default-constructed drone/bounds allow `(mission)` calls in tests, but the factory always passes all three. Bounds come from `hidden_map->getMapConfig().boundaries`.

3. **Output map boundaries from hidden map** — Factory extracts `hidden_map->getMapConfig().boundaries` and uses those to create the output map (as the new skeleton stub shows). Mission YAML boundaries are parsed but not stored in `MissionConfigData`.

4. **Scoring moves from MissionControlImpl → SimulationRunImpl** — `MissionControlImpl::runMission()` no longer scores; `SimulationRunImpl::run()` calls `MapsComparison::compare()` after `runMission()` returns.

5. **yaml-cpp replaces hand-rolled parser** — Use `YAML::LoadFile()` throughout. The old `YamlConfigParser.cpp` is completely rewritten. The `ErrorLogger.cpp` and `ScoreReportWriter.cpp` are preserved (updated types only).
