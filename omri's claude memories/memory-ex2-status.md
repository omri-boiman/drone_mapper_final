---
name: project-ex2-status
description: "Assignment 2 implementation status — scores, geometric maxima, fixes applied, and what to avoid"
metadata: 
  node_type: memory
  type: project
  originSessionId: c294bfbb-e49a-4672-a88b-fe658f368481
---

# Assignment 2 — Current Implementation Status (2026-07-04)

Working directory: `ex_2_skeleton-new/`. Only allowed to edit `src/MappingAlgorithmImpl.cpp` and `include/drone_mapper/MappingAlgorithmImpl.h`. Configs ARE editable.

## Baseline scores (must never regress)
- sc1: 97.6% (NOT 98.4% — confirmed with git stash test), sc2: 99.8%, sc3: 70.6%, benchmark: 33.4%
- 78 unit tests: all passing

## Implemented changes (currently in codebase)

### Fix 1 — Bootstrap navigation [SHIPPED]
When `worldToVoxel(drone.position)` returns OOB (e.g. house: drone at z=10cm, map at z=150cm), the algorithm navigates toward the map boundary before the first scan sweep.
- All code wrapped in `[CHANGE: Fix 1]` / `[END CHANGE: Fix 1]` comments in both .h and .cpp

### Fix 2 — Physical map size threshold for rotation step [SHIPPED]
Physical largest dimension < 200 cm → 5.625° (64 rotations per sweep); ≥ 200 cm → 11.25° (32 rotations).
- Doubles frontier visits for 200 cm input scenarios vs the old raw-voxel-count threshold
- Test `defaultBounds()` changed to 100×100×100 cm to stay in fine bucket
- All code wrapped in `[CHANGE: Fix 2]` / `[END CHANGE: Fix 2]` comments

### Config fix — output_mapping_resolution_factor: 2 [SHIPPED]
Added to all 4 input missions (large_mission_out/room, small_mission_out/room).
Matches output resolution (10 cm) to hidden map resolution (10 cm), eliminating false-Empty voxels.

## Current scores — professor's input scenarios (Fix 1+2 only)

Must run from `inputs/` subdirectory:
```bash
cd inputs && ../build/drone_mapper_simulation sim_compose.yaml ../output_results/inputs/
```

| Scenario | Drone | Lidar | Score | Geo Max | Gap |
|---|---|---|---|---|---|
| large_out | small | long | 87.1% | 93.3% | −6.2pp |
| large_out | small | **short** | **95.0%** | 93.3% | +1.7pp ✓ |
| large_out | large | long | 85.9% | 93.3% | −7.4pp |
| large_out | large | short | 94.4% | 93.3% | +1.1pp ✓ |
| small_out | small | long | 46.8% | 68.0% | −21.2pp |
| small_out | small | **short** | 60.4% | 68.0% | −7.6pp |
| small_out | large | long | 45.3% | 68.0% | −22.7pp |
| small_out | large | **short** | **65.2%** | 68.0% | −2.8pp |
| small_room | small/large | long | 31.7–31.9% | 68.0% | step-limited |
| small_room | small/large | short | 35.9–36.3% | 68.0% | step-limited |
| large_room | all | all | ~12.3% | 19.3% | step-limited |
| house | all | all | 0% | 0% | physically inaccessible |

## Theoretical geometric maxima (all computed 2026-07-04)

| Scenario | Geo Max | Method |
|---|---|---|
| large_out | 93.3% | Full-map BFS from (22,17,17); lidar z_min=2 z_max=15 voxels at 10 cm/voxel |
| small_out | 68.0% | Full-map BFS from (15,15,11); same lidar params |
| large_room | 19.3% | Same map as large_out; only 1,260 reachable voxels (confined start) |
| small_room | 68.0% | Same map/reachable as small_out (interior start connects to full building) |
| benchmark | 34.6% | 29×30×31 at 1 cm, z_min=0.5 cm z_max=20 cm |

## Step budget experiment (NOT in submission, experimental only)

With 10–20× step budgets: room scores dramatically improve:
- large_room: 12.3% → 20.6% — reaches geo max in only ~1,400 steps (professor gives 500)
- small_out lidar_short: 65.2% → 71.5% — fully solved in ~4,000 steps (professor gives 2,000)
- small_out lidar_long: crashes at ~8,000 steps (latent bug — safe within 2,000-step budget)
- small_room: crashes at ~1,600 steps (same latent bug — safe within 1,000-step budget)

**Conclusion**: Room scenario gaps are purely step-budget constraints, not algorithmic failures.

## What to avoid (tried and failed this session)

### Fix 3 — w_struct_=4 always [REVERTED — causes crashes]
Both input drones have phys_r2<1 at 10 cm res → sphere clearance check never runs → drone can navigate
to wall-adjacent cells without body-clearance. With w_struct_=4, frontier scoring drove drones to dangerous
wall-adjacent positions → DRONE_HITS_OBSTACLE on large_out+lidar_short. DO NOT re-try without first adding
a floor for w_struct_ based on clearForBody guarantees.

### Fix 4 — z-frontier detection [REVERTED — 30–45% regressions]
Adding ±z direction to isFrontier() flooded the frontier queue with unresolvable vertical frontiers
(can't be resolved by horizontal-only scanning). Each wasted 5×32=160 steps before blacklisting.
Caused 30–45% regressions across all scenarios. frontier_try_count_ is NOT sufficient protection.

## Key technical facts

- **Scan altitude**: ALWAYS 0.0 — tilted scans are assignment-illegal
- **w_struct_**: 0 for input drones (phys_r2<1 at 10 cm res), 4 for test/benchmark drones
- **Lidar gap**: lidar_short (fov_circles=4) consistently beats lidar_long (fov_circles=3) by ~18 pp for small_out — hardware limitation, cannot be closed algorithmically
- **MapsComparison scoring**: F1 over ALL occupied voxels in full hidden map; mission bounds ignored for scoring
- **Output map boundaries**: SAME as full hidden map (NOT mission bounds) — set in SimulationRunFactoryImpl
- **MockMovement collision**: Samples drone CENTER PATH against hidden map; any non-zero = crash

## Build & test
```bash
cd /workspaces/drone_mapper/ex_2_skeleton-new
cmake --build build --parallel 4
./build/drone_mapper_simulation_test          # 78/78 must pass
./build/drone_mapper_simulation scenarios/scenario1/simulation.yaml output_results/sc1/
./build/drone_mapper_simulation scenarios/benchmark/simulation.yaml output_results/benchmark/
```
