---
name: project-ex2-grading-feedback
description: Ex2 grading feedback received (score ~88.5/89) — root cause of interface-changed penalty and test coverage gaps
metadata: 
  node_type: memory
  type: project
  originSessionId: f5b10029-860a-4157-b285-1e2360e2bb17
---

# Ex2 grading feedback (received 2026-08-16, under `feedback ex2/`)

Final score 88.5 (also shown as 89 pre-rounding). Breakdown from `feedback ex2/ex2_bugs - Sheet1.csv` +
`feedback ex2/feedback.txt` + `feedback ex2/Exercise 2 - Grading Explanation.pdf`:
- `interface_changed: 1` → -5 points (single biggest lever)
- `obsolete_bugs: 0` — all required functionality implemented (except DRO07/SIM20, explicitly excepted)
- `crashes: 0`, `timeouts: 0` — weighted 0 anyway this round
- component bug coverage: 10/28 caught (goal ~14 = 50%)
- integration bug coverage: 3/28 caught (goal ~7 = 25%)
- Bonus: +3

## Root cause of interface_changed=1 (confirmed via diff against `skeleton/ex_2_skeleton-main`, the pristine staff skeleton)

`ex_2_skeleton-new/include/drone_mapper/types/MapTypes.h` — a **staff-owned, do-not-touch** interface
header — has two student-added types appended to it that don't belong there:
- `struct ComparisonMapConfig` (only used by `src/YamlConfigParser.cpp` / `YamlConfigParser.h`)
- `struct GridCell3D` + `std::hash<GridCell3D>` specialization (only used internally by
  `MappingAlgorithmImpl` and `MapsComparison.cpp`)

Both are pure implementation details of student-authored classes and should live in a student-owned
header (e.g. inside `MappingAlgorithmImpl.h`/`YamlConfigParser.h`, or a new local header) instead of
inside the shared `types/MapTypes.h`. This is almost certainly what tripped the "interface changed"
flag. All other staff interfaces (`I*.h`, `Units.h`, `DroneTypes.h`, `LidarTypes.h`, `SimulationTypes.h`)
diff clean against the pristine skeleton. `MissionTypes.h` has a diff but it's comment-reordering only,
same fields — not a real change.

**Fix applied 2026-08-17**: `types/MapTypes.h` restored to pristine skeleton content. `GridCell3D`
(+ hash) moved to new student-owned `include/drone_mapper/GridCell3D.h`, included by
`MappingAlgorithmImpl.h` and `MapsComparison.cpp`. `ComparisonMapConfig` moved into
`YamlConfigParser.h` (its only consumer), same pattern already used there for
`CompositionWithPaths`. Verified behavior-neutral: 78/78 tests pass, sc1/sc2/sc3/benchmark scores
unchanged (97.6/99.8/70.6/33.4), and output `.npy` maps are byte-identical to the pre-fix baseline.

## Test coverage gaps (14/28 bugs have zero test coverage, component or integration)

Confirmed by grepping `ex_2_skeleton-new/tests/components/*.cpp` — no test exercises these paths:
- LID01/LID02: lidar exact z_max/z_min boundary (off-by-2-step detection edge)
- DRO08–DRO12: negative elevation truncation, first-step index, `latest_scan` persistence semantics,
  `applyToMap` scan_origin bounds check, end-of-beam hit logic when no hit occurs
- SIM19: mission ending in `Error` shouldn't be added to `mission_results`
- MAN22/MAN23/MAN27: min score -1.0 semantics, no-aggregation when all missions failed
- COM24/COM25: null map in `MapsComparison` (should crash per grading rubric — currently untested,
  0 crashes recorded so this path is simply never exercised), skip-last-map-if->2 logic
- ALG29: algorithm behavior when lidar always reports "too close walls"

Component tests only reached 10/28 vs goal 14; integration only reached 3/28 vs goal 7 — the shortfall
is almost entirely missing test cases, not code defects (obsolete_bugs=0 says implementation is fine).

## Coverage gap fix applied 2026-08-17

Added 14 tests, one per previously-uncovered bug (DRO07/SIM20 stay untested — excepted by the
grader). All pass against current code (confirms obsolete_bugs=0 was accurate — implementation was
already correct, just untested):
- `test_mock_lidar.cpp`: LID01 (exact-z_max detection), LID02 (exact-z_min not marked too-close)
- `test_drone_control.cpp`: DRO08 (negative elevate not truncated), DRO09 (first-step index bump),
  DRO10 (latest_scan doesn't leak past one step), DRO11 (applyToMap skips out-of-bounds scan_origin),
  DRO12 (miss beam doesn't mark endpoint Occupied)
- `test_maps_comparison.cpp`: COM24 (null target → -1.0, no crash), COM25 (3rd+ target still compared)
- `test_score_report_writer.cpp` (**new file**, registered in CMakeLists.txt): MAN22 (min_score falls
  back to 0.0, not -1.0, when no runs scored)
- `test_simulation_run.cpp`: SIM19 (errored mission_result still recorded)
- `test_simulation_manager.cpp`: MAN23 (all-failed run still aggregated), MAN27 (score_range stays
  {0,100} even with errors)
- `test_mapping_algorithm.cpp`: ALG29 (terminates when the whole map reads as too-close walls)

Verified after every batch: rebuild, full 92/92 test suite green, sc1/sc2/sc3/benchmark scores
unchanged (97.6/99.8/70.6/33.4), output `.npy` maps byte-identical to the original baseline —
zero behavior change, pure coverage addition. Component coverage should now be ~24/28 (was 10/28),
comfortably above the 50% goal.

See [[project_ex2_status]] for the separate, ongoing algorithm-scoring work (unrelated branch of effort).
