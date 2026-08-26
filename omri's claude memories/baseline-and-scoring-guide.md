# Baseline Scores & Ex3 Status — Guide for the Team

This doc explains how our drone-mapper is scored, what our locked-in baseline numbers are, what
counts as a real regression vs. noise, and where the ex3 (assignment 3) work currently stands.
The other files in this folder are the raw, session-by-session working notes this doc was built
from — read those if you want the blow-by-blow; read this one if you just need the current state.

## 1. How a run is scored

Each simulated mission produces a `mission_score` (0–100%). It's computed by `MapsComparison`:
an F1-style match between the map our algorithm actually built (the output map) and the *full*
hidden map — **not** just the mission's configured boundaries. That last point matters: an
algorithm that maps its assigned region perfectly but ignores the rest of a larger hidden map
still gets penalized for the parts it didn't see, since scoring always compares against the whole
hidden map.

A `SimulationManagerReport` aggregates `mission_score` across every run in a composition (min,
max, and per-run breakdown in the output YAML).

## 2. Baseline scores — must not regress

These four numbers are our reference point, established and tuned during assignment 2, and
**re-verified bit-for-bit identical through the new assignment-3 dlopen/plugin pipeline** (not
just re-measured in isolation — the actual `.so`-loading `-comparative` path reproduces these
exactly):

| Scenario   | Baseline score | Notes |
|------------|---------------:|-------|
| scenario1  | 97.6%          | |
| scenario2  | 99.8%          | |
| scenario3  | 70.6%          | |
| benchmark  | 33.4%          | 29×30×31 voxel map at 1cm resolution |

Plus: **92/92** unit/component/integration tests green (78 original + 14 added to close grading
test-coverage gaps — see §4), and **84/84** tests passing in the new ex3 three-project layout
(Algorithm 14, MissionControl 23, Simulator 47 — different split, same coverage, see §5).

**Note: the ex2 test suite (all 92 tests) has been reviewed and approved by Omri** as the
confirmed regression baseline — treat it as the source of truth for "does this still work,"
not just a formality to keep green.

These aren't the *theoretical maximum* — geometric-reachability analysis puts the true ceiling
higher for several scenarios (e.g. large_out ≈ 93.3%, small_out ≈ 68%, computed via full-map BFS
from the drone's start voxel). The gap between our score and the ceiling on room-type scenarios
is a step-budget constraint (confirmed experimentally: 10–20× the step budget closes most of the
gap), not an algorithm defect. The baseline table above is what we actually ship and defend
against regressions — not the ceiling.

## 3. What counts as a regression

- **At or above baseline for the same scenario/config** → fine. Above baseline is a genuine
  improvement worth calling out, not assumed measurement noise (scores are deterministic given a
  fixed algorithm/config — same input always produces the same score).
- **Below baseline** → something broke. Root-cause it before merging. This has caught real bugs
  twice already: once during the ex2 grading-feedback fixes, and multiple times during the ex3
  port (see §5) — every step of the port was gated on reproducing these exact numbers before
  moving to the next step.

### Reproducing the baseline yourself

**Ex2** (from `ex_2_skeleton-new/`):
```bash
cmake --build build --parallel 4
./build/drone_mapper_simulation_test                          # expect 92/92
./build/drone_mapper_simulation scenarios/scenario1/simulation.yaml output_results/sc1/
./build/drone_mapper_simulation scenarios/benchmark/simulation.yaml output_results/benchmark/
```

**Ex3** (from `ex_3_skeleton-main/ex_3_skeleton-main/`), through the real `.so`/dlopen pipeline:
```bash
cmake -B build -DCMAKE_TOOLCHAIN_FILE=$VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake
cmake --build build
cd /path/to/ex_2_skeleton-new   # map_filename paths in scenario configs are CWD-relative
/path/to/ex3/build/Simulator/simulator_211781141_325049575 -comparative \
  simulation=scenarios/scenario1/simulation.yaml \
  mission_control_folder=<dir containing exactly one MissionControl .so> \
  algorithm=<path to Algorithm_211781141_325049575.so>
```
The comparative report's `total_score` should match the baseline table above exactly for a
single-plugin run.

## 4. Ex2 grading feedback — recap

Official score: **88.5 / 89**. Grading formula (from the staff's grading explanation): 28 bugs
are injected one at a time via compile flags; our own test suite is rerun against each; the score
starts at 100 and is docked for `interface_changed`, `obsolete_bugs` (required behavior missing),
crashes, timeouts, and insufficient bug coverage from component/integration tests (goal: ~50%
component, ~25% integration), with bonus points available for exceeding the coverage goal.

Two things were docked and both are now fixed (zero score/behavior regression from either fix):

- **`interface_changed` (−5 points)**: two student-defined types (`ComparisonMapConfig`,
  `GridCell3D`) had been added directly into a staff-owned, do-not-touch header
  (`types/MapTypes.h`). Fixed by moving them into student-owned headers instead. That header is
  now byte-identical to the pristine staff skeleton.
- **Test coverage gap**: component tests only caught 10/28 injected bugs (goal ~14); integration
  caught 3/28 (goal ~7). Not an implementation problem — `obsolete_bugs: 0` confirmed all required
  behavior was already correctly implemented, it just wasn't being exercised by any test. Added 14
  targeted tests (one per uncovered bug), bringing component coverage to ~24/28, comfortably above
  goal, with the existing 78 tests plus these 14 all still green.

## 5. Ex3 (assignment 3) — where things stand

**The ask**: split the ex2 simulator into three independently-buildable projects — `Algorithm`
and `MissionControl` (each compiled as a `.so`, dynamically loaded at runtime) and `Simulator`
(the executable that loads and drives them) — plus a staff-provided `common/` and our own
`UserCommon/` for shared code. Add two run modes: `-comparative` (many MissionControl `.so`s vs.
one Algorithm) and `-competition` (one MissionControl vs. many Algorithm `.so`s), with
multithreading, new aggregate YAML report formats, and full CLI validation.

**Status: functionally complete**, verified against a fresh, full re-read of the assignment 3 PDF
this session — every concrete, checkable requirement (exact CLI argument names, both report YAML
shapes and their specific sort orders, output-folder naming and placement) was cross-checked
line-by-line against the current implementation and matches exactly. No discrepancies found.

What's done and verified:
- **Structural port**: all 3 projects build both standalone and combined via the root build,
  reproduce the exact ex2 baseline scores through the real plugin pipeline (§3).
- **Dynamic loading**: `dlopen`/registration mechanism working end-to-end — `.so`s self-register
  via a macro-generated static object at load time, into a singleton registrar the Simulator
  drains after each `dlopen`. Two genuine bugs were found and fixed here, the kind that only shows
  up under real concurrent multi-plugin runs, not by inspection:
  - A captured factory (`std::function`) holding code from inside a `.so` must be destroyed
    *before* that `.so` is unloaded (`dlclose`) — got this backwards once, fixed via C++'s
    reverse-declaration-order destruction guarantee.
  - `MissionControl`'s error log was writing to one shared path; concurrent plugins sharing one
    output directory raced on it. Fixed by giving each plugin its own output subdirectory.
- **Both run modes**, full CLI validation (any argument order, missing/unsupported-argument
  detection, bad file/folder checks), the two new aggregate report YAML formats (with our own
  documented choice for what "agreeing" mission controls means, since the spec leaves that to us:
  grouped by matching total score + total steps across the whole composition), and real
  multithreading (verified with actual concurrent CPU usage, not just code inspection) that
  correctly avoids the spec's "never exactly 2 threads" edge case.
- **Test suite ported**: 84/84 tests passing across the three projects (14 Algorithm + 23
  MissionControl + 47 Simulator) — not a submission requirement, but our own regression safety
  net, since our test suite is what a change like this is easiest to silently break.
- **Submission basics**: `students.txt` and `README.md` filled in with build/run instructions and
  documented design decisions.

**Not done, on purpose**: physically packaging the submission zip (mechanical, last step before
the Sep 6 deadline), and two optional bonus items neither attempted nor required — lazy
plugin load/unload (load a `.so` only when needed instead of all up front) and algorithm-tuning
for the class competition.

## 6. Where to look for more detail

- `ex3-plan.md` — the original architecture/design plan for the ex3 port (registration mechanism,
  threading design, why `UserCommon` has no makefile, etc.)
- `memory-ex2-status.md` — ex2 algorithm implementation notes, scenario-by-scenario score
  breakdowns, geometric-maximum analysis, and dead ends that were tried and reverted.
- `memory-ex2-grading-feedback.md` — full detail behind §4 above.
- `memory-ex3-status.md` — full session-by-session log of the ex3 port and feature work, including
  every bug found and how it was fixed.
