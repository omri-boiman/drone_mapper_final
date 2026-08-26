# Assignment 3 — Original Port + Build Plan

_Design plan produced during the ex2→ex3 port. All phases described here are complete; kept as
project history documenting the architecture decisions (registration mechanism, threading design,
UserCommon rationale). See `memory-ex3-status.md` in this folder for the verified outcomes._

## Context

Assignment 3 requires splitting the completed, working assignment-2 drone-mapping simulator
(`ex_2_skeleton-new/`) into three independently-buildable C++ projects — **Algorithm** (a `.so`),
**MissionControl** (a `.so`), and **Simulator** (an executable that `dlopen`s both) — plus a
staff-provided `common/` folder (already complete, must not be touched) and a student-owned
`UserCommon/` folder (does not exist yet) for code shared across 2+ of the three projects.

The user's stated priorities, in order:
1. Understand the assignment PDFs and skeleton README (done — see summary below).
2. **First priority**: port ex2 into the ex3 skeleton so it *fits* the new structure and the
   grader can run our existing tests/scenarios against it to confirm we still meet the ex2
   baseline (sc1 97.6%, sc2 99.8%, sc3 70.6%, benchmark 33.4%, 92/92 tests). This is a structural
   refactor, not new functionality — get everything building and passing at parity first.
3. Only then implement what ex3 actually asks for: the `-comparative`/`-competition` CLI modes,
   multithreading, dynamic `.so` loading/unloading, and the two new YAML report formats.

Confirmed decisions (asked and answered):
- Submitter IDs: **211781141_325049575** (Amit Halfon, Omri Boiman — from `ex1/readme.txt`).
- Port ex2's gtest component/integration tests into the new per-project layout as an
  unsubmitted dev-only regression safety net (not part of the 5 required submission folders).
- The registration mechanism is now grounded in the actual class demo the user pasted into
  `helper.cpp` (confirmed pattern, see below) rather than guesswork.

## Architecture summary (grounded in direct reads of the skeleton + a Plan-agent design pass)

**`common/`** (`common/include/Common/`) is 100% staff-provided and already working
(`common/CMakeLists.txt` builds it as a header-only INTERFACE library `common::common`). It
mirrors ex2's `Types.h`/`Units.h`/interfaces almost exactly, with namespace `drone_mapper` →
`common` and two real API changes to accommodate the factory pattern:
- `IMappingAlgorithm`'s constructor now takes one `MappingAlgorithmDependencies` struct instead
  of 4 positional args (inline-defined in the header; still stores members the same way, so
  `MappingAlgorithmImpl`'s `using IMappingAlgorithm::IMappingAlgorithm;` keeps working unchanged).
- `IMissionControl` is now just `runMission()` — no base constructor at all. Concrete
  implementations are built exclusively via `MissionControlDependencies` (declared in
  `MissionControlFactory.h`): `mission_config`, `drone_config`, `ILidar& lidar`, `IGPS& gps`,
  `IDroneMovement& movement`, `IMutableMap3D& output_map`, `IMappingAlgorithm& mapping_algorithm`,
  `output_map_file`, `verbose`. Note there is **no separate `lidar_config` field** — get it via
  `lidar.config()` — and **no `hidden_map`** (confirmed by reading ex2's `MissionControlImpl.cpp`:
  `hidden_map_` was stored but never actually used, so nothing is lost).
- `IDroneControl` is *not* in top-level `common/` — it's staff-provided but scoped to
  `MissionControl/common_mission_control/include/MissionControl/IDroneControl.h`, namespace
  `mission_control` (fixed, not per-student). This matches `Structuring the project.pdf`: driving
  a single drone is MissionControl's own private responsibility, not shared with the Simulator.
- Similarly `ISimulation`/`ISimulationRun`/`ISimulationRunFactory`/`SimulationTypes.h` are
  staff-provided under `Simulator/common_simulator/include/Simulator/`, namespace `simulator`.
  `SimulationTypes.h` matches ex2's `types/SimulationTypes.h` closely but with two renames to
  watch for during the port: `SimulationCompositionData::drones/lidars` → `drone_configs/
  lidar_configs`, and `composition_file` moved from a separate `ScoreReportWriter` parameter into
  a field on `SimulationManagerReport` itself.

**`UserCommon/`** (new) holds code needed by 2+ projects: `GridCell3D` (header-only value type +
`std::hash`, used by both Algorithm's `MappingAlgorithmImpl` and Simulator's `MapsComparison`) and
`ErrorLogger` (usable by any project, per spec "MissionControl and Algorithm project *may* create
error logs"). Per the spec's literal, twice-repeated "no makefiles in these folders," `UserCommon`
gets **no `CMakeLists.txt`** — each consumer lists its headers via a plain
`target_include_directories` and compiles its `.cpp` files (if any) directly into its own
`add_library`/`add_executable` source list. This is deliberate, not a shortcut: it keeps every
`.so` a single self-contained relocatable file (no 4th shared object to resolve at `dlopen` time),
and matches how the staff's own skeleton keeps mock implementations inside `Simulator/src` rather
than a shared library.

**Registration / dynamic loading**, confirmed by the user's class-demo snippet (`helper.cpp`) and
refined by a design pass against the actual staff headers:
- `MappingAlgorithmRegistration`/`MissionControlRegistration` constructors are *declared* in
  `common/` but never *defined* there — by design, their `.cpp` definitions belong to the
  **Simulator** project only (explicit in the assignment PDF).
- `Algorithm.so`/`MissionControl.so` link only `common::common` (header-only), so the
  registration constructor is a genuinely undefined symbol at `.so` link time. This is legal for
  a `SHARED` library (GNU `ld` doesn't reject undefined symbols there by default). At `dlopen()`
  time it resolves against the *executable's* exported symbols — which requires building
  `simulator_<ids>` with `ENABLE_EXPORTS ON` (`-rdynamic`).
- Storage: a Meyer's-singleton `PluginRegistrar` living in `Simulator/src/plugin/`, holding a
  single pending `std::optional<Factory>` slot per registration type. The hard invariant that
  makes this lock-free and correct: **every `dlopen()` + registrar-drain happens serially on the
  main thread, before any worker thread is spawned** — the assignment text explicitly endorses
  this ("If you prefer to load all the required .so files ahead it might be a proper solution!").
  Worker threads only ever *call* an already-captured `std::function` factory (stateless lambda,
  safe to invoke concurrently); they never touch `dlopen`/`dlclose`/the registrar again.
- `dlopen(path, RTLD_NOW | RTLD_LOCAL)` — `RTLD_NOW` surfaces symbol-resolution failures
  immediately at load time (not mid-mission, deep in a worker thread) which serves "the Simulator
  shall not crash"; `RTLD_LOCAL` keeps each plugin's symbols isolated from every other plugin,
  appropriate since we'll be loading many unrelated `.so` files from other teams.
- Ownership: bundle each `.so`'s RAII handle with its captured factory in one struct
  (`LoadedPlugin`). Every algorithm/mission-control *instance* is constructed as a stack-local
  `unique_ptr` scoped to exactly one work item and destroyed before that work item's lambda
  returns. The `.so` handles themselves (and thus `dlclose`) only go out of scope after every
  thread has joined — trivially satisfying "don't `dlclose` while related objects are alive."
- `Algorithm/CMakeLists.txt` / `MissionControl/CMakeLists.txt` build `SHARED` libs with `PREFIX ""`
  so the output is exactly `Algorithm_<ids>.so` / `MissionControl_<ids>.so` (no `lib` prefix).

**Standalone-buildable subprojects**: each of `Algorithm/CMakeLists.txt`, `MissionControl/
CMakeLists.txt`, `Simulator/CMakeLists.txt` guards its own `project()`/toolchain setup with
`if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)` (true only when built standalone, not
via the root's `add_subdirectory`), and guards pulling in `common` with
`if(NOT TARGET common::common) add_subdirectory(../common ...) endif()`. This satisfies "each part
may run independently with another team's implementation of the other parts" without touching the
already-working root `CMakeLists.txt` or `common/CMakeLists.txt`.

## Phase 1 — Structural port to baseline parity

Goal: all 3 projects build (standalone and via root), the full dlopen→registration→factory→run
pipeline works end-to-end, and running the SAME scenarios/tests we had in ex2 produces the SAME
scores. Scope this phase to the **`-comparative` mode with exactly one `.so` in the mission_control
folder** — that's the natural minimal exercise of the real pipeline (not a shortcut/mock), and
comparative-with-N=1 must produce results identical to plain ex2. `-competition` mode, real
multithreading, and the two new report-aggregation formats are Phase 2.

1. **Scaffold `UserCommon/`**: create `UserCommon/include/UserCommon/GridCell3D.h` (ported
   verbatim from `ex_2_skeleton-new/include/drone_mapper/GridCell3D.h`, namespace
   `user_common_211781141_325049575`, stays header-only — no `.cpp` needed) and
   `UserCommon/include/UserCommon/ErrorLogger.h` + `UserCommon/src/ErrorLogger.cpp` (ported from
   ex2's `ErrorLogger.h/.cpp`, same namespace).

2. **Port Algorithm project**: `MappingAlgorithmImpl.h/.cpp` → `Algorithm/include/Algorithm/` +
   `Algorithm/src/`, renamed class `MappingAlgorithmImpl_211781141_325049575`, wrapped in
   `namespace algorithm_211781141_325049575 { using namespace common; ... }` (matching the
   `using namespace common;` idiom already used in the skeleton's own `IDroneControl.h`). Update
   `#include <drone_mapper/...>` → `#include <Common/...>`; `types::GridCell3D` →
   `user_common_211781141_325049575::GridCell3D` via `#include <UserCommon/GridCell3D.h>`. Add
   `REGISTER_MAPPING_ALGORITHM(MappingAlgorithmImpl_211781141_325049575)` at file scope inside the
   namespace, at the bottom of the `.cpp`. Write `Algorithm/CMakeLists.txt` per the design above.

3. **Port MissionControl project**:
   - `DroneControlImpl.h/.cpp` and `ScanResultToVoxels.h/.cpp` → `MissionControl/include/
     MissionControl/` + `MissionControl/src/`, wrapped in `namespace
     mission_control_211781141_325049575`, `DroneControlImpl` implements the staff's
     `mission_control::IDroneControl` (qualify explicitly or `using mission_control::IDroneControl;`).
   - `MissionControlImpl.h/.cpp` → renamed `MissionControlImpl_211781141_325049575`, constructor
     now takes one `common::MissionControlDependencies` by value: extract `mission_config`/
     `drone_config` by copy, keep references to `lidar`/`gps`/`movement`/`output_map`/
     `mapping_algorithm`, derive `lidar_config` via `dependencies.lidar.config()`, internally
     `std::make_unique<DroneControlImpl_...>(...)` (no `hidden_map_` — confirmed unused in ex2).
     Same `runMission()` step loop as ex2. Add a minimal verbose-mode output (write a per-step log
     file when `dependencies.verbose` is true) — functional now, richer content is a Phase 2/3
     polish item, not blocking. `REGISTER_MISSION_CONTROL(MissionControlImpl_211781141_325049575)`
     at the bottom.
   - Write `MissionControl/CMakeLists.txt` per the design above.

4. **Build the Simulator's plugin-loading core** (genuinely new code, not a port):
   `Simulator/src/plugin/PluginRegistrar.h`, `MappingAlgorithmRegistration.cpp`,
   `MissionControlRegistration.cpp` per the singleton design above. A small `loadAlgorithmPlugin`/
   `loadMissionControlPlugin` pair of functions (`dlopen` → drain registrar → RAII-bundle handle +
   factory, or a clear error). Confirm the `-rdynamic`/`ENABLE_EXPORTS` linkage actually works with
   a throwaway two-line stub `.so` before building anything on top of it (`readelf -d` / `nm -D`
   sanity check) — this is the one piece of the whole design with real platform-specific risk.

5. **Port Simulator's simulation-only infrastructure**: `Map3DImpl`, `MockGPS`, `MockLidar`,
   `MockMovement`, `MapsComparison` (now using `UserCommon::GridCell3D`), `YamlConfigParser`,
   `ScoreReportWriter` (existing per-run YAML format only — comparative_report/competitive_report
   formats are Phase 2), all → `Simulator/include/Simulator/` + `Simulator/src/`, namespace
   `simulator` (matches the already-provided `ISimulation`/`ISimulationRun`/etc.). Watch for the
   `drones/lidars` → `drone_configs/lidar_configs` field rename and the `composition_file` move
   into `SimulationManagerReport` while porting `YamlConfigParser`/`SimulationManager`.

6. **Adapt `SimulationManager`/`SimulationRunImpl`/`SimulationRunFactoryImpl`**: instead of
   directly `make_unique<MappingAlgorithmImpl>(...)`/`make_unique<MissionControlImpl>(...)`, they
   now receive an already-loaded `common::MappingAlgorithmFactory` + `common::MissionControlFactory`
   (from step 4) and call `factory(dependencies)` per run. This is the crux of proving the whole
   plugin architecture works — everything else in this class is unchanged from ex2's cartesian-
   product-of-missions/drones/lidars loop.

7. **Write a minimal `Simulator` `main.cpp`**: implement just enough of `-comparative` mode to
   exercise the full pipeline — parse `simulation=`/`mission_control_folder=`/`algorithm=`
   (accept but ignore `num_threads=`/`-verbose` for now, single-threaded), scan the folder
   (expect exactly one `.so` for this phase), load it via step 4, run the same composition loop as
   ex2 via steps 5-6, write the existing per-run YAML report. Skip full CLI validation/error-
   message rules and the `comparative_report.yaml` aggregation format for now (Phase 2) — but do
   create the `comparative_results_<time>` output folder per spec, even with only one entry inside,
   so the directory-naming logic is exercised early too.

8. **Port tests**: `test_mapping_algorithm.cpp` → `Algorithm/tests/`; `test_drone_control.cpp` +
   `test_mission_control.cpp` → `MissionControl/tests/`; `test_mock_lidar.cpp` +
   `test_maps_comparison.cpp` + `test_simulation_manager.cpp` + `test_simulation_run.cpp` +
   `test_score_report_writer.cpp` + both integration test files → `Simulator/tests/`. Each project
   gets its own optional gtest target, gated by `find_package(GTest CONFIG)` (not `REQUIRED`) so a
   grader without gtest can still build the 3 required deliverables. Copy the `.npy` fixtures each
   test file needs into that project's own `tests/data_maps/` (small files, avoids cross-project
   relative-path fragility).

9. **Verify baseline parity**: build all 3 projects both via root `CMakeLists.txt` and standalone;
   run every ported test suite (expect the same pass count as ex2's 92/92, split across 3
   binaries); run the same scenario YAMLs through the new `-comparative` single-`.so` path and
   confirm scores match ex2's recorded baseline (sc1 97.6%, sc2 99.8%, sc3 70.6%, benchmark 33.4%)
   and, where applicable, output `.npy` maps are byte-identical to the ex2 baseline outputs already
   on disk from the last session.

## Phase 2 — Implement what ex3 actually asks for

Coarser-grained (design decisions here depend on Phase 1 landing cleanly first):

1. **Full CLI parsing** for both modes: accept args in any order, validate all-mandatory /
   reject-unsupported-args / detect-missing-args / bad-file / bad-or-empty-folder cases, each with
   a usage message, per the spec's explicit list of required checks.
2. **Generalize `-comparative` to real N**: loop over every `.so` in `mission_control_folder`,
   run each against the one algorithm across the full composition, aggregate into
   `results_summary` grouped by "agreeing" mission controls (need to pick and document a concrete
   equality definition — e.g. matching `(total_score, total_steps)` tuples — since the spec leaves
   this to us), write the `comparative_report.yaml` plus one legacy-format per-mission-control YAML
   each, in a `comparative_results_<time>` folder. The folder must also contain every run's output
   map file (unique, mission-relatable name — this falls out for free once `SimulationRunImpl`
   writes into the new folder location) and error log file(s); if the folder itself cannot be
   created, print a proper error to screen (per spec, both comparative and competitive modes).
3. **Implement `-competition` mode**: mirror structure — one MissionControl `.so` vs many
   Algorithm `.so`'s from `algorithms_folder`, `results_summary` sorted by score desc/steps asc,
   `competitive_report.yaml` in a `competition_<time>` folder, same output-map/error-log/folder-
   creation-failure handling as comparative mode.
4. **Multithreading**: pre-built, pre-sized `std::vector<RunResult>` work-item list (built
   single-threaded after all plugins are loaded), `extra_threads = min(num_threads, work_items - 1)`
   with the `total==2` edge case collapsed to single-threaded (documented open question — spec
   says total threads are never 2; flag on the course forum if ambiguous), contiguous disjoint
   index ranges per thread, no locking needed since each thread only ever writes its own slice.
5. **Error handling**: catch `.so` load/symbol failures and per-run exceptions (except crashes,
   which the spec says we don't need to handle) into each report's `errors: [...]` list without
   aborting the whole run.
6. **`dlclose` on shutdown**: after all worker threads join and all `LoadedPlugin` instances are
   destroyed, unload every `.so` that was opened.
7. **Polish `-verbose` MissionControl output** beyond the Phase-1 minimal stub, and consider the
   documented bonus (load/unload plugins on demand rather than all up front — explicitly optional).
8. **`students.txt`, `README.md`, submission packaging**: fill in the two skeleton TODOs, document
   design decisions (esp. the "agreeing managers" definition and any threading edge cases) in the
   README, and do a final pass to confirm nothing beyond the 5 required folders + 4 build files +
   `students.txt` + `README.md` is needed in the submission zip (tests stay as our own dev
   scaffolding, not submitted, per the earlier decision). Package as
   `ex3_211781141_325049575.zip` per the spec's exact naming convention, and verify it contains
   no binary files (build artifacts, `.so`/`.o`/executables) before handing it off.

## Verification approach

- After Phase 1, the pass/fail bar is exactly the one already established for ex2: full test
  suite green (92/92, now split across 3 binaries), and the four baseline scenarios reproducing
  their recorded scores through the new `-comparative` single-plugin path.
- Before writing any real algorithm/mission-control code, do the throwaway-stub `dlopen`/
  `-rdynamic` smoke test called out in step 4 above — it's the one piece of this design with
  genuine platform risk, and it's cheap to falsify early rather than discover after porting
  hundreds of lines of ported logic on top of a broken assumption.
- Rebuild-and-retest after each major step (mirroring the discipline used for the ex2 grading-
  feedback fixes last session), not just at the end of each phase.
