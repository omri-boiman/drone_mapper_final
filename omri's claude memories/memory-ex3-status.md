---
name: project-ex3-status
description: "Assignment 3 port progress — architecture decisions, confirmed working patterns, and traps found so far"
metadata: 
  node_type: memory
  type: project
  originSessionId: f5b10029-860a-4157-b285-1e2360e2bb17
---

# Assignment 3 — status (2026-08-17)

Plan lives at `/home/vscode/.claude/plans/mighty-jumping-peach.md` (Phase 1: structural port of
ex2 into the ex3 3-project skeleton at baseline parity; Phase 2: comparative/competition modes,
threading, new report formats). Submitter IDs: `211781141_325049575` (Amit Halfon, Omri Boiman).

## Confirmed working: dlopen/registration mechanism (smoke-tested, then deleted)

Built a throwaway stub `.so` + stub executable to validate the riskiest part of the design before
porting real code on top of it. Confirmed:
- `ENABLE_EXPORTS ON` + `target_link_options(... -rdynamic)` on the Simulator executable correctly
  exports the `MappingAlgorithmRegistration`/`MissionControlRegistration` constructor symbols.
- `Algorithm.so`/`MissionControl.so` linking only `common::common` (header-only) leaves the
  registration constructor as an undefined symbol at `.so` link time — this is fine, GNU `ld`
  doesn't reject it for `SHARED` libraries.
- `dlopen(path, RTLD_NOW | RTLD_LOCAL)` correctly resolves that undefined symbol back into the
  executable's exported symbols at load time, static init runs, the registration constructor
  fires, the factory becomes retrievable, and virtual dispatch through the dynamically loaded
  instance works correctly.

## Real bug found and fixed during the smoke test — load-bearing for `PluginRegistrar`/`LoadedPlugin` design

**A `std::function` captured from inside a `.so` (i.e. any `Factory` retrieved from the
registrar) must be destroyed BEFORE that `.so` is `dlclose`'d — not after.** The factory's
type-erased destroy/invoke thunks are code living inside the `.so`'s mapped memory; destroying
the `std::function` after `dlclose()` segfaults (confirmed by reproducing it: `dlclose()` then
let the factory go out of scope at end of `main()` → `SIGSEGV` inside
`std::_Function_base::~_Function_base()`; moving the factory's destruction to before `dlclose()`
fixed it immediately).

**Implication for the real `LoadedPlugin` struct** (Simulator's plugin-loading core, not yet
built as of this note): field declaration order matters because C++ destroys members in reverse
declaration order. The captured `Factory` member must be declared AFTER the RAII `.so` handle
member, so it gets destroyed BEFORE the handle's destructor (`dlclose`) runs:
```cpp
struct LoadedPlugin {
    DynamicLibraryHandle handle;   // declared first -> destroyed LAST (dlclose)
    common::MappingAlgorithmFactory factory;  // declared second -> destroyed FIRST
};
```
Same rule applies to any constructed `IMappingAlgorithm`/`IMissionControl` *instance* built from
that factory — it must be destroyed before `dlclose` too (this was already the plan's stated rule;
the factory-ordering trap was the new, non-obvious part this smoke test surfaced).

## Milestone: end-to-end pipeline proven working (2026-08-17)

Phase 1 tasks #1-8 complete: UserCommon (GridCell3D, ErrorLogger) scaffolded; Algorithm project
ported (`MappingAlgorithmImpl_211781141_325049575` in `namespace algorithm_211781141_325049575`,
builds standalone as `Algorithm_211781141_325049575.so`); MissionControl project ported
(`MissionControlImpl_211781141_325049575` builds its OWN internal `DroneControlImpl` now — the
Simulator never sees `IDroneControl` at all, matching the Structuring doc's intent); real
`PluginRegistrar`/registration `.cpp`s + `PluginLoader` built in Simulator; all ex2 simulation
infra (Map3DImpl, MockGPS/Lidar/Movement, MapsComparison, YamlConfigParser, ScoreReportWriter,
SimulationManager, SimulationRunImpl, SimulationRunFactoryImpl) ported into `namespace simulator`;
minimal `-comparative` N=1 `main.cpp` written and it WORKS:

```
cd ex_2_skeleton-new   # map_filename paths in configs are CWD-relative, matches ex2 convention
Simulator/build/simulator_211781141_325049575 -comparative \
  simulation=<ex2 scenarioN/simulation.yaml> mission_control_folder=<dir with 1 .so> \
  algorithm=<Algorithm .so path>
```
Reproduced the exact ex2 baseline scores through the full dlopen→registration→factory→run
pipeline: **scenario1 97.6%, scenario2 99.8%, scenario3 70.6%** — bit-for-bit match, not just
"close." This confirms the architecture (registration singleton, factory injection, MissionControl
owning its own DroneControlImpl) is behaviorally identical to the ex2 monolith, not just
structurally similar.

## A second real gotcha found during this port (worth knowing before touching Simulator/ again)

`Simulator/common_simulator/include/Simulator/SimulationTypes.h` declares `namespace
simulator::types { ... }` (SimulationConfigData, SimulationResult, etc). Any Simulator-project
file written as `namespace simulator { using namespace common; ... }` that ALSO ends up in a
translation unit where `SimulationTypes.h` was already included (directly or transitively)
will find `types::X` resolving to `simulator::types::X` FIRST — via ordinary (nearer-scope)
unqualified lookup, not ambiguity — silently shadowing `common::types::X` even though the
using-directive for `common` is active. This doesn't show up when a file is compiled alone
(its own .cpp, own TU) but breaks the moment it's `#include`d alongside `ISimulationRunFactory.h`
et al. in the same TU (this hit `SimulationRunFactoryImpl.cpp`, which pulls in `Map3DImpl.h`,
`MockGPS.h`, `MockMovement.h`, `MockLidar.h` all at once).
**Fix applied**: in every Simulator-project header/source that references a `common::types`
member, qualify it explicitly as `common::types::X` — never rely on the bare `types::X` form
there. Only genuinely `simulator::types` members (SimulationConfigData, SimulationResult,
SimulationManagerReport, ResolutionRequestStatus, SimulationCompositionData) should stay
unqualified as `types::X`. This is now applied consistently across Map3DImpl, MockGPS,
MockMovement, MockLidar, YamlConfigParser, ScoreReportWriter, SimulationManager,
SimulationRunImpl, SimulationRunFactoryImpl.

## Phase 1 status: functionally complete (2026-08-17)

Verified the root `CMakeLists.txt` (add_subdirectory of common+Algorithm+MissionControl+Simulator)
builds all 3 deliverables together with no target-collision issues from the standalone-build
guards, producing `Algorithm_211781141_325049575.so`, `MissionControl_211781141_325049575.so`,
`simulator_211781141_325049575` — then reran the scenario1 end-to-end check using THESE
root-built binaries specifically (not the separately-built standalone ones) and got the same
97.6% baseline match. Both build paths (standalone per-project, and root-combined) work and
agree.

**User initially deferred task #9** (porting the ~90 ex2 gtest cases into
Algorithm/tests, MissionControl/tests, Simulator/tests) — asked to move to Phase 2 instead,
baseline parity being already proven by direct runs. Later reversed this ("lets do the tests
now and see it all passes") — see completed milestone below.

## Phase 2 progress (2026-08-17, same session)

Tasks #11 (full CLI validation), #12 (real N-mission-control `-comparative`), #13
(`-competition` mode), #15 (error handling + dlclose) all done and verified by direct runs:

- `Simulator/src/CliArgs.h/.cpp`: full any-order arg parsing, batches ALL errors together
  (missing + unsupported + bad file/folder) before printing usage once, per spec. Verified:
  no-mode-flag, both-mode-flags, missing-required-args, unsupported-args, bad-file-path all
  produce correct combined error output.
- `Simulator/src/AggregateReport.h/.cpp`: `writeComparativeReport`/`writeCompetitiveReport`.
  **Chosen "agreeing managers" definition** (spec leaves this to us, needs to go in
  README.md): grouped by identical `(total_score, total_steps)` tuples, where `total_score`
  = sum of `mission_score` and `total_steps` = sum of `mr.steps` across every run in that
  plugin's `SimulationManagerReport`. Verified with 2 identical mission-control `.so` copies
  correctly grouping into one `same_results: [...]` entry with matching totals.
  Competitive mode's `results_summary` sorted score-desc/steps-asc, verified with 2 identical
  algorithm `.so` copies.
- `ScoreReportWriter::write()` now takes an optional `filename` param so each plugin's legacy
  per-run YAML gets a distinct name (`simulation_output_<so_filename>.yaml`) in the shared
  output folder, per spec point 7 of both modes.
- Verified a corrupted/non-ELF `.so` in the folder is caught (dlopen fails cleanly), reported
  in the `errors: [...]` list, and does NOT crash or block the other plugins from running.
- Verified a malformed (but existing/openable) `simulation.yaml` no longer crashes — added a
  top-level `try/catch` in `main()` around mode dispatch, since `YamlConfigParser` can throw on
  bad *content* even when the CLI layer's file-exists check already passed.
- `dlclose` ordering: confirmed correct by construction — each plugin's `LoadedPlugin` (RAII
  handle + factory) is a loop-local variable, so it destructs (dlclose) at the end of each
  loop iteration, strictly after every instance built from it (constructed deep inside
  `manager.run()`) has already been destroyed. No manual dlclose-ordering bugs possible here
  because it's all scope-based, not manual.
- **Known simplification, not yet threading-safe**: current comparative/competitive loops
  load ONE plugin at a time (dlopen → run → dlclose) rather than all-up-front. This is fine
  single-threaded but must change before task #14 (threading) — the established invariant
  (all dlopen + registrar-drain happens serially before any thread spawns) requires loading
  every plugin first, THEN handing out already-captured factories to worker threads. Task #14
  needs to restructure `runComparative`/`runCompetition` to a two-phase load-all-then-process
  shape before adding `std::thread`.

## Task #14 (multithreading) done and verified (2026-08-17, same session)

Restructured `runComparative`/`runCompetition` to the two-phase shape the invariant required:
Phase 1 loads every varying-side `.so` serially (main thread) into a pre-sized
`std::vector<std::optional<Loaded*Plugin>>`; Phase 2 partitions indices into contiguous chunks
(`partitionWork()` — `extra_threads = min(requested, work_items-1)`, collapsed to 0 if it would
land on exactly 1 extra thread, satisfying "total threads never 2") and spawns one `std::thread`
per extra chunk plus the main thread's own chunk, joining before proceeding.

**A second real thread-safety bug found and fixed here**: `SimulationManager::run()`
unconditionally writes to `<output_path>/output_results/error.log`. With multiple mission-controls
running concurrently against the SAME shared `output_dir`, that's a genuine race — multiple
independent `ofstream` objects appending to the same path from different threads. Fixed by giving
each plugin its own `output_dir/<so filename>/` subdirectory as the `output_path` passed to
`SimulationManager::run()` (so its `error.log` and output maps land in a private subdirectory per
plugin), while the legacy per-plugin YAML still goes into the shared top-level folder with a
distinguishing filename — this matches the spec's own wording exactly ("Error log(s) files" is
explicitly plural; item 7 explicitly requires the per-plugin YAML in the "same output folder").

Also hit a real template-design bug before it ever compiled: an early version tried a runtime
`bool fixed_is_algorithm ? ... : ...` ternary inside a function template shared by both modes —
both ternary branches must type-check for a SINGLE instantiation, but comparative and competitive
need `SimulationRunFactoryImpl`'s two factory args in opposite order, so one branch was always
ill-typed. Fixed by having each call site pass its own correctly-typed `build_run_factory`
closure instead of runtime-branching inside the shared template.

**Verified empirically, not just by inspection**: 4 identical mission-control `.so` copies +
`num_threads=3` → `user` CPU time (3m15s) vastly exceeds `real` wall time (49s), confirming actual
concurrent execution, all 4 correctly grouped into one `same_results` entry with byte-identical
totals (no data corruption from the parallel run). Exactly-2-work-items + `num_threads=5` →
`user`≈`real` (single-threaded, collapse rule correctly engaged). Competition mode re-verified
through the same refactored code path. Then reran the full scenario1/2/3/benchmark baseline sweep
through the final threaded binary — all four still match ex2 exactly (97.6/99.8/70.6/33.4,32.1) —
zero regressions from adding threading.

## Task #16 (submission polish) done (2026-08-17, same session)

`students.txt` filled in (Amit Halfon 211781141, Omri Boiman 325049575). `README.md` rewritten
with build/run instructions and documents the two spec-left-to-us decisions (agreeing-managers
definition, threading collapse rule) plus the MissionControl-owns-its-own-DroneControl design
note and known limitations (no bonus lazy-loading, minimal `-verbose` stub, tests not included).
Verified no stray binary artifacts outside gitignored `build/` dirs. **Verified the exact
README-documented build command** (`cmake -B build -DCMAKE_TOOLCHAIN_FILE=... && cmake --build
build`, no extra flags) works from a clean checkout and produces all 3 required deliverables —
this matters because it's literally what a grader would copy-paste.

## Phase 2 status: essentially complete

All of tasks #11-16 done and verified. Only task #9 (test-suite porting, deferred by user) and
possibly the optional bonus (lazy-load plugins instead of load-all-upfront — explicitly NOT
attempted, documented as such in README) remain. Submission zip
(`ex3_211781141_325049575.zip`) has not been physically created yet — that's a final packaging
step (zip the 5 folders + root CMakeLists.txt + students.txt + README.md, excluding build/
directories) whenever the user wants to actually produce it for submission.

## `-verbose` output enriched (2026-08-17, same session)

User asked for "only what's not bonus" (excluding zip packaging and doc updates for now). Of the
remaining open items, only `-verbose` enrichment was genuinely non-bonus work with something left
to do (test-porting is neutral/optional, not required or bonus; zip/lazy-loading/algorithm-tuning
were explicitly excluded or are bonus). Enriched `MissionControlImpl::runMission()`'s verbose log
(`MissionControl/src/MissionControlImpl.cpp`): now writes a mission-start header (UTC timestamp,
`max_steps`, `gps_resolution_cm`, output map path), per-step lines with `moved_cm` (distance since
previous step) and `elapsed_ms` (wall-clock since mission start) added to the existing
position/heading/status fields, and a mission-end footer (final status, total steps, total
distance, total elapsed ms, error count). Verified: builds clean, real run through the Simulator
with `-verbose` produces a well-formed 327-line log with correct header/body/footer, and — most
importantly — score is bit-identical to the non-verbose baseline (97.58224336107808) both with
and without `-verbose`, confirming the added instrumentation doesn't affect mission behavior.

## Task #9 (gtest suite porting) done and verified (2026-08-17, same session)

Ported the full ex2 test suite into the 3-project layout, one `gtest`/`gmock` target per project
(gated by `find_package(GTest CONFIG)`, not `REQUIRED`, so a grader without gtest still builds the
3 required deliverables). **Result: 84/84 tests passing** — Algorithm 14/14, MissionControl 23/23
(17 DroneControl + 6 MissionControl), Simulator 47/47 (10 MockLidar + 9 MapsComparison +
11 SimulationManager + 11 SimulationRun + 1 ScoreReportWriter + 5 Integration).

Two structural restructurings were required because ex2's original test shapes no longer fit the
post-refactor architecture:
- `MissionControl/tests/test_mission_control.cpp`: ex2 mocked `IDroneControl` directly and
  injected it into `MissionControlImpl`. That's impossible now — `MissionControlImpl` builds its
  own internal `DroneControlImpl` from `MissionControlDependencies` and never accepts an external
  one. Restructured to mock at the `ILidar`/`IGPS`/`IDroneMovement`/`IMappingAlgorithm` level
  instead, driving the same Completed/MaxSteps/Error scenarios indirectly through the real
  internal `DroneControlImpl`.
- `Simulator/tests/test_integration_pipeline.cpp` + new `Simulator/tests/StubPlugins.h`: ex2's
  integration tests directly `make_unique`'d the real `MappingAlgorithmImpl`/`MissionControlImpl`.
  Impossible in ex3 by design — those are separate dynamically-loaded `.so` projects, and
  `SimulationRunFactoryImpl` requires two factories to be supplied. Built trivial
  `StubFinishedAlgorithm`/`StubMissionControl` + factory functions in `simulator::test` to exercise
  `SimulationManager`/`SimulationRunFactoryImpl`/`SimulationRunImpl` wiring in isolation. The real
  end-to-end behavior (actual `.so` plugins reproducing exact ex2 scores) is verified separately
  by running the built executable — see the Phase 1/2 milestones above, both bit-for-bit matches.
- Same recurring `types::X` vs `common::types::X` shadowing gotcha (documented above) hit several
  new test files during porting (`test_mock_lidar.cpp`, `test_maps_comparison.cpp`,
  `StubPlugins.h`) — fixed the same way, qualifying explicitly.

**Final regression check**: since only test files + CMakeLists.txt test-target sections were
touched (no production `src`/`include` files), reran the real `.so` pipeline against scenario1
after all test-porting was done — score still bit-identical (97.58224336107808), confirming zero
production-code impact from the test-porting work.

## Not yet done
Physically creating the submission zip file (explicitly deferred by user). Optional bonus (lazy
plugin loading) and algorithm competition tuning (bonus) not attempted, by design. README not yet
updated to mention the (unsubmitted, dev-only) test suites now existing per project.
