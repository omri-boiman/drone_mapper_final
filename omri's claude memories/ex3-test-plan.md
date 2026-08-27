# Ex3 Test Plan — What's Not Covered Yet

Status as of 2026-08-27. This is a **plan only** — nothing here is implemented. It complements
`memory-ex3-status.md` (what's done) and `baseline-and-scoring-guide.md` (current scores/counts).

Current suite: 93 tests passing across 4 binaries (Algorithm 14, MissionControl 23, Simulator
stub-based 49, Simulator real-plugin benchmark 7). All of that coverage is either a straight port
of ex2's component tests, or exercises exactly **one** Algorithm `.so` + **one** MissionControl
`.so`. Everything below is genuinely new ex3 behavior (CLI, multi-plugin aggregation, threading,
plugin-failure handling) that has **zero automated coverage** today — it was only checked manually
during development (per `memory-ex3-status.md`'s session log).

## Priority order

1. `CliArgs` unit tests (isolated, pure function, highest bug-surface-to-effort ratio)
2. `AggregateReport` unit tests (isolated, pure function, the two YAML formats are graded output)
3. Multi-plugin comparative/competitive runs (currently N=1 only — the actual point of ex3)
4. Plugin-load failure handling (corrupted `.so`, missing registration)
5. Threading correctness beyond timing (data races, result correctness under concurrency)
6. End-to-end CLI black-box tests (spawn the real binary)
7. Everything else (YamlConfigParser, UserCommon, `-verbose` content, dlclose stress, build structure)

---

## 1. `CliArgs.cpp` — zero test coverage

`Simulator/src/CliArgs.h` exposes one pure function, `parseAndValidateArgs(vector<string>, program_name)
-> optional<ParsedArgs>` — trivially unit-testable today with no refactor, but no test file exists
for it (checked: no `test_*.cpp` references `CliArgs`).

Proposed `Simulator/tests/test_cli_args.cpp`:
- **Happy path, both modes**: all required args present, any order → correct `ParsedArgs` (mode,
  paths, `num_threads` nullopt when absent).
- **Argument order independence**: same args shuffled into several permutations → identical result.
- **Missing mode flag** (neither `-comparative` nor `-competition`) → nullopt, error mentions mode.
- **Both mode flags given** → nullopt, error message.
- **Missing one mandatory arg** (once per arg, both modes: `simulation=`, `mission_control_folder=`/
  `algorithm=` for comparative, `mission_control=`/`algorithms_folder=` for competition) → nullopt,
  error names that specific arg.
- **Multiple missing args at once** → nullopt, error batches *all* of them together (per spec: "print
  a usage with an error message detailing the missing command line arguments" — plural).
- **Unsupported/unknown argument** (e.g. `foo=bar`) → nullopt, error names it; combined with a
  missing arg in the same invocation → both errors reported together (per spec, batched not
  first-one-wins).
- **`simulation=` points at a non-existent file** → nullopt, proper error (not a crash from a bad
  `ifstream` or similar).
- **`simulation=` points at a file that exists but can't be opened** (e.g. no read permission, or a
  directory given instead of a file) → nullopt, proper error.
- **`mission_control_folder=`/`algorithms_folder=` non-existent** → nullopt.
- **Folder exists but is empty of `.so` files** → nullopt (spec: "zero files of the desired usage").
- **Folder exists with irrelevant files but no `.so`** (e.g. only `.txt`) → nullopt, same as empty.
- **`algorithm=`/`mission_control=` (single-file args) pointing at a folder instead of a file** →
  nullopt.
- **`num_threads=` variants**: absent (ok, nullopt), `1` (ok), `2`/large N (ok), non-numeric (e.g.
  `num_threads=abc`) → nullopt with a clear error, `0` (decide/confirm intended behavior — spec
  doesn't explicitly say, worth asserting whatever we implement), negative (`num_threads=-1`) →
  nullopt.
- **`-verbose` present/absent** → `ParsedArgs.verbose` set correctly; also confirm it's accepted in
  any position among the other args.
- **`=` sign edge cases**: a value that itself contains `=` (e.g. a path with `=` in it, unlikely but
  worth one case), and confirming args are matched by the literal `key=` prefix per spec ("no spaces
  around `=`").
- **Duplicate argument given twice** (e.g. `simulation=a.yaml simulation=b.yaml`) — decide and assert
  the chosen behavior (last-wins vs. reject as unsupported-duplicate); currently unspecified in code
  and untested.
- **Comparative-only args given in competition mode (or vice versa)** — e.g. `algorithm=` supplied
  while `-competition` is set → should this be "unsupported argument" or silently ignored? Decide and
  test.

## 2. `AggregateReport.cpp` — zero test coverage

`writeComparativeReport`/`writeCompetitiveReport` are pure functions taking plain data
(`vector<PluginTotals>`, `vector<string> errors`, an output path) and writing a YAML file — no
plugin loading or simulation running needed to test them in isolation.

Proposed `Simulator/tests/test_aggregate_report.cpp` (parse the written YAML back with `yaml-cpp`
and assert on structure, not just "file exists"):
- **`writeComparativeReport` grouping**: several `PluginTotals` with matching `(total_score,
  total_steps)` pairs → grouped into one `same_results` entry listing all their `so_name`s; totals
  written once per group.
- **Sort order — comparative**: groups sorted by *number of agreeing managers descending* (spec's
  own comment in the YAML example) — construct totals that produce groups of size 3, 2, 1 in
  scrambled input order, assert output order is 3, 2, 1. Add a tie-break case (two groups of equal
  size) and decide/assert what we do (undefined by spec).
- **Sort order — competitive**: `writeCompetitiveReport` sorted by score descending, then steps
  ascending — construct entries that require both keys to disambiguate (equal score, different
  steps) and assert exact order.
- **`errors` list correctness**: plugins that failed to load/run appear by filename in `errors:`,
  and are *not* also present in `results_summary`.
- **No successful runs, only errors** → `results_summary` empty/absent, `errors` has everything;
  file is still valid YAML (no crash on empty aggregate).
- **No errors at all** → `errors: []` (or however we render empty), not omitted/malformed.
- **Single-entry case** (N=1) — regression guard that this still matches today's manually-verified
  single-plugin output shape.
- **Top-level fields**: `composition_file`, `mission_control_folder` (comparative) /
  `mission_control` (competitive), `generated_at_utc` (valid ISO-8601 UTC, per the
  `2026-05-30T23:31:10Z` example format) all present and correctly populated from the args passed
  in.
- **Output path**: file actually lands at the given `output_path`, with the exact documented
  filenames (spec doesn't name the aggregate file explicitly beyond "Comparative/Competitive
  Simulation Result Output File" — confirm/document whatever filename our code picks and lock it
  with a test).

## 3. Multi-plugin comparative/competitive runs — only N=1 exercised anywhere

Every existing integration/benchmark test (stub-based and real-plugin) uses exactly one
MissionControl `.so` and one Algorithm `.so`. Nothing exercises the actual *point* of
`-comparative`/`-competition`: running several `.so`s and aggregating across them. The multi-`.so`
grouping behavior was checked once manually per `memory-ex3-status.md` (identical `.so` copies) —
never with genuinely different implementations, and never as an automated test.

Proposed (extends `Simulator/tests/test_integration_benchmark.cpp`'s real-dlopen approach, or a new
`test_integration_multiplugin.cpp` — needs 2-3 small additional stub-like `.so` targets built for
test purposes, e.g. `tests/fixtures/` mission-controls with deliberately different behavior):
- **N=3 mission-control `.so`s, `-comparative`**: two behaviorally identical, one different →
  `comparative_report.yaml` groups the two matching into one `same_results` entry and the third
  into its own; per-plugin legacy YAML exists for all three with correct distinct filenames.
- **N=3 algorithm `.so`s, `-competition`**: verify `results_summary` sorted correctly across three
  genuinely different score/step outcomes (not just two identical copies).
- **One `.so` in the folder fails to load, others succeed**: `errors` contains only the failing
  one, the rest still run and appear in `results_summary` — this is currently a documented behavior
  but untested with a *real* mixed-success folder end to end (only unit-adjacent via manual runs).
- **Output isolation under concurrency**: with `num_threads` ≥ 3 and 3+ plugins, confirm each
  plugin's output map + `error.log` lands in its own `output_dir/<so filename>/` subdirectory with
  no cross-contamination (content, not just absence-of-crash).
- **Folder scan determinism**: `soFilesIn`'s `std::sort` on paths — confirm run order (and thus
  which YAML/log gets which name when multiple plugins tie) is deterministic across repeated runs.

## 4. Plugin-load failure handling — only the "corrupted .so" case spot-checked manually

- **Corrupted/non-ELF `.so`** (e.g. a text file renamed `.so`) in the folder → `dlopen` fails
  cleanly, reported in `errors`, doesn't abort other plugins. (Per notes this was checked manually
  once; promote to an automated test.)
- **Valid `.so` missing the matching `REGISTER_*` call** (e.g. an Algorithm `.so` that never calls
  `REGISTER_MAPPING_ALGORITHM`) → `loadAlgorithmPlugin`/`loadMissionControlPlugin` returns nullopt
  with the "no REGISTER_* call found" error, not a crash or hang.
- **`.so` that throws during static initialization** (if constructible) — decide/document expected
  behavior; likely out of scope but worth a note if untestable.
- **Same `.so` loaded twice in one run** (e.g. two identically-named copies via symlink or literal
  duplicate path in the composition) — shouldn't double-register or corrupt the registrar's
  single-pending-slot invariant.

## 5. Threading — only wall-clock timing checked, never correctness under concurrency

`memory-ex3-status.md` confirms threading was verified via `user` vs `real` CPU time (proves
concurrency happens) and identical-`.so`-copies producing "byte-identical totals" (proves no
corruption *in that one symmetric case*). Nothing has exercised asymmetric/adversarial concurrent
scenarios or been run under a race detector.

- **`partitionWork` as a pure unit** (in `main.cpp`'s anonymous namespace today — see note below):
  table-driven tests over `(total_items, num_threads_requested)` → exact expected chunk boundaries,
  including: 0 items, 1 item, exactly 2 items (must collapse to single-threaded per spec's "total
  threads never 2"), `num_threads` requesting more workers than there are items, `num_threads`
  absent/`1`, and the documented `extra_threads == 1 -> 0` collapse rule at its exact boundary.
- **ThreadSanitizer (TSan) run** of the multi-plugin test scenarios in §3, at least once — the
  strongest single tool for catching a real data race that "worked by luck" in a couple of manual
  runs. (This project already has a working ASan build recipe from this session's Map3DImpl fix —
  a `-fsanitize=thread` build dir is the same pattern.)
- **Result correctness under real concurrency with asymmetric work**: plugins whose mission
  durations differ a lot (fast stub vs. slow real algorithm) mixed in one run with `num_threads` >
  1, confirming totals/order are still correct once chunks finish at different times.
- **Repeat-run determinism**: same multi-plugin composition run N times with threading enabled →
  identical aggregate scores/steps every time (catches nondeterministic corruption that only shows
  up occasionally).

**Refactor note**: `partitionWork`, `soFilesIn`, `makeOutputDir`, `runWorkItems`, `runComparative`,
`runCompetition` are all `static`/anonymous-namespace functions defined directly in
`Simulator/src/main.cpp`, which is excluded from every test target (see the comment in
`Simulator/CMakeLists.txt`: "nothing in Simulator's own test code calls the REGISTER_* macros...
so main.cpp and plugin/* are simply excluded"). **None of them are unit-testable as written.**
Before implementing §3/§5's unit-level cases, either (a) extract the pure, registration-independent
ones (`partitionWork` at minimum, arguably `soFilesIn`/`makeOutputDir` too) into their own
header+`.cpp` that a test target can include without pulling in the registration machinery, or (b)
accept that this logic is only reachable via black-box process tests (§6). (a) is cheap and
strictly better for `partitionWork` specifically, since it has no dependency on plugins at all.

## 6. End-to-end CLI black-box tests — zero coverage

Everything about `main.cpp`'s actual argv-to-exit-code/stdout behavior has only ever been checked
by hand. Proposed: a small test helper that runs the built `simulator_<ids>` binary as a
subprocess (`std::system`/`popen`, or a tiny fixture wrapper) and asserts on exit code + stdout/
stderr text + resulting files, e.g. `Simulator/tests/test_cli_end_to_end.cpp` (or a
non-gtest shell/Python smoke-test script, if process-spawning from gtest feels awkward):
- No args at all → usage printed, non-zero exit, no output directory created anywhere.
- Each of the "missing/unsupported/bad file/bad folder" CLI cases from §1, driven through the real
  binary instead of the unit-level `parseAndValidateArgs` call, to catch any drift between the two.
- A fully valid `-comparative` invocation → exit 0, stdout matches the `"Comparative run: %zu
  succeeded, %zu failed..."` format, output directory exists with the documented contents.
- Same for `-competition`.
- Output directory creation failure (e.g. point `mission_control_folder` at a read-only location)
  → "proper error to screen" per spec, non-zero exit, no partial output directory left behind in a
  broken state.

## 7. Smaller/lower-priority gaps

- **`YamlConfigParser`**: only incidentally exercised (as a helper inside
  `test_score_report_writer.cpp`), no dedicated test file for its own parsing correctness —
  malformed YAML, missing required keys, the `drone_configs`/`lidar_configs` field rename, and the
  `composition_file` field living on `SimulationManagerReport` (all called out as port traps in
  `memory-ex3-status.md`) have no regression test guarding them specifically.
- **`UserCommon`** (`GridCell3D`, `ErrorLogger`): no dedicated test file for either, in any project.
  `GridCell3D`'s hash/equality is only exercised indirectly via `MapsComparison`/algorithm tests;
  `ErrorLogger` has no test anywhere. Low risk (small, simple code) but currently 100% coverage gap.
- **`-verbose` output content**: memory notes describe manually inspecting a 327-line log
  (header/per-step/footer fields) once. No automated test asserts the verbose log has the right
  structure, or — more importantly — that turning `-verbose` on never changes `mission_score`
  (this was checked manually and matters a lot; worth locking down as a regression test).
  Belongs in `MissionControl/tests/test_mission_control.cpp`.
- **Repeated load/unload (`dlclose`) stress test**: load and unload the *same* real `.so` many
  times in a loop (e.g. 50-100x) within one test process, to catch a resource leak or
  use-after-free regression in the registration/PluginRegistrar singleton that a single load/run
  wouldn't surface. Directly protects the invariant documented in `PluginRegistrar.h`.
- **Build-structure verification** (not gtest — a CI/script-level check): confirm `Algorithm/`,
  `MissionControl/`, `Simulator/` each still build **standalone** (their own `cmake -B build` from
  inside that folder alone, sibling folders absent/untouched) — this regressed silently once
  already if not checked after every CMakeLists change, per the "each part may run independently"
  requirement. Also worth scripting: a submission-zip dry run (5 folders + 4 build files +
  `students.txt` + `README.md`, no binaries) as a pre-submission gate rather than a manual step.

---

## Summary table

| Area | File(s) | Current coverage | Gap severity |
|---|---|---|---|
| CLI parsing | `Simulator/src/CliArgs.cpp` | none | High — user-facing, spec-mandated behavior |
| Aggregate reports | `Simulator/src/AggregateReport.cpp` | none | High — graded output format |
| Multi-plugin runs | `main.cpp` (`runComparative`/`runCompetition`) | N=1 only | High — the actual point of ex3 |
| Plugin load failures | `PluginLoader.cpp` | one manual check | Medium |
| Threading correctness | `main.cpp` (`partitionWork`, `runWorkItems`) | timing only | Medium-High |
| CLI end-to-end | `main.cpp` | manual only | Medium |
| YamlConfigParser | `Simulator/src/YamlConfigParser.cpp` | incidental | Low-Medium |
| UserCommon | `UserCommon/` | none | Low |
| `-verbose` content | `MissionControl/src/MissionControlImpl.cpp` | manual only | Low-Medium |
| dlclose stress | `PluginRegistrar`/`PluginLoader` | none | Low |
| Build structure | all `CMakeLists.txt` | none automated | Low (but cheap to script) |
