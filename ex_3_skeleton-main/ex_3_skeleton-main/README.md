# Assignment 3 - Drone Mapper

A multithreaded simulator that dynamically loads drone-mapping `Algorithm` and `MissionControl`
implementations as shared libraries (`.so`) and runs them in two modes: **comparative** (many
mission controls vs. one algorithm) and **competitive** (one mission control vs. many
algorithms).

Namespaces: `common` (staff-provided, unmodified), `algorithm_211781141_325049575`,
`mission_control_211781141_325049575`, `simulator`, `user_common_211781141_325049575`.

## Building

Build all three projects together from the repository root:

```bash
cmake -B build -DCMAKE_TOOLCHAIN_FILE=$VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake
cmake --build build
```

This produces `build/Algorithm/Algorithm_211781141_325049575.so`,
`build/MissionControl/MissionControl_211781141_325049575.so`, and
`build/Simulator/simulator_211781141_325049575`.

Each of `Algorithm/`, `MissionControl/`, and `Simulator/` also builds standalone (its
`CMakeLists.txt` detects whether it's the CMake source root or being pulled in via
`add_subdirectory` from the top level, and only re-does `project()`/toolchain setup in the
former case) — e.g. `cd Algorithm && cmake -B build ... && cmake --build build` produces just
`Algorithm_211781141_325049575.so`, with no dependency on the other two projects' source.

## Running

```bash
# Comparative: every MissionControl .so in a folder, vs. one algorithm
./simulator_211781141_325049575 -comparative \
  simulation=<composition.yaml> mission_control_folder=<folder> algorithm=<algo.so> \
  [num_threads=<n>] [-verbose]

# Competitive: one MissionControl vs. every Algorithm .so in a folder
./simulator_211781141_325049575 -competition \
  simulation=<composition.yaml> mission_control=<mc.so> algorithms_folder=<folder> \
  [num_threads=<n>] [-verbose]
```

`map_filename` inside a `simulation_config.yaml` is resolved relative to the current working
directory (matching the assignment-2 convention), so run the simulator from the directory that
contains your `data_maps/` folder, or use absolute paths in your configs.

## Design decisions the spec left to us

**"Agreeing" mission controls (comparative mode)**: two mission controls are considered to
produce the same result if their **totals across the whole composition** match exactly —
`total_score` (sum of `mission_score` over every run) and `total_steps` (sum of steps over every
run). Mission controls with matching `(total_score, total_steps)` are grouped into one
`same_results` entry; groups are sorted by size descending, per the spec's example.

**Threading model**: `num_threads` (if `>=2`) requests that many *extra* worker threads beyond
the main thread. We compute `extra_threads = min(num_threads, work_items - 1)` so we never spin
up a thread with nothing to do, then collapse `extra_threads == 1` down to `0`: the spec
guarantees the CLI itself never asks for a total of exactly 2 threads, but a naive utilization
cap could still land there (e.g. 2 mission controls with `num_threads=8` requested would
naively cap to 1 extra thread, i.e. 2 total) — we round down to single-threaded instead of
violating that invariant. All `.so` files for a run are `dlopen`'d serially on the main thread
*before* any worker is spawned (the registration mechanism assumes one `dlopen` at a time), and
each worker thread only ever invokes an already-captured, stateless factory afterward — this is
what makes the parallel phase lock-free.

**Per-plugin output subdirectories**: each plugin's own output maps and `error.log` are written
into a private `<results_folder>/<so filename>/` subdirectory rather than directly into the
shared results folder. This isn't just organizational — it's what makes the threaded case safe:
`SimulationManager` unconditionally opens `error.log` inside its given output path, so without
a private subdirectory per plugin, concurrently-running threads would race on the same file. The
per-plugin *legacy* `simulation_output_<so filename>.yaml` still lives directly in the shared
top-level results folder, per the spec's explicit instruction ("Files will be added to the same
output folder").

**MissionControl owns its own drone control.** `IDroneControl` is not exposed to the Simulator
at all — `MissionControlImpl` builds its own internal `DroneControlImpl` from the raw
`ILidar`/`IGPS`/`IDroneMovement`/`IMutableMap3D`/`IMappingAlgorithm` references it receives via
`MissionControlDependencies`. This matches `Structuring the project.md`'s stated intent: driving
a single drone is the Mission Control's own responsibility, and both projects could — with
minimal changes — run on real hardware.

## Known limitations / not attempted

- The documented bonus (load/unload plugins on demand instead of all up front) is not
  implemented — we load every `.so` for a run up front, which the spec explicitly calls "a
  proper solution" in its own right.
- `-verbose` MissionControl output is currently a per-step log (`<map>_verbose.log`, one line
  per step with position/heading/status) rather than a richer trace.
- Our own gtest component/integration tests (ported from assignment 2, not part of the required
  submission) are not included in this build; they were used during development to validate the
  ported code against the assignment-2 baseline scores before layering assignment-3 features on
  top.
