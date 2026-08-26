# Drone Mapper — Complete Study Guide (Assignment 2)

**Course:** TAU — Advanced Topics in Programming, 2026B
**Student:** omri-boiman (boiman.omri@gmail.com)
**Scope of this document:** everything you need to explain the project in an oral exam — the *design*, the *algorithm* (in depth), the *problems you hit and how you solved them*, and the *C++ techniques* worth mentioning.

> **The one-sentence summary:** A simulated autonomous drone explores an unknown 3D environment using a LiDAR sensor, incrementally builds a voxel occupancy map, and the simulator scores that reconstructed map against a hidden ground-truth map. This is a **refactoring + testing** exercise: the whole skeleton had to be built to an exact prescribed API, plus GTest/GMock component & integration tests. **Only `MappingAlgorithmImpl.{h,cpp}` is student-authored logic** — the rest is the given skeleton, fully implemented.

---

## 1. The Big Picture — What the Program Does

1. Read a **simulation composition** YAML that describes a *cartesian product* of scenarios: `simulations × missions × drones × lidars`. Each combination is one **run**.
2. For each run, load a **hidden ground-truth map** (`.npy` binary voxel array) that the drone cannot see directly.
3. Place a virtual drone at a start position/heading. The drone has three actuators — **rotate, advance, elevate** — and one sensor — a **LiDAR**.
4. Loop: the **mapping algorithm** decides the next move + whether to scan. The simulator executes the move against the hidden map (checking for collisions), fires the LiDAR against the hidden map, and folds the scan result into the **output map** the drone is building.
5. When the algorithm reports *finished* (or the step budget runs out), **compare** the output map to the hidden map → a **score in [0, 100]**.
6. Aggregate all runs into `simulation_output.yaml` and write per-run output maps + error logs.

The key conceptual split:
- **Hidden map** = ground truth. Only the *simulation* side (MockLidar, MockMovement) may read it — it stands in for the real world.
- **Output map** = what the drone has discovered so far. This is what the algorithm reads/writes and what gets scored.

---

## 2. Architecture & Design

### 2.1 Interface-driven design (dependency inversion)

Every component is split into an **interface** (`IXxx`) and an **implementation** (`XxxImpl` / `MockXxx`). High-level code depends on interfaces, never concretes. This is the classic **Dependency Inversion Principle** and it's what makes the mocks swappable and the code testable.

| Interface | Real / Mock impl | Responsibility |
|---|---|---|
| `ISimulation` | `SimulationManager` | Top-level runner; expands the composition, aggregates the report. |
| `ISimulationRunFactory` | `SimulationRunFactoryImpl` | The single construction seam — wires up one fully-connected run. |
| `ISimulationRun` | `SimulationRunImpl` | Owns the object graph for one run; drives it; returns a `SimulationResult`. |
| `IMissionControl` | `MissionControlImpl` | Runs the step loop, saves the output map, returns mission status. |
| `IDroneControl` | `DroneControlImpl` | One `step()`: ask algorithm → move → scan → fold scan into map. |
| `IMappingAlgorithm` | **`MappingAlgorithmImpl`** ← *your code* | Decides next move + scan from current state. |
| `ILidar` | `MockLidar` (real would be `LidarDriver`) | Ray-casts beams against the hidden map. |
| `IGPS` | `MockGPS` | Reports position & heading. |
| `IDroneMovement` | `MockMovement` | Applies rotate/advance/elevate, checks collisions. |
| `IMap3D` / `IMutableMap3D` | `Map3DImpl` | Read-only / mutable voxel map with geometry config. |
| *(no interface)* | `ScanResultToVoxels` | Utility: folds a LiDAR scan into the output map. |
| *(no interface)* | `MapsComparison` | Standalone scorer + its own executable. |

**Why `IMap3D` vs `IMutableMap3D`?** Interface Segregation. Consumers that only *read* (the mapping algorithm, the LiDAR, the comparison) take a `const IMap3D&`. Only the writer (`ScanResultToVoxels`, output-map saving) needs `IMutableMap3D`, which *inherits* `IMap3D` and adds `set()` / `save()` / `isInBounds()`. So the algorithm literally *cannot* mutate the output map through its handle — the type system enforces read-only access.

### 2.2 Ownership model (who owns what)

- `SimulationRunImpl` **owns** the whole per-run object graph via `std::unique_ptr`: hidden map, output map, GPS, movement, lidar, mapping algorithm, drone control, mission control.
- Everything *below* it holds **references** (`&`) to its dependencies, not owning pointers. E.g. `DroneControlImpl` holds `ILidar&`, `IGPS&`, `IDroneMovement&`, `IMutableMap3D&`, `IMappingAlgorithm&`.
- **Rule of thumb you can state:** *"One owner (the run node), expressed with `unique_ptr`; everyone else borrows with references. Lifetime is guaranteed because the owner outlives all borrowers."* This avoids shared ownership complexity and dangling pointers.

### 2.3 The Factory pattern

`ISimulationRunFactory::create(sim, mission, drone, lidar, output_path)` is the **only** place the concrete object graph is assembled. `SimulationManager` depends only on the factory *interface*, so:
- In production it gets `SimulationRunFactoryImpl` → real wiring.
- In tests you can inject a **mock factory** returning a **mock run**, and test the manager's loop/aggregation in isolation.

The factory builds the hidden map first, reads its `MapConfig`, then builds the output map with matching geometry (offset/bounds) but possibly a different resolution.

### 2.4 The step / run flow (know this sequence cold)

```
SimulationManager.run(composition)
  └─ for each (sim × mission × drone × lidar):
       factory.create(...) → SimulationRunImpl
       run.run():
         MissionControlImpl.runMission():
           loop up to max_steps:
             DroneControlImpl.step():
               state = gps.position()+heading()
               cmd   = mappingAlgorithm.nextStep(state, lastScan)   ← YOUR CODE
               if cmd.status == Finished*     → stop
               if cmd.movement:  movement.rotate/advance/elevate()  ← may return DRONE_HITS_OBSTACLE
               if cmd.scan:      scan = lidar.scan(orientation)
                                 ScanResultToVoxels.applyToMap(outputMap, ...)
           outputMap.save(file)
         score = MapsComparison.compare(hiddenMap, outputMap)
       → SimulationResult
  └─ aggregate → SimulationManagerReport → simulation_output.yaml
```

Two subtle but examinable ordering facts:
- **Movement happens before scan** when a step provides both (spec requirement) — the drone moves, *then* senses from the new pose. See `DroneControlImpl::step()`.
- **`latest_scan` is fed back** to the algorithm on the *next* call (`latest_scan_` is stored then passed in and reset). The algorithm may receive `nullptr` — the interface explicitly documents this, and your impl ignores the raw scan anyway (it reads the *output map*, which `ScanResultToVoxels` has already updated).

---

## 3. Key Data Types

- `VoxelOccupancy` enum — the heart of the map. `PotentiallyOccupied(-3)`, `OutOfBounds(-2)`, `Unmapped(-1)`, `Empty(0)`, `Occupied(1)`.
  - `Unmapped` = never observed → **this is what frontiers are made of**.
  - `PotentiallyOccupied` = a hit landed closer than the LiDAR's `z_min` (too close to localize precisely) → uncertain.
  - `OutOfBounds` = outside the map array.
- `MapConfig` = `{ MappingBounds boundaries, Position3D offset, PhysicalLength resolution }` — the canonical geometry bundle. Introduced in the 9.6 refactor so boundaries/offset/resolution travel together on the map instead of being scattered.
- `MappingStepCommand` = `{ optional<MovementCommand> movement, optional<Orientation> scan_orientation, AlgorithmStatus status }` — what your algorithm returns each step. Both fields optional → you can move-only, scan-only, both, or neither.
- `MovementCommand` = `{ type (Rotate/Advance/Elevate/Hover), rotation (Left/Right), angle, distance }`.
- `DroneState` = `{ position, heading, step_index }`.
- `GridCell3D` = `{ int x,y,z }` with `operator==`, `operator<`, and a **custom `std::hash` specialization** (boost-style hash combine) so it can be a key in `unordered_map`/`unordered_set`.

---

## 4. THE ALGORITHM — Frontier-Based 3D Exploration (deep dive)

This is the part you'll be grilled on. `MappingAlgorithmImpl` implements **frontier-based exploration**: repeatedly go to the most informative boundary between known and unknown space, scan, repeat.

### 4.1 What is a "frontier"?

A **frontier cell** is a navigable Empty cell that has at least one **Unmapped** face-neighbour. Standing on it and scanning reveals new space. `isFrontier(c)`:
1. `c` must be `navigable` (Empty + no obstacle within drone's sphere).
2. `c` must be within mission bounds.
3. At least one **horizontal** (dz=0) face-neighbour is `Unmapped`.

> **Important design decision — horizontal-only frontiers.** Vertical (±z) Unmapped neighbours are *ignored*. Why? The LiDAR has limited vertical coverage (few `fov_circles`), so vertical Unmapped cells often can **never** be resolved from any reachable pose. Treating them as frontiers floods the queue with unresolvable targets → infinite exploration with no score gain. This was **Fix 4 (reverted)** — see §6.

### 4.2 The `nextStep()` state machine (priority order)

Each call resolves the *first* applicable case:

1. **Bootstrap** (`bootstrap_done_ == false`): if the drone's cell is `OutOfBounds` (it started outside the map array — e.g. the *house* scenario where the drone spawns 140 cm below the map), issue single Elevate/Advance commands via `navigateToMap()` to close the gap, **without scanning** (LiDAR may not even reach the map yet). Exit bootstrap once within one voxel of the boundary. For sc1/sc2/sc3/benchmark the drone starts inside → bootstrap is a no-op on the first call.
2. **Drain**: if `pending_commands_` (a `std::deque`) is non-empty, pop and return the next queued command. This is how a multi-command plan (a whole scan sweep, or a whole A* path) is emitted one command per step.
3. **Scan sweep** (`needs_scan_ == true`): on arrival at a new waypoint, enqueue a **full 360° yaw sweep** — `ceil(360 / rot_step)` rotate commands (32 at 11.25°, or 64 at 5.625°), each also carrying a scan request. This carves out the map around the current position.
4. **Select**: `selectBestFrontier(cur)` — value-scored BFS (below) picks the best target.
5. **Plan**: `enqueueNavigationTo(cur, frontier)` builds an A* path and translates it into rotate/advance/elevate commands, then sets `needs_scan_ = true` so a sweep fires on arrival.

### 4.3 Frontier selection — value-scored BFS (`selectBestFrontier`)

A **breadth-first search** expands over *navigable* voxels from the drone's cell, computing BFS hop-distance to each. Every cell that is a frontier is scored:

```
score = (gain + w_struct · structure) / (1 + β · dist)
```

- **gain** = count of `Unmapped` voxels in an `L³` cube around the candidate (`L = los_L_`, derived from LiDAR z_max). This is the *information value* — how much new space a scan here would likely reveal.
- **structure** = count of `Occupied`/`PotentiallyOccupied` voxels in that same cube. Interior-preference signal.
- **dist** = BFS hop count. Nearer frontiers are cheaper.
- **w_struct** (`w_struct_`) = **4.0** for drones whose radius ≥ 1 voxel, **0.0** for small drones. Rationale: big drones should prefer building interiors (lots of walls = high structure) over flying off into open sky; small drones fit anywhere and don't risk "escaping" a building, so the structure term only adds noise → disabled.
- **β** (`beta_`) = **0.15** distance penalty.

The BFS is bounded by `MAX_POPS = 50000` for safety. Navigability results are **cached** in an `unordered_map` (`nav_cache`) because the sphere check is expensive and cells get revisited.

**Two anti-loop guards** (critical for termination):
- **Anti-oscillation:** the last **8** chosen targets (`recent_targets_`, a `deque`) are excluded, so the drone doesn't ping-pong between two frontiers. Loaded into an `unordered_set` for O(1) lookup during the BFS.
- **Blacklist / exhaustion:** `frontier_try_count_` counts attempts per cell; any frontier attempted **≥ 5 times** is permanently excluded. This guarantees eventual termination even for frontiers that look attractive but can never actually be resolved (permanently shadowed) — this was **Fix for sc3's infinite loop**.

### 4.4 Navigability — two strictness levels (know the distinction!)

| Predicate | Used by | Empty required | Sphere clear of obstacles | Unmapped horizontal face |
|---|---|---|---|---|
| `navigable()` | Frontier BFS & selection | ✓ | ✓ | **allowed** |
| `clearForBody()` | A* path cells | ✓ | ✓ (`clearForBodyKnown`) | **blocked** |

- `navigable()` is **loose**: it *must* let the drone stand next to an Unmapped wall — that's the whole point of a frontier (you go there *to* observe the unknown).
- `clearForBody()` is **strict**: intermediate path cells must not touch any Unmapped horizontal face, because an Unmapped cell might actually be a wall and the drone body would clip it → `DRONE_HITS_OBSTACLE`. **Vertical Unmapped neighbours are deliberately allowed** (they're routinely Unmapped due to limited LiDAR elevation; blocking them deadlocks the drone indoors).
- The **sphere check** (shared `clearForBodyKnown`): iterate the cube `[-radius_voxels_, radius_voxels_]³`, and for any offset whose squared distance ≤ `phys_r2_` (the *float* squared radius), reject if that voxel is Occupied/PotentiallyOccupied/OutOfBounds. Using the **float** `phys_r2_` (not the ceil'd integer radius) matters: a 1.5-voxel-radius drone must not be inflated to 2 by rounding, which would wrongly block a 4-wide doorway.

### 4.5 3D A* navigation (`findPath3D`)

- **Graph:** 6-connected voxel grid (`kDirs` = ±x, ±y, ±z).
- **Heuristic:** Manhattan distance (`|dx|+|dy|+|dz|`) — admissible on a 6-connected unit-cost grid.
- **Open set:** `std::priority_queue` of `(f, cell)` with `std::greater` so it's a **min-heap** on f = g + h. Lazy deletion via a `closed` set (skip already-expanded).
- **Passability:** a neighbour is expanded only if `clearForBody(nb)` *and* `withinMissionBounds(nb)`.
- **Partial-path fallback:** it tracks the closest-to-goal cell reached (`best_partial`). If the goal is unreachable through clear space, it returns the path to that closest cell instead of giving up — the drone makes progress and scans from there. If it can't move at all, returns `{from}`.

### 4.6 Path → commands (`enqueueNavigationTo` / `enqueueRotateToAngle`)

For each path segment:
- **Vertical segment** (dz ≠ 0): emit Elevate commands chunked by `max_elevate`.
- **Horizontal segment:** compute the bearing with `atan2(dy, dx)`, rotate to face it (chunked by `max_rotate`, choosing Left/Right by the shortest signed angle wrapped to [-180,180]), then Advance chunked by `max_advance`.

`enqueueRotateToAngle` maintains a running `current_deg` so successive rotations compose correctly, and wraps into [0,360).

### 4.7 Termination

`FinishedWithUnmappableVoxels` is declared when:
- `selectBestFrontier` returns the sentinel `{INT_MIN,0,0}` even *after* clearing the anti-oscillation guard and retrying once (no reachable frontier remains), **or**
- `stuck_scan_count_ > 10` — the drone did 10 consecutive full sweeps without moving (the chosen frontier is blocked by an Unmapped horizontal face that scanning fails to resolve; remaining frontiers unreachable).

Otherwise the mission stops when `MissionControl` hits `max_steps`.

---

## 5. Supporting Components (know these at a "what & why" level)

### 5.1 MockLidar — ray casting against the hidden map

- Emits a **center beam** plus concentric **circles** of beams. Circle `k` has `4^k` beams (`beams_on_circle`), spaced by `d` at distance `z_min`. `fov_circles` controls how many rings → coverage.
- Each beam direction = scan orientation + drone heading; `traceBeam` marches in steps of `0.1 × resolution` from 0 to `z_max`, returning:
  - the hit distance if it strikes an `Occupied` voxel beyond `z_min`,
  - `0` if the hit is *closer* than `z_min` (unresolvable → becomes `PotentiallyOccupied`),
  - `numeric_limits<double>::max()` ("miss") if nothing is hit within range.
- **Trig with strong types** via `mp-units` `si::cos/sin/atan2`.

### 5.2 ScanResultToVoxels — fusing scans into the output map

Given the scan, for each hit it walks the beam and writes cells with a **priority/evidence policy** (`setIfStronger`):
- **Normal hit:** everything before the hit = `Empty` (proven free), the hit point = `Occupied`.
- **Miss:** whole ray to `z_max` = `Empty`.
- **Zero-distance (too close):** near segment to `z_min` = `PotentiallyOccupied`.
- **Conflict resolution** by `occupancyPriority`: `Occupied(3) > Empty(2) > PotentiallyOccupied(1) > Unmapped/OOB(0)`. A stronger observation overwrites a weaker one; a weaker one never downgrades a stronger. This prevents a later grazing "empty" ray from erasing a confirmed wall.

### 5.3 MockMovement — actuation + collision

- `rotate` clamps to `max_rotate`, updates heading (wrapped to [0,360)).
- `advance`/`elevate` clamp to the max, then **sample the drone's center path** in ~1 cm steps against the hidden map; if any sample is `Occupied` → return `{false, "DRONE_HITS_OBSTACLE"}` and **don't move**. (Note: samples the *center*, not the full sphere — hence the algorithm's own sphere-clearance responsibility.)

### 5.4 MapsComparison — scoring

- Rasterizes both maps over the origin's bounds at its resolution, collects the set of `Occupied` voxel keys for each, and computes an **F1 score ×100**:
  ```
  F1 = 200·TP / (2·TP + FP + FN)
  ```
  where TP = occupied in both, FP = occupied only in output, FN = occupied only in hidden.
- Edge cases: both empty → 100; exactly one empty → 0. Satisfies the spec's four required properties (identical→100, similar→~100, distinct→~0, in-between→reasonable).
- **F1, not raw accuracy** — because occupied voxels are sparse; plain accuracy would be dominated by the huge empty background and look artificially high. F1 balances precision & recall on the occupied class.
- Also builds a **standalone `maps_comparison` executable** that prints just the number to stdout (and `-1` + stderr message on error), per spec.
- **Only `Occupied` counts** for scoring — `Empty`, `Unmapped`, and `PotentiallyOccupied` are *not* scored as walls. So the score measures "did you find the real walls, without inventing fake ones." Leaving a region `Unmapped` costs you FN (missed walls); marking free space as `Occupied` costs you FP.

### 5.5 Map3DImpl — the voxel map (both hidden and output)

One class, **two modes**, chosen by which constructor is used:

- **Input/hidden mode** — `Map3DImpl(path, resolution, offset)`: loads a `.npy` binary array (via `NpyArray::LoadNPY`), validates it is **3-D, row-major**, and of a supported integer dtype (`int`, `uint8_t`, `int8_t`, `char`). It stores the array in a `shared_ptr<NpyArray>`. `atVoxel` reads the raw value and maps **non-zero → `Occupied`, zero → `Empty`** (the ground truth is fully known — no `Unmapped`). Boundaries are computed from `offset + shape × resolution`.
- **Output mode** — `Map3DImpl(bounds, resolution, offset)`: allocates a flat `std::vector<int8_t>` sized `ceil(range/res)` per axis, **initialized to `Unmapped(-1)`**. `set()` writes into it; `save()` serializes it back to `.npy`. This is the map the drone fills in.

Core geometry helpers you should be able to explain:
- **`toIndex(pos, xi, yi, zi)`** — world → array index: `floor((pos - min)/res)` per axis, returns `false` (→ `OutOfBounds`) if any index is negative or ≥ size. **Your algorithm's `worldToVoxel` mirrors this exactly**, which is why the two grids stay aligned.
- **`flatIndex(x,y,z) = x·(y_size·z_size) + y·z_size + z`** — 3-D → 1-D **row-major** layout.
- `isInBounds(pos)` — used by `ScanResultToVoxels` to avoid writing outside the array.

**Why one class with a flag (`is_output_map_`)?** The two maps share identical coordinate math (`toIndex`/`flatIndex`) and the same `IMap3D` read API — only the *backing store* and the *unknown* representation differ. Sharing the code guarantees the hidden map and output map interpret positions identically, so a scan traced against the hidden map lands in the exact same voxel when written to the output map.

### 5.6 MissionControlImpl — the step loop

`runMission()` is a simple, robust loop:
```
loop:
  r = drone_control.step()
  if r == Completed → status=Completed, stop      (algorithm finished)
  if r == Error     → status=Error, record msg, stop  (e.g. DRONE_HITS_OBSTACLE)
  ++steps
  if steps >= max_steps → status=MaxSteps, stop
output_map.save(file)      ← always saved, even on error/timeout
return {status, steps, errors}
```
Key points: **the output map is saved no matter how the mission ends** (so even a crashed run produces a scoreable partial map), and the three terminal statuses map directly to the spec's `completed` / `error` / `max_steps`.

### 5.7 SimulationManager — cartesian product + fault isolation

- Expands the **quadruple-nested loop** `sim × mission × drone × lidar` — this is the "20 runs" the spec describes.
- **Fault isolation via `try/catch` per run:** if `create()` or `run()` throws (bad map file, invalid bounds, …), the manager **logs the error, assigns score `-1`, and continues to the next run** — one broken scenario never aborts the batch. This is exactly the spec's error-handling requirement.
- Writes an ISO-8601 **UTC timestamp** (`utcNow()` via `std::put_time`/`gmtime`), the metric name, the score range, and the aggregated per-run results into the report → `simulation_output.yaml`.
- Owns a single `ErrorLogger` writing to `output_results/error.log`; **errors are logged immediately when they occur**, as the spec mandates (not deferred).

---

## 6. Problems Encountered & How You Solved Them

This is the section the examiner cares about most — *"tell me about a bug and how you fixed it."* Each fix in the code is wrapped in `[CHANGE: Fix N] … [END CHANGE]` comments.

### Fix 1 — Bootstrap navigation (drone starts outside the map)
**Problem:** In the *house* scenario the drone spawns at z≈10 cm but the map array sits at z≈150–460 cm (`height_offset=150`). `worldToVoxel(start)` is `OutOfBounds`, so the very first scan sees nothing and the algorithm can't get going.
**Fix:** Before the first sweep, if the drone's cell is OOB, issue single Elevate (priority) / rotate-then-Advance commands toward the map boundary until within one voxel of it, *without* scanning. Then hand off to the normal sweep. Targets `min_height − 0.5 cm` (just outside) because the map's bottom layers are solid ground the drone can't enter.

### Fix 2 — Adaptive rotation step (angular density vs. budget)
**Problem:** The scan sweep uses a fixed rotation step. Too coarse → misses thin features; too fine → burns the step budget on rotations, leaving fewer frontier visits. The *old* threshold keyed off raw voxel count (≤20), which accidentally put a 20-voxel × 10 cm = 200 cm map in the *fine* bucket, halving frontier visits.
**Fix:** Threshold on **physical** largest dimension: `< 200 cm` → **5.625°** (64 rotations/sweep, small high-res test maps where precision matters); `≥ 200 cm` → **11.25°** (32 rotations, large input scenarios) to *double* the number of frontier visits within the fixed budget. This meaningfully improved `small_out` coverage.

### Fix 3 — Mission bounds enforcement (latent crash)
**Problem:** At extended step budgets, `DRONE_HITS_OBSTACLE` crashes appeared — the drone wandered into building sections / altitude ranges outside the mission area, where the output map still had *false-Empty* wall voxels.
**Fix:** `withinMissionBounds()` gates **frontier BFS**, **A\* expansion**, and **`isFrontier`**. Cells whose center is outside `mission_config_.mission_bounds` are rejected. (Unset bounds `max_x ≤ min_x` → no restriction.) Root-caused and eliminated the latent crash within budget.

### sc3 infinite loop → the blacklist
**Problem:** Some frontiers are *permanently shadowed* — attractive by score, but can never actually be resolved. The drone kept re-selecting them forever.
**Fix:** `frontier_try_count_`: after **5** attempts a frontier is permanently excluded, guaranteeing termination. Plus `stuck_scan_count_ > 10` as a global safety net.

### Fix 4 — z-frontier detection (REVERTED — cautionary tale)
**Attempt:** Add ±z directions to `isFrontier` to also chase vertical unknowns.
**Result:** 30–45% score regression across all scenarios. Vertical Unmapped cells can't be resolved by horizontal-only scanning; the queue flooded with unresolvable targets, each wasting ~160 steps before blacklisting. **Reverted.** Lesson: `frontier_try_count_` alone doesn't protect you if you generate unbounded *new* bad frontiers.

### Fix 3b (w_struct=4 always) — REVERTED
**Attempt:** Always use structure weight 4.
**Result:** For small input drones (`phys_r2 < 1`), the sphere check never fires, so scoring drove them to wall-adjacent cells → collisions on `large_out + lidar_short`. **Reverted:** `w_struct_` is 0 for sub-voxel-radius drones, 4 otherwise.

### Config: `output_mapping_resolution_factor: 2`
**Problem:** Output map at 5 cm vs hidden map at 10 cm produced false-Empty voxels and mis-scored comparisons.
**Fix:** Set the factor to 2 in the input missions so output resolution = 10 cm = hidden resolution, aligning voxel grids for scoring.

**Known hardware limits you should be honest about:** `lidar_short` (4 rings) beats `lidar_long` (3 rings) by ~18 pp on `small_out` — a *sensor coverage* limit, not fixable in software. Room scenarios are *step-budget* limited (given 10–20× steps they reach the geometric maximum). The house scenario is physically inaccessible → 0% is the true maximum.

---

## 6.5 Limitations & Known Weaknesses (be honest — examiners respect this)

Don't oversell the algorithm. Knowing exactly where it's weak, and *why*, is more impressive than pretending it's perfect. Group them like this:

### Algorithmic / quality limitations
- **Greedy, not globally optimal.** Frontier selection is a local `argmax` of a hand-tuned score. There's no global tour planning, so the drone backtracks, re-crosses explored space, and wastes steps. A TSP-like or information-gain-over-a-route planner would map more per step. (The spec explicitly says a smart algorithm isn't required for ex2 — this is a *conscious* deferral, not an oversight.)
- **Vertical exploration is effectively disabled.** `isFrontier` only looks at horizontal neighbours; z-frontiers are ignored because limited LiDAR elevation (`fov_circles`) can't resolve them. Consequence: multi-level / tall vertical structure is under-mapped. This was a deliberate trade (adding z-frontiers = **Fix 4**, which caused 30–45% regressions), but it *is* a real coverage ceiling.
- **Room scenarios are step-budget-limited.** They only reach the geometric maximum with ~10–20× the given step budget (large_room 12.3%→20.6%, small_out short 65%→71% at 10× steps). Within budget the drone under-explores — a direct symptom of the greedy planner burning steps on sweeps and backtracking.
- **House scenario scores 0%.** The map's bottom layers are solid ground and the interior is physically inaccessible to the drone, so 0% is actually the *true* maximum — but it's still a scenario we map nothing on.
- **`w_struct_ = 0` for small drones → pure nearest-frontier.** Robust (no crashes) but tends to wander locally rather than head for high-information regions.

### Latent bug we never fully root-caused (say this plainly)
- **`DRONE_HITS_OBSTACLE` at *extended* step budgets.** With 10–20× steps, `small_out + lidar_long` crashes around ~8,000 steps and `small_room` around ~1,600. The **mission-bounds fix (Fix 3) mitigates it within the real budget**, so submitted scenarios are safe — but the underlying cause (the drone eventually commanding a move into a false-`Empty` / unresolved cell) is **not fully eliminated**, only bounded. This is the honest "there's a lurking bug we contained rather than cured."

### Performance / efficiency weaknesses
- **Everything is recomputed from scratch every waypoint.** `selectBestFrontier` runs a fresh BFS over *all* navigable cells (up to `MAX_POPS = 50000`) each time it's called, and `countCube` is **O(L³)** per frontier candidate. Nothing is cached or updated incrementally between steps — an incremental frontier set / cached occupancy would be far faster on large maps.
- **`MAX_POPS = 50000` can truncate the BFS** on a very large map, potentially causing the algorithm to miss reachable frontiers and terminate early (report `FinishedWithUnmappableVoxels` prematurely).
- **`stateOf` round-trips through floating point** on *every* neighbour touch: cell → `centerOf` (world center) → `atVoxel` → `toIndex` (world→index). Lots of redundant `double` conversions; slower than an integer grid lookup and, in principle, exposed to rounding at cell boundaries (mitigated by the `+0.5` center convention).
- **A\* is recomputed for every navigation** with no path caching/reuse.

### Fragility / coupling
- **The algorithm ignores `latest_scan` entirely** and instead reads the *output map*, trusting that `DroneControl` applied the previous scan *before* the next `nextStep`. That coupling to the harness's call order is implicit — a differently-ordered harness would leave the algorithm blind.
- **Many hand-tuned magic constants:** `200 cm` step threshold, `w_struct_ = 4`, `β = 0.15`, `los_L_` formula, 5-try blacklist, recent-8 ring, `stuck > 10`, `MAX_POPS`. These were tuned to the given scenarios and aren't derived from first principles, so they may not generalize to new maps/drones.
- **Fragile floating-point equality in the skeleton:** `ScanResultToVoxels` detects "miss"/"too close" with exact `==` against `0.0*cm` and `numeric_limits<double>::max()`. It works because those are exact sentinel values produced by `MockLidar`, but exact float comparison is generally brittle.
- **Simulator collision model is *looser* than the algorithm's.** `MockMovement` samples only the drone **center** path (not the full sphere) against the hidden map, whereas the algorithm enforces full **sphere clearance**. So the algorithm is *conservative relative to the simulator* — safe, but it may refuse maneuvers the simulator would actually have allowed (leaving score on the table).

### Testing limitations
- The `MappingAlgorithm` component tests are mostly **structural/contract** checks (returns Working, requests a scan every step, sweep is 32/64 Rotate-Right commands covering ≥360°, respects `max_rotate`, finishes on a tiny map, `done_` is sticky). They verify *shape and safety*, **not mapping quality** — they wouldn't catch a bug that merely makes exploration worse, only one that breaks the contract or crashes.

---

## 7. C++ Techniques — In Depth (this is where you score points)

The project is built as **C++20** (`set(CMAKE_CXX_STANDARD 20)`, `target_compile_features(... cxx_std_20)`), compiled with **`-Wall -Wextra -Werror -pedantic`** — i.e. *every warning is a hard error*, so the code is warning-clean by construction. (The `readme.txt` mentions a "C++23-capable compiler" as a prerequisite, but the standard the build actually requests is C++20 — know the difference if asked.) Dependencies come via **vcpkg** and are pulled in with CMake `find_package(... CONFIG REQUIRED)`: `mp-units`, `TinyNPY` (npy I/O), `yaml-cpp`, and `GTest`/`GMock`.

### 7.1 `mp-units` — compile-time dimensional analysis (the headline feature)
- `PhysicalLength`, `XLength/YLength/ZLength`, `HorizontalAngle`, `AltitudeAngle` are **distinct** `mp::quantity` types created with the `QUANTITY_SPEC` macro. The X/Y/Z split is deliberate: even though all three are lengths, they're *different quantity kinds*, so you **cannot** accidentally assign an x-coordinate into a y-slot — it's a **compile error**, not a runtime bug.
- Literals carry units: `50.0 * cm`, `11.25 * horizontal_angle[deg]`, `40.0 * z_extent[cm]`. A bare `double` will **not** implicitly convert into a quantity.
- To get a raw number back you must call **`.force_numerical_value_in(cm)`**. The word **"force"** is the point: it's a deliberate, visible escape hatch that says "I am dropping unit safety here" — you can't do it by accident. Every such call in the code is a conscious boundary crossing (usually into a `double` for arithmetic/indexing).
- Trigonometry is unit-aware: `si::sin/cos/atan2` take/return quantities and angles. `MockLidar` and `ScanResultToVoxels` use them so beam geometry stays dimensionally correct.
- **What to say in the exam:** *"mp-units turns a whole class of silent geometry bugs — mixing metres and centimetres, or x and y — into compile errors. It costs some verbosity (`force_numerical_value_in`) but buys correctness for free at compile time."*

### 7.2 Ownership, RAII, and value semantics
- **`unique_ptr` ownership tree:** `SimulationRunImpl` holds `unique_ptr` to every component; borrowers hold `&`. No `new`/`delete`, no leaks, deterministic teardown (RAII). Constructors **validate then throw** (`std::invalid_argument` if any injected pointer is null) — fail fast at construction.
- **Sink parameters + `std::move`:** `DroneControlImpl`, `MissionControlImpl`, `SimulationRunImpl` take configs/paths **by value** and `std::move` them into members. This is the modern "take by value and move" idiom — one copy for lvalues, zero for rvalues, and no `const&` + copy.
- **`std::shared_ptr<NpyArray>`** backs the hidden map's loaded data (`Map3DImpl`), so a copied map view can share the (immutable) array cheaply.

### 7.3 Interfaces, polymorphism, and construction patterns
- **Dependency Injection** through constructor + interface everywhere → no globals/singletons, everything mockable.
- **Factory pattern** (`ISimulationRunFactory`) = single construction seam → the manager can be tested with a mock factory/run.
- **Interface Segregation:** `IMutableMap3D : IMap3D`. Read-only consumers take `const IMap3D&`; only writers see `set/save`. The algorithm literally can't mutate the map it reads.
- **`final` class + `override`:** `class MappingAlgorithmImpl final : public IMappingAlgorithm` — `final` allows the compiler to devirtualize and forbids further subclassing; `override` on `nextStep` catches signature drift.
- **Inheriting constructors:** `using IMappingAlgorithm::IMappingAlgorithm;` — the impl reuses the base's constructor (which stores `mission_config_`, `lidar_config_`, `drone_config_`, `output_map_`) instead of re-declaring it. Neat, and easy to overlook — the impl has *no* explicitly written constructor.

### 7.4 STL data structures — chosen deliberately (be ready to justify each)
- **`std::priority_queue<Entry, vector<Entry>, std::greater<Entry>>`** for A*, where `Entry = std::pair<int, Cell>`. A pair compares **lexicographically**, so ordering by `first` = ordering by `f = g + h`, and `std::greater` turns the default max-heap into the **min-heap** A* needs. Lazy deletion via a `closed` set instead of decrease-key.
- **`std::queue<Cell>`** for the frontier BFS (FIFO → correct hop-distance layering).
- **`std::deque`** twice: `pending_commands_` (push_back plan, pop_front to emit one per step) and `recent_targets_` as a **fixed-size ring** (`push_back`; `pop_front` when size > 8).
- **`std::unordered_map/std::unordered_set`** for `dist`, `parent`, `g`, `closed`, the nav cache, `frontier_try_count_` — all O(1) average, keyed on `GridCell3D`.
- **Custom `std::hash<GridCell3D>` specialization** in `namespace std` — boost-style hash-combine using the `0x9e3779b9` golden-ratio constant with shifts. Without it, none of the `unordered_*` containers would compile for `GridCell3D` keys. `GridCell3D` also provides `operator==` (defaulted, C++20) and `operator<`.

### 7.5 Modern-C++ idioms sprinkled through the algorithm
- **Structured bindings:** `auto [gain, structure] = countCube(c);` and the caching trick `auto [it, ins] = nav_cache.emplace(c, false);` — `emplace` returns `{iterator, inserted?}`, so one call both checks and inserts, and the lambda memoizes navigability in a single lookup.
- **Lambdas with captures** as private local helpers: `isNav`, `countCube`, `tryCandidate`, `heuristic`, `reconstruct`. They keep state-heavy logic local without bloating the class's header API.
- **`std::optional` + `std::nullopt`:** `MappingStepCommand.movement` / `.scan_orientation` and `DroneControl::latest_scan_` express "maybe" without sentinels. (Note the asymmetry: the interface *also* passes `latest_scan` as a raw `const*` that may be `nullptr` — documented in the header.)
- **`explicit operator bool()`** on `MovementResult` → `if (!result)` reads naturally, `explicit` blocks accidental int/bool conversions.
- **`[[nodiscard]]`** on `nextStep` and the LiDAR/scan helpers so a caller can't silently drop a result.
- **`constexpr`** unit-conversion helpers (`toCm`, `toDeg`, `toRad`) in **anonymous namespaces** (internal linkage → no ODR clashes across TUs).
- **`enum class`** everywhere (`VoxelOccupancy`, `MovementCommandType`, `AlgorithmStatus`, …) — scoped, strongly typed, with explicit underlying values on `VoxelOccupancy`.
- **`<numbers>`** `std::numbers::pi` (C++20), plus `std::clamp`, `std::min/max`, `std::reverse`, `std::floor/ceil`, `std::atan2`, `std::fmod` from `<algorithm>`/`<cmath>`.
- **`noexcept`** on hot, non-throwing helpers (`flatIndex`, `toIndex`, the hash).

### 7.6 Build & tooling to name-drop
- **CMake targets** with `PUBLIC`/`PRIVATE` link scopes; `$<BUILD_INTERFACE:...>` **generator expression** for include dirs; three executables (`drone_mapper_simulation`, `maps_comparison`, the test binary) sharing one static lib.
- **`gtest_discover_tests`** auto-registers tests with CTest.
- **`-Werror -pedantic`** — a real signal of code quality; the whole thing builds clean under the strictest common flags.

---

## 7.7 Coordinate Systems & The World↔Voxel Transform (be fluent in this)

There are **three coordinate frames** in play — mixing them up is the classic source of geometry bugs, and the whole design is built to keep them consistent.

1. **World (physical) coordinates** — centimetres, strongly typed as `XLength/YLength/ZLength`. The drone's GPS reports these. Origin is arbitrary; a map's `offset` places the map array in world space.
2. **Voxel/grid indices** — integer `GridCell3D {x,y,z}`, one per cell of the array.
3. **Array flat index** — a single `size_t` into the backing store (row-major).

**World → voxel** (identical in `Map3DImpl::toIndex` and `MappingAlgorithmImpl::worldToVoxel`):
```
xi = floor( (pos.x − boundaries.min_x) / resolution )   // and same for y, z
```
**Voxel → world center** (`centerOf`): take the *center* of the cell, not its corner:
```
pos.x = min_x + (xi + 0.5) · resolution
```
The `+0.5` matters: you query occupancy at the cell **center**, so rounding is symmetric and a scan that hit anywhere in the cell maps back to the same cell.

**Why the algorithm re-implements `worldToVoxel` instead of asking the map:** the `IMap3D` read API only exposes `atVoxel(Position3D)` (world → occupancy), not "give me the index." So to reason about *grid neighbours* (BFS/A* over `±x/±y/±z`), the algorithm converts world→cell itself, then converts each candidate cell back to a world center via `centerOf` and calls `stateOf(cell) = output_map_.atVoxel(centerOf(cell))`. The transform is copied **exactly** from `Map3DImpl` so the two never disagree — if they drifted by even half a voxel, the drone's model of "where the walls are" would be offset from reality and it would crash.

**Resolution mismatch pitfall (the reason for `output_mapping_resolution_factor`):** the hidden map is at, say, 10 cm/voxel; the output map defaults to the mission's GPS resolution (e.g. 5 cm). A 10 cm wall then straddles two 5 cm output voxels, and empty scans through the "second half" can mark real wall cells `Empty` → wrong score. Setting the factor to 2 makes the output map 10 cm too, so the grids line up 1:1.

---

## 8. Error Handling & Robustness (spec-graded)

Error handling is explicitly graded, so have the story ready:

- **Immediate logging:** every error is written to `output_results/error.log` the moment it occurs (via `ErrorLogger`), never batched — a spec requirement.
- **Per-run fault isolation:** `SimulationManager` wraps each run in `try/catch`. A thrown exception → that run scores **`-1`**, the error is logged, and the batch continues. A whole *group* can be filled with `-1` if its shared input (e.g. a map file) is broken.
- **Movement errors don't throw** — `MockMovement` returns `{false, "DRONE_HITS_OBSTACLE"}`, which `DroneControl` turns into a `DroneStepStatus::Error`, which `MissionControl` records and stops the mission cleanly (status `Error`, score `-1`). No exception, no crash.
- **Partial results are still saved:** the output map is written even on error/timeout, so a crashed run yields whatever was mapped.
- **Defensive construction:** `Map3DImpl` validates NPY shape/order/dtype and throws descriptive `std::runtime_error`s; `SimulationManager` rejects a null factory with `std::invalid_argument`. These throws are caught upstream and become `-1` scores.
- **Algorithm-level safety valves** (so a bad map never hangs the simulator): `MAX_POPS = 50000` bound on the frontier BFS, the 5-try frontier blacklist, the `stuck_scan_count_ > 10` cutoff, and the sentinel-with-retry termination. The algorithm **always** returns a valid command or a `Finished*` status — it can't loop forever.

---

## 9. A Worked Example — One `nextStep()` Call, Start to Finish

Trace what happens the first few steps in a normal scenario (drone starts *inside* the map):

1. **Call 1:** `initialized_` is false → `initialize()` runs: reads drone radius → `phys_r2_`, picks `w_struct_` (0 or 4), computes `los_L_` from LiDAR z_max, picks `rot_step_deg_` (5.625 or 11.25) from physical map size, sets `needs_scan_ = true`. `bootstrap_done_` becomes true immediately (cell is in-bounds). `pending_commands_` empty, `needs_scan_` true → **enqueue a full 360° sweep** (say 32 rotate+scan commands), pop the first, return it. The drone rotates 11.25° and scans; `ScanResultToVoxels` carves ~`Empty` cells and marks the walls it saw `Occupied`.
2. **Calls 2–32:** drain the rest of the sweep, one rotate+scan per call. By the end, the neighbourhood is largely mapped; a ring of `Unmapped` cells remains at the LiDAR's range.
3. **Call 33:** `pending_commands_` empty, `needs_scan_` now false → **`selectBestFrontier(cur)`**. BFS expands navigable cells, scores each frontier by `(gain + w_struct·structure)/(1+β·dist)`, returns the best (nearest high-information boundary not in the recent-8 and not blacklisted). Increment its try-count, push to `recent_targets_`.
4. Still call 33: **`enqueueNavigationTo(cur, frontier)`** → A* path → rotate/advance/elevate commands queued; `needs_scan_ = true`. Pop the first movement command, return it.
5. **Following calls:** drain the movement queue (drone flies toward the frontier). On arrival, queue is empty and `needs_scan_` is true → a fresh 360° sweep fires. Repeat from step 3.
6. **Eventually:** no frontier remains (or the drone is stuck) → return `FinishedWithUnmappableVoxels`; `DroneControl` reports `Completed`; `MissionControl` saves the map; the run is scored.

**Edge trace (house scenario, drone below the map):** Call 1 sees the drone's cell is `OutOfBounds` and `bootstrap_done_` is false → `navigateToMap()` returns a single Elevate command toward `min_height − 0.5 cm`, **no scan**. This repeats until the drone is within one voxel of the boundary, then `bootstrap_done_` flips true and the normal sweep begins.

---

## 10. Testing Strategy (the *other* half of the assignment)

The grader **injects intentional bugs** into a component and expects:
- Component tests: the *matching* component's tests catch **≥60%** of its bugs; *unrelated* component tests must **not** break.
- Integration tests: catch **≥20%** of injected bugs.

**Component tests** (`tests/components/`): one per component — `SimulationManager`, `SimulationRun` (also exercises the Mock GPS & Movement), `MissionControl`, `DroneControl`, `MappingAlgorithm`, `MockLidar`, `MapsComparison`. Each tests happy path + error cases in isolation, using **GMock** mocks for dependencies so a failure localizes to *that* component.

**Integration tests** (`tests/integration/`): full end-to-end flow — one with the **real** `MappingAlgorithmImpl`, one with a **mock algorithm** — to verify the wiring/flow independent of exploration quality.

Run filters (spec-mandated): `--gtest_filter=MappingAlgorithm.*`, `Integration.*`, etc.

**Why mocks matter here:** injecting the mock LiDAR/movement/algorithm lets each test assert *"given this input, this component produces exactly this output"* without the nondeterminism of a full simulation — that's what makes the 60%/20% bug-detection targets achievable and keeps unrelated tests stable.

---

## 11. Likely Exam Questions — Quick Answers

- **"Why frontier-based exploration?"** Unknown environment; frontiers are exactly the boundary between known-free and unknown space, so going there maximizes new information per move. Greedy-by-value + distance penalty is simple, robust, and terminates.
- **"How do you guarantee it terminates?"** Anti-oscillation (last 8 excluded) + per-frontier blacklist after 5 tries + `stuck_scan_count_ > 10` + the sentinel-with-one-retry in `selectBestFrontier`. No infinite loop is possible.
- **"Why two navigability predicates?"** `navigable` (loose) lets you *stand at* a frontier next to unknown space; `clearForBody` (strict) keeps the *path* away from possibly-solid Unmapped walls to avoid collisions. Vertical Unmapped is tolerated because LiDAR can't cover it.
- **"Why F1 instead of accuracy?"** Occupied voxels are sparse; accuracy is dominated by empty background. F1 balances precision & recall on the occupied class.
- **"Why `mp-units`?"** Compile-time unit safety — mixing lengths/angles/axes is a compile error, eliminating a whole class of silent geometry bugs.
- **"What's the hardest bug you fixed?"** The latent `DRONE_HITS_OBSTACLE` at extended budgets (Fix 3): the drone left the mission area into regions of false-Empty walls. Fixed by enforcing mission bounds in *all three* of BFS, A*, and frontier detection.
- **"What would you improve next?"** Sphere-accurate collision in `MockMovement` (currently center-path only), z-aware frontiers done *safely* (only when LiDAR coverage can resolve them), and a smarter global planner instead of greedy nearest-value.
- **"Why does the algorithm read the output map and not the raw scan?"** The output map is the *fused, authoritative* model — `ScanResultToVoxels` has already merged the scan with all prior observations using the evidence-priority policy. Reading raw scans would mean re-deriving that state every step. That's why `nextStep` ignores `latest_scan`.
- **"Where does `Unmapped` come from and why does it drive everything?"** The output map starts all-`Unmapped`. Scans convert cells to `Empty`/`Occupied`/`PotentiallyOccupied`. A frontier is precisely a navigable cell *touching* `Unmapped` — so exploration is literally "chase the `Unmapped`."
- **"What's the difference between the mission bounds and the map bounds?"** Map bounds = the full array extent (what gets scored). Mission bounds = the sub-region the drone is *allowed* to explore this run. `withinMissionBounds` enforces the latter; scoring uses the former.
- **"Why `4^k` beams per LiDAR circle?"** Outer rings cover more solid angle, so they need exponentially more beams to keep angular density roughly constant. `fov_circles` trades coverage vs. compute.
- **"How does a collision actually get detected?"** `MockMovement` samples the drone's *center* along the motion in ~1 cm steps against the **hidden** map; any `Occupied` sample → `DRONE_HITS_OBSTACLE`, movement rejected. The algorithm must therefore keep its own body-clearance (sphere check) so it never commands such a move.
- **"Is A* optimal here?"** Yes for the path it's given — Manhattan is admissible & consistent on a 6-connected unit-cost grid. But the *frontier choice* is greedy, so global exploration isn't optimal (and doesn't need to be for ex2).

---

## 12. Common Pitfalls & Gotchas (things graders probe)

- **Center vs. corner:** always query occupancy at the voxel **center** (`+0.5`). Off-by-half-voxel bugs silently corrupt navigation.
- **Integer vs. float radius:** the sphere check uses the **float** `phys_r2_`, not the ceil'd `radius_voxels_`. Using the integer would inflate a 1.5-voxel drone to 2 and wrongly block valid openings. `radius_voxels_` is only the *loop bound*.
- **`w_struct_` must be 0 for sub-voxel drones:** otherwise scoring pushes them to wall-adjacent cells they can't safely occupy → crashes. (Reverted "always 4" attempt.)
- **Never add ±z to frontiers naively:** limited LiDAR elevation means vertical `Unmapped` can't be resolved → infinite unresolvable frontiers (reverted Fix 4).
- **Resolution must match for scoring:** output vs hidden grid misalignment silently tanks the score (`output_mapping_resolution_factor`).
- **`PotentiallyOccupied` is not `Occupied`:** it doesn't score as a wall, and `navigable`/`clearForBody` treat it as an obstacle (conservative — don't fly into uncertainty).
- **Scans are horizontal-only (`altitude = 0`):** tilted scans are assignment-illegal; every scan request is `{0° horizontal-offset, 0° altitude}`.
- **The output map is saved even on failure** — don't assume a crashed run has no map.

---

## 13. Mini-Glossary

| Term | Meaning |
|---|---|
| **Voxel** | A 3-D pixel — one cubic cell of the map grid. |
| **Frontier** | A navigable `Empty` cell with an `Unmapped` horizontal neighbour — the edge of the known world. |
| **Hidden / ground-truth map** | The real environment; only the simulation (LiDAR/movement) may read it. |
| **Output map** | The drone's reconstruction; starts all-`Unmapped`, gets scored. |
| **Occupancy grid** | A map that stores per-voxel state (free/occupied/unknown). |
| **BFS** | Breadth-first search — used to expand navigable space & measure hop-distance to frontiers. |
| **A\*** | Best-first shortest path with an admissible heuristic (Manhattan) — used for navigation. |
| **Admissible heuristic** | Never overestimates true cost → A* returns an optimal path. |
| **F1 score** | Harmonic-mean balance of precision & recall: `2TP/(2TP+FP+FN)`. |
| **TP / FP / FN** | True positive (wall found), false positive (invented wall), false negative (missed wall). |
| **Sweep** | A full 360° yaw rotation with a scan at each step, done on arrival at a waypoint. |
| **Bootstrap** | Moving the drone into the map array before the first scan (house scenario). |
| **`mp-units`** | Compile-time units/dimensional-analysis library — makes unit mixups a compile error. |
| **RAII** | Resource Acquisition Is Initialization — lifetime tied to object scope (`unique_ptr`). |
| **DI** | Dependency Injection — pass collaborators in via constructor/interfaces. |
| **Row-major** | Array layout where the last index varies fastest: `idx = x·(Y·Z)+y·Z+z`. |

---

## 14. File Map (where to point during the exam)

| Concern | File |
|---|---|
| **Your algorithm** | `src/MappingAlgorithmImpl.cpp`, `include/.../MappingAlgorithmImpl.h` |
| Strong-typed units | `include/.../Units.h` |
| Voxel/map types | `include/.../types/MapTypes.h`, `DroneTypes.h` |
| Scan fusion policy | `src/ScanResultToVoxels.cpp` |
| LiDAR ray casting | `src/MockLidar.cpp` |
| Collision model | `src/MockMovement.cpp` |
| Scoring (F1) | `src/MapsComparison.cpp` |
| Step loop | `src/DroneControlImpl.cpp` |
| Wiring/ownership | `src/SimulationRunFactoryImpl.cpp`, `SimulationRunImpl.cpp` |
| Design + diagrams | `docs/HLD.md` |

**Build & run:**
```bash
cmake --build build --parallel 4
./build/drone_mapper_simulation_test                              # all tests
./build/drone_mapper_simulation_test --gtest_filter=MappingAlgorithm.*
./build/drone_mapper_simulation scenarios/scenario1/simulation.yaml sc1_out/
./build/maps_comparison origin.npy target.npy                    # prints one number
```
