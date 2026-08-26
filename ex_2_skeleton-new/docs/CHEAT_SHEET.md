# Drone Mapper — Exam Cheat Sheet 🚁

> One page. Skim before the exam. Full detail in `STUDY_GUIDE.md`.

## 30-second pitch
A simulated drone explores an **unknown 3D world** with a **LiDAR**, incrementally builds a **voxel occupancy map**, and the simulator **scores** it against a **hidden ground-truth map**. Refactoring + testing exercise. **Only `MappingAlgorithmImpl.{h,cpp}` is my code.**

## Two maps (never confuse them)
- **Hidden map** = ground truth. Only sim side (MockLidar, MockMovement) reads it.
- **Output map** = drone's reconstruction. Starts all-`Unmapped`. Gets scored.

## `VoxelOccupancy`
`PotentiallyOccupied(-3)` · `OutOfBounds(-2)` · `Unmapped(-1)` · `Empty(0)` · `Occupied(1)`
Frontiers chase **Unmapped**. Only **Occupied** counts for scoring.

## Architecture (interface + impl, DI everywhere)
`SimulationManager`→`SimulationRunFactory`→`SimulationRunImpl`→`MissionControl`→`DroneControl`→`MappingAlgorithm`
Sensors: `MockLidar`, `MockGPS`, `MockMovement`. Map: `IMap3D`(read)/`IMutableMap3D`(write) = `Map3DImpl`.
- **Ownership:** `SimulationRunImpl` owns all via `unique_ptr`; everyone else borrows via `&`.
- **Factory** = single construction seam → mockable in manager tests.
- **`IMap3D` vs `IMutableMap3D`** = interface segregation; algorithm gets **read-only** handle.

## The step loop (memorize order)
```
DroneControl.step():
  state = gps.pos+heading
  cmd   = mappingAlgorithm.nextStep(state, lastScan)   ← MY CODE
  if Finished* → stop
  MOVE first (rotate/advance/elevate)  → may return DRONE_HITS_OBSTACLE
  THEN scan → ScanResultToVoxels.applyToMap(output)
```
Movement **before** scan. Algorithm may get `latest_scan == nullptr` (and ignores it — reads output map).

## ALGORITHM = frontier-based exploration
`nextStep()` priority: **1 Bootstrap → 2 Drain queue → 3 Scan sweep → 4 Select frontier → 5 Plan path**

**Frontier** = navigable `Empty` cell with an **Unmapped horizontal** neighbour (dz=0 only — vertical ignored: LiDAR can't resolve it).

**Selection = value-scored BFS**, score:
```
score = (gain + w_struct·structure) / (1 + β·dist)
```
- gain = # Unmapped in L³ cube · structure = # Occupied in cube · dist = BFS hops
- **w_struct = 4** (radius ≥ 1 voxel) or **0** (small drone) · **β = 0.15**
- Guards: last **8** targets excluded (anti-oscillation); **≥5 tries → blacklist**.

**Two navigability levels:**
| | Empty | Sphere clear | Unmapped horiz. face |
|--|--|--|--|
| `navigable()` (BFS/frontier) | ✓ | ✓ | **allowed** |
| `clearForBody()` (A* path) | ✓ | ✓ | **blocked** |
Sphere check uses **float `phys_r2_`** (not int radius). Vertical Unmapped always allowed.

**A\*:** 6-connected grid · Manhattan heuristic (admissible) · `priority_queue`+`greater` min-heap · partial-path fallback to closest cell if goal unreachable.

**Path→commands:** dz≠0 → Elevate; else `atan2(dy,dx)` → Rotate to bearing → Advance. All chunked by max_advance/elevate/rotate.

**Sweep:** 360° yaw, `ceil(360/step)` rotate+scan. Step = **5.625°(64)** if map <200cm else **11.25°(32)**.

**Terminate** `FinishedWithUnmappableVoxels`: no frontier after clearing guard+retry, OR `stuck_scan_count_ > 10`.

## Supporting components
- **MockLidar:** center beam + circles; circle k has **4^k** beams; ray-march 0.1·res steps to z_max. Hit<z_min→0 (→PotentiallyOccupied); no hit→max (miss→Empty).
- **ScanResultToVoxels:** hit→before=Empty, point=Occupied; miss→all Empty; too-close→PotentiallyOccupied. Conflict priority **Occupied(3)>Empty(2)>PotentiallyOccupied(1)>Unmapped(0)** — stronger wins.
- **MockMovement:** clamps to max; samples **center path** ~1cm vs hidden map; Occupied→`DRONE_HITS_OBSTACLE` (no move).
- **Map3DImpl:** input mode = load `.npy`, nonzero→Occupied; output mode = `int8` vector init Unmapped. `toIndex = floor((pos−min)/res)`; `flatIndex = x·Y·Z+y·Z+z` (row-major).
- **MapsComparison:** **F1×100** = `200·TP/(2TP+FP+FN)`. Both empty→100, one empty→0. Standalone exe prints one number.

## Coordinate transforms
- world→voxel: `floor((pos−min)/res)` · voxel→center: `min+(i+0.5)·res` (**+0.5!**)
- Algorithm copies `worldToVoxel` exactly from `Map3DImpl` → grids never drift.

## Problems I solved (fixes)
| Fix | Problem → Solution |
|--|--|
| **1 Bootstrap** | Drone starts outside map (house, z below) → Elevate/Advance to boundary before first scan, no scan. |
| **2 Adaptive step** | Old raw-voxel threshold halved visits on 200cm maps → threshold on **physical** size (<200cm→5.625°, else 11.25°). |
| **3 Mission bounds** | Latent `DRONE_HITS_OBSTACLE` at long budgets → enforce `withinMissionBounds` in BFS+A*+isFrontier. |
| **sc3 blacklist** | Infinite loop on shadowed frontier → 5-try permanent blacklist. |
| **Config** | Output 5cm vs hidden 10cm mismatch → `output_mapping_resolution_factor: 2`. |
| ~~4 z-frontiers~~ | **REVERTED**: flooded queue with unresolvable vertical frontiers, −30–45%. |
| ~~w_struct=4 always~~ | **REVERTED**: drove small drones into walls → crashes. |

## C++ highlights (say these!)  — built as **C++20**, `-Wall -Wextra -Werror -pedantic`, deps via vcpkg
- **`mp-units`** — compile-time dimensional analysis. `XLength≠YLength≠PhysicalLength`, angles typed; mixing = compile error. Extract raw only via **`force_numerical_value_in(cm)`** ("force" = deliberate unit-safety escape hatch). Unit-aware `si::sin/cos/atan2`. **The standout feature.**
- **RAII + `unique_ptr`** ownership tree (borrowers use `&`) · ctors **validate+throw** · **sink params + `std::move`** · `shared_ptr<NpyArray>` for map data.
- **DI via interfaces** · **Factory** (single seam, mockable) · **Interface Segregation** (`IMutableMap3D:IMap3D`) · `final`+`override` · **inheriting ctor** (`using IMappingAlgorithm::IMappingAlgorithm;`).
- **Custom `std::hash<GridCell3D>`** (hash-combine, `0x9e3779b9`) → O(1) BFS/A* sets. `operator==`/`<` on cell.
- `priority_queue<pair<int,Cell>,…,greater>` = min-heap on f (pair compares lexicographically) · `queue`(BFS) · `deque`(command queue + recent-8 ring) · `unordered_map/set`.
- **Structured bindings** (`auto[gain,structure]=…`, `auto[it,ins]=map.emplace(…)` nav-cache) · **lambdas** (isNav/countCube/heuristic/reconstruct) · **`std::optional`+`nullopt`** · **`explicit operator bool`** (`if(!result)`) · **`[[nodiscard]]`/`noexcept`** · **`constexpr`** helpers in anon ns · `enum class` · C++20 `<numbers>::pi`, `std::clamp`.

## ⚠️ Limitations / what we're NOT good at (state honestly!)
- **Greedy, not optimal** — local argmax frontier, backtracks/wastes steps; no global route planner. (ex2 doesn't require smart algo.)
- **No vertical exploration** — z-frontiers ignored (LiDAR can't resolve them) → tall/multi-floor structure under-mapped.
- **Room scenarios step-limited** (~12–36%); reach geo-max only at 10–20× budget. **House = 0%** (physically inaccessible).
- **Latent `DRONE_HITS_OBSTACLE`** at *extended* budgets (small_out long ~8k steps, small_room ~1.6k) — Fix 3 *contains* it within real budget but doesn't fully cure it.
- **Recomputes everything each waypoint** — full BFS (cap `MAX_POPS=50000`, can truncate huge maps) + O(L³) `countCube` per candidate; nothing incremental. `stateOf` round-trips through floats each neighbor.
- **Ignores `latest_scan`** — reads output map; relies on harness applying scan *before* next `nextStep` (implicit coupling).
- **Many magic constants** (200cm, w_struct=4, β=0.15, 5-try, recent-8, stuck>10) — hand-tuned, may not generalize.
- **Sim collision = center-path only** (not sphere); algo is *stricter* than sim → conservative, may refuse legal moves.
- **Algo tests are structural** (rotates, finishes, doesn't crash) — don't verify mapping *quality*.
- Skeleton uses **exact float `==`** for lidar miss/too-close sentinels — brittle in general.

## Testing
- Grader injects bugs: matching component test catches **≥60%**, integration **≥20%**, unrelated tests **must not break**.
- 7 component tests (`tests/components/`) + 2 integration (real algo + mock algo). GMock isolates.
- Filters: `--gtest_filter=MappingAlgorithm.*`, `Integration.*`, …

## Error handling
- Immediate log to `error.log` · per-run `try/catch` → score **−1**, batch continues · movement error ≠ throw (returns bool) · output map saved even on failure · BFS `MAX_POPS=50000`, blacklist, stuck cutoff = can't hang.

## Rapid-fire answers
- **Why frontiers?** Boundary of known/unknown → max info per move.
- **Why terminates?** recent-8 + 5-try blacklist + stuck>10 + sentinel-retry.
- **Why 2 nav predicates?** stand-at-frontier (loose) vs safe-path (strict).
- **Why F1?** occupied sparse; accuracy dominated by empty background.
- **Why read output map not raw scan?** it's the fused authoritative state.
- **Why 4^k beams?** keep angular density ~constant over larger solid angle.
- **Collision?** center-path sampled vs hidden map; algo keeps own sphere clearance.
- **Improve next?** sphere-accurate collision, safe z-frontiers, non-greedy planner.

## Commands
```
cmake --build build --parallel 4
./build/drone_mapper_simulation_test --gtest_filter=MappingAlgorithm.*
./build/drone_mapper_simulation scenarios/scenario1/simulation.yaml sc1_out/
./build/maps_comparison origin.npy target.npy
```
