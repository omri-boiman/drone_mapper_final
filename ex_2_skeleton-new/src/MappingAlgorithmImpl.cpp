#include <drone_mapper/MappingAlgorithmImpl.h>
#include <drone_mapper/IMap3D.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <numbers>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace drone_mapper {

namespace {
constexpr double toCm(PhysicalLength l)  { return l.force_numerical_value_in(cm); }
constexpr double toDeg(HorizontalAngle a){ return a.force_numerical_value_in(deg); }

const std::array<types::GridCell3D, 6> kDirs = {{
    {1, 0, 0}, {-1, 0, 0},
    {0, 1, 0}, {0, -1, 0},
    {0, 0, 1}, {0, 0, -1},
}};
} // namespace

// ---------------------------------------------------------------------------
// Initialization
// ---------------------------------------------------------------------------

void MappingAlgorithmImpl::initialize(const types::DroneState& /*state*/) {
    max_rotate_  = drone_config_.max_rotate;
    max_advance_ = drone_config_.max_advance;
    max_elevate_ = drone_config_.max_elevate;
    map_cfg_     = output_map_.getMapConfig();
    res_cm_      = toCm(map_cfg_.resolution);
    if (res_cm_ <= 0.0) res_cm_ = 1.0;
    const double radius_cm = toCm(drone_config_.radius);
    const double phys_r    = radius_cm / res_cm_;
    phys_r2_        = phys_r * phys_r;
    radius_voxels_  = (phys_r > 0.0) ? static_cast<int>(std::ceil(phys_r)) : 1;
    const double z_max_cm = toCm(lidar_config_.z_max);
    los_L_ = (z_max_cm > 0.0) ? std::max(2.0, std::ceil(z_max_cm / res_cm_ / 4.0)) : 3.0;
    // Small drones (r² < 1) fit through any opening and never escape to the roof, so
    // the structure weight only adds noise inside the building.  Disable it for them.
    // Medium/large drones need it to prefer interior (many walls) over exterior sky.
    w_struct_ = (phys_r2_ >= 1.0) ? 4.0 : 0.0;
    // [CHANGE: Fix 2 — Use physical map size to choose sweep rotation step]
    // Use finer 5.625° step (64 rotations) for maps whose largest physical dimension is
    // < 200 cm — these are the small 1 cm-resolution test maps where angular precision
    // matters.  For large physical maps (≥ 200 cm, e.g. the 10 cm-resolution input
    // scenarios), use 11.25° (32 rotations) to double the number of frontier visits
    // within the fixed step budget.  The old threshold was raw voxel count ≤ 20, which
    // accidentally put the 20-voxel × 10 cm = 200 cm map in the fine-step bucket,
    // cutting available frontier visits by half.
    const auto& b   = map_cfg_.boundaries;
    const double xv = (b.max_x.force_numerical_value_in(cm) - b.min_x.force_numerical_value_in(cm)) / res_cm_;
    const double yv = (b.max_y.force_numerical_value_in(cm) - b.min_y.force_numerical_value_in(cm)) / res_cm_;
    const double zv = (b.max_height.force_numerical_value_in(cm) - b.min_height.force_numerical_value_in(cm)) / res_cm_;
    rot_step_deg_   = (std::max({xv, yv, zv}) * res_cm_ < 200.0) ? 5.625 : 11.25;
    // [END CHANGE: Fix 2]
    initialized_    = true;
    needs_scan_     = true;
}

// ---------------------------------------------------------------------------
// Voxel <-> world conversion (mirrors Map3DImpl::toIndex)
// ---------------------------------------------------------------------------

MappingAlgorithmImpl::Cell
MappingAlgorithmImpl::worldToVoxel(const Position3D& pos) const {
    const auto& b = map_cfg_.boundaries;
    return {
        static_cast<int>(std::floor(
            (pos.x.force_numerical_value_in(cm) - b.min_x.force_numerical_value_in(cm)) / res_cm_)),
        static_cast<int>(std::floor(
            (pos.y.force_numerical_value_in(cm) - b.min_y.force_numerical_value_in(cm)) / res_cm_)),
        static_cast<int>(std::floor(
            (pos.z.force_numerical_value_in(cm) - b.min_height.force_numerical_value_in(cm)) / res_cm_)),
    };
}

Position3D MappingAlgorithmImpl::centerOf(const Cell& c) const {
    const auto& b = map_cfg_.boundaries;
    return {
        (b.min_x.force_numerical_value_in(cm)      + (c.x + 0.5) * res_cm_) * x_extent[cm],
        (b.min_y.force_numerical_value_in(cm)      + (c.y + 0.5) * res_cm_) * y_extent[cm],
        (b.min_height.force_numerical_value_in(cm) + (c.z + 0.5) * res_cm_) * z_extent[cm],
    };
}

types::VoxelOccupancy MappingAlgorithmImpl::stateOf(const Cell& c) const {
    return output_map_.atVoxel(centerOf(c));
}

// ---------------------------------------------------------------------------
// Navigability and frontier detection
// ---------------------------------------------------------------------------

bool MappingAlgorithmImpl::navigable(const Cell& c) const {
    if (stateOf(c) != types::VoxelOccupancy::Empty) return false;
    // Use physical (float) r² so a 1.5-voxel-radius drone doesn't get inflated to 2
    // by integer rounding — that would wrongly block passage through 4-wide openings.
    for (int dx = -radius_voxels_; dx <= radius_voxels_; ++dx) {
        for (int dy = -radius_voxels_; dy <= radius_voxels_; ++dy) {
            for (int dz = -radius_voxels_; dz <= radius_voxels_; ++dz) {
                if (dx == 0 && dy == 0 && dz == 0) continue;
                if (static_cast<double>(dx*dx + dy*dy + dz*dz) > phys_r2_) continue;
                const auto s = stateOf({c.x + dx, c.y + dy, c.z + dz});
                if (s == types::VoxelOccupancy::Occupied          ||
                    s == types::VoxelOccupancy::PotentiallyOccupied ||
                    s == types::VoxelOccupancy::OutOfBounds)
                    return false;
            }
        }
    }
    return true;
}

bool MappingAlgorithmImpl::clearForBodyKnown(const Cell& c) const {
    if (stateOf(c) != types::VoxelOccupancy::Empty) return false;
    for (int dx = -radius_voxels_; dx <= radius_voxels_; ++dx) {
        for (int dy = -radius_voxels_; dy <= radius_voxels_; ++dy) {
            for (int dz = -radius_voxels_; dz <= radius_voxels_; ++dz) {
                if (dx == 0 && dy == 0 && dz == 0) continue;
                if (static_cast<double>(dx*dx + dy*dy + dz*dz) > phys_r2_) continue;
                const auto s = stateOf({c.x + dx, c.y + dy, c.z + dz});
                if (s == types::VoxelOccupancy::Occupied           ||
                    s == types::VoxelOccupancy::PotentiallyOccupied ||
                    s == types::VoxelOccupancy::OutOfBounds)
                    return false;
            }
        }
    }
    return true;
}

bool MappingAlgorithmImpl::clearForBody(const Cell& c) const {
    if (!clearForBodyKnown(c)) return false;
    // Only check HORIZONTAL face-neighbours for Unmapped status.
    // Vertical (z) neighbours are routinely Unmapped in maps with low fov_circles lidar
    // because the lidar has limited elevation coverage; blocking on them causes deadlocks
    // in building interiors.  Horizontal Unmapped neighbours are the real collision risk:
    // wall/obstacle surfaces in x-y cause crashes when the drone body just touches them.
    for (const auto& d : kDirs) {
        if (d.z != 0) continue;   // vertical direction — skip
        if (stateOf({c.x + d.x, c.y + d.y, c.z + d.z}) == types::VoxelOccupancy::Unmapped)
            return false;
    }
    return true;
}

bool MappingAlgorithmImpl::isFrontier(const Cell& c) const {
    if (!navigable(c)) return false;
    if (!withinMissionBounds(c)) return false;
    // Only horizontal (dz=0) face-neighbours count.  Vertical Unmapped neighbours
    // are routinely present due to limited lidar elevation coverage (fov_circles) and
    // can never be fully resolved from accessible positions, so treating them as
    // frontier triggers infinite exploration loops without score benefit.
    for (const auto& d : kDirs) {
        if (d.z != 0) continue;
        if (stateOf({c.x + d.x, c.y + d.y, c.z + d.z}) == types::VoxelOccupancy::Unmapped)
            return true;
    }
    return false;
}

// [CHANGE: Fix 3 — Mission bounds enforcement]
// Returns true if the voxel center lies within mission_config_.mission_bounds.
// When mission_bounds is unset (max_x <= min_x), all cells pass — no restriction.
// This prevents the drone from navigating into building sections or altitude ranges
// that are outside the mission area, which could cause DRONE_HITS_OBSTACLE crashes
// when the algorithm explores areas with false-Empty wall voxels.
bool MappingAlgorithmImpl::withinMissionBounds(const Cell& c) const {
    const auto& mb = mission_config_.mission_bounds;
    const double max_x = mb.max_x.force_numerical_value_in(cm);
    const double min_x = mb.min_x.force_numerical_value_in(cm);
    if (max_x <= min_x) return true;  // unset — no restriction
    const Position3D center = centerOf(c);
    const double cx = center.x.force_numerical_value_in(cm);
    const double cy = center.y.force_numerical_value_in(cm);
    const double cz = center.z.force_numerical_value_in(cm);
    return cx >= min_x && cx <= max_x &&
           cy >= mb.min_y.force_numerical_value_in(cm) &&
           cy <= mb.max_y.force_numerical_value_in(cm) &&
           cz >= mb.min_height.force_numerical_value_in(cm) &&
           cz <= mb.max_height.force_numerical_value_in(cm);
}
// [END CHANGE: Fix 3]

// ---------------------------------------------------------------------------
// Value-based frontier selection (Change 1 + 2)
// ---------------------------------------------------------------------------

MappingAlgorithmImpl::Cell
MappingAlgorithmImpl::selectBestFrontier(const Cell& from) const {
    // Cache navigability to avoid recomputing the sphere check per cell.
    std::unordered_map<Cell, bool, CellHash> nav_cache;
    auto isNav = [&](const Cell& c) -> bool {
        auto [it, ins] = nav_cache.emplace(c, false);
        if (!ins) return it->second;
        return (it->second = navigable(c));
    };

    // O(1) lookup for the anti-oscillation guard (Change 2).
    std::unordered_set<Cell, CellHash> recent_set(recent_targets_.begin(), recent_targets_.end());

    // Blended score: (gain + w_struct_ * structure) / (1 + beta_ * dist)
    // w_struct_=0 for small drones (pure gain/dist ≈ nearest frontier — no escape risk).
    // w_struct_=4 for medium/large drones (interior wins over exterior sky by structure count).
    const int L = static_cast<int>(los_L_);

    auto countCube = [&](const Cell& c) -> std::pair<int,int> {
        int gain = 0, structure = 0;
        for (int dx = -L; dx <= L; ++dx)
            for (int dy = -L; dy <= L; ++dy)
                for (int dz = -L; dz <= L; ++dz) {
                    const auto s = stateOf({c.x+dx, c.y+dy, c.z+dz});
                    if      (s == types::VoxelOccupancy::Unmapped)           ++gain;
                    else if (s == types::VoxelOccupancy::Occupied ||
                             s == types::VoxelOccupancy::PotentiallyOccupied) ++structure;
                }
        return {gain, structure};
    };

    std::unordered_map<Cell, int, CellHash> dist;
    std::queue<Cell> q;

    dist[from] = 0;

    // Seed from `from`'s navigable neighbors (from itself may not be carved yet).
    for (const auto& d : kDirs) {
        const Cell nb{from.x+d.x, from.y+d.y, from.z+d.z};
        if (dist.count(nb)) continue;
        if (stateOf(nb) == types::VoxelOccupancy::OutOfBounds) continue;
        if (!withinMissionBounds(nb)) continue;
        dist[nb] = 1;
        if (isNav(nb)) q.push(nb);
    }

    Cell   best{std::numeric_limits<int>::min(), 0, 0};
    double best_score = -1.0;

    auto tryCandidate = [&](const Cell& c, int d_val) {
        auto [gain, structure] = countCube(c);
        const double sc = (gain + w_struct_ * structure) / (1.0 + beta_ * d_val);
        if (sc > best_score) { best_score = sc; best = c; }
    };

    // Also check `from` itself.
    auto notExhausted = [&](const Cell& c) {
        auto it = frontier_try_count_.find(c);
        return it == frontier_try_count_.end() || it->second < 5;
    };
    if (isFrontier(from) && !recent_set.count(from) && notExhausted(from))
        tryCandidate(from, 0);

    int pops = 0;
    constexpr int MAX_POPS = 50000;

    while (!q.empty() && pops < MAX_POPS) {
        const Cell cur = q.front(); q.pop();
        ++pops;
        const int d_cur = dist[cur];

        if (!recent_set.count(cur) && notExhausted(cur)) {
            // cur is navigable (only navigable cells are enqueued).
            for (const auto& d : kDirs) {
                if (stateOf({cur.x+d.x, cur.y+d.y, cur.z+d.z}) == types::VoxelOccupancy::Unmapped) {
                    tryCandidate(cur, d_cur);
                    break;
                }
            }
        }

        for (const auto& d : kDirs) {
            const Cell nb{cur.x+d.x, cur.y+d.y, cur.z+d.z};
            if (dist.count(nb)) continue;
            if (stateOf(nb) == types::VoxelOccupancy::OutOfBounds) continue;
            if (!withinMissionBounds(nb)) continue;
            dist[nb] = d_cur + 1;
            if (isNav(nb)) q.push(nb);
        }
    }
    return best;
}

// ---------------------------------------------------------------------------
// 3D A*: path over navigable voxels
// ---------------------------------------------------------------------------

std::vector<MappingAlgorithmImpl::Cell>
MappingAlgorithmImpl::findPath3D(const Cell& from, const Cell& to) const {
    if (from == to) return {from};

    auto heuristic = [](const Cell& a, const Cell& b) {
        return std::abs(a.x - b.x) + std::abs(a.y - b.y) + std::abs(a.z - b.z);
    };

    auto reconstruct = [&](const std::unordered_map<Cell, Cell, CellHash>& parent,
                           const Cell& end) {
        std::vector<Cell> path;
        for (Cell n = end; !(n == from); n = parent.at(n))
            path.push_back(n);
        path.push_back(from);
        std::reverse(path.begin(), path.end());
        return path;
    };

    using Entry = std::pair<int, Cell>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
    std::unordered_map<Cell, int,  CellHash> g;
    std::unordered_map<Cell, Cell, CellHash> parent;
    std::unordered_set<Cell, CellHash>       closed;

    g[from] = 0;
    parent[from] = from;
    open.push({heuristic(from, to), from});

    // Track the closest clearForBody cell reached so far for a partial-path fallback.
    Cell best_partial   = from;
    int  best_h_partial = heuristic(from, to);

    while (!open.empty()) {
        auto [f, cur] = open.top(); open.pop();
        if (closed.count(cur)) continue;
        closed.insert(cur);

        if (cur == to) return reconstruct(parent, to);

        // Update best-partial even if goal unreachable (cur is always clearForBody here).
        const int h_cur = heuristic(cur, to);
        if (h_cur < best_h_partial) { best_h_partial = h_cur; best_partial = cur; }

        for (const auto& d : kDirs) {
            const Cell nb{cur.x + d.x, cur.y + d.y, cur.z + d.z};
            if (closed.count(nb)) continue;
            if (!clearForBody(nb)) continue;
            if (!withinMissionBounds(nb)) continue;
            const int tg = g[cur] + 1;
            if (!g.count(nb) || tg < g[nb]) {
                g[nb] = tg;
                parent[nb] = cur;
                open.push({tg + heuristic(nb, to), nb});
            }
        }
    }

    // Goal unreachable via clear space — return path to the closest clear cell we reached.
    if (!(best_partial == from))
        return reconstruct(parent, best_partial);
    return {from};  // couldn't move at all
}

// ---------------------------------------------------------------------------
// Command generation
// ---------------------------------------------------------------------------

void MappingAlgorithmImpl::enqueueRotateToAngle(double target_deg, double& current_deg) {
    double diff = target_deg - current_deg;
    while (diff >  180.0) diff -= 360.0;
    while (diff < -180.0) diff += 360.0;

    const auto dir = (diff >= 0.0) ? types::RotationDirection::Right
                                   : types::RotationDirection::Left;
    double remaining = std::abs(diff);
    const double max_rot = toDeg(max_rotate_);

    while (remaining > 0.5) {
        const double step = std::min(remaining, max_rot);
        pending_commands_.push_back({
            types::MovementCommandType::Rotate, dir,
            step * horizontal_angle[deg], 0.0 * cm,
        });
        current_deg += (dir == types::RotationDirection::Right) ? step : -step;
        while (current_deg >= 360.0) current_deg -= 360.0;
        while (current_deg <    0.0) current_deg += 360.0;
        remaining -= step;
    }
}

void MappingAlgorithmImpl::enqueueNavigationTo(const Cell& from, const Cell& to,
                                                double& heading_deg) {
    const auto path = findPath3D(from, to);
    if (path.size() < 2) return;

    const double max_adv  = toCm(max_advance_);
    const double max_elev = toCm(max_elevate_);

    for (std::size_t i = 1; i < path.size(); ++i) {
        if (!clearForBody(path[i])) break;

        const Position3D p0 = centerOf(path[i - 1]);
        const Position3D p1 = centerOf(path[i]);
        const double dx = p1.x.force_numerical_value_in(cm) - p0.x.force_numerical_value_in(cm);
        const double dy = p1.y.force_numerical_value_in(cm) - p0.y.force_numerical_value_in(cm);
        const double dz = p1.z.force_numerical_value_in(cm) - p0.z.force_numerical_value_in(cm);

        if (std::abs(dz) > 0.1) {
            double diff = dz;
            while (std::abs(diff) > 0.1) {
                const double step = (diff > 0.0)
                    ? std::min(diff,  max_elev)
                    : std::max(diff, -max_elev);
                pending_commands_.push_back({
                    types::MovementCommandType::Elevate,
                    types::RotationDirection::Left,
                    0.0 * horizontal_angle[deg],
                    step * isq::length[cm],
                });
                diff -= step;
            }
        } else {
            double desired = std::atan2(dy, dx) * 180.0 / std::numbers::pi;
            if (desired < 0.0) desired += 360.0;
            enqueueRotateToAngle(desired, heading_deg);

            double dist = std::sqrt(dx * dx + dy * dy);
            while (dist > 0.1) {
                const double step = std::min(dist, max_adv);
                pending_commands_.push_back({
                    types::MovementCommandType::Advance,
                    types::RotationDirection::Left,
                    0.0 * horizontal_angle[deg],
                    step * isq::length[cm],
                });
                dist -= step;
            }
        }
    }
}

// [CHANGE: Fix 1 — Bootstrap navigation]
// Issues one movement command per call to move the drone toward the map array bounds.
// Elevate has priority (most common case: house scenario, drone 140 cm below the map).
// x/y OOB is handled secondarily by rotating toward the map centre then advancing.
// No scan is requested during bootstrap — the lidar range may not reach the map yet.
types::MappingStepCommand MappingAlgorithmImpl::navigateToMap(
        const types::DroneState& state) const {
    const double z     = state.position.z.force_numerical_value_in(cm);
    const double min_z = map_cfg_.boundaries.min_height.force_numerical_value_in(cm);
    const double max_z = map_cfg_.boundaries.max_height.force_numerical_value_in(cm);
    const double x     = state.position.x.force_numerical_value_in(cm);
    const double min_x = map_cfg_.boundaries.min_x.force_numerical_value_in(cm);
    const double max_x = map_cfg_.boundaries.max_x.force_numerical_value_in(cm);
    const double y     = state.position.y.force_numerical_value_in(cm);
    const double min_y = map_cfg_.boundaries.min_y.force_numerical_value_in(cm);
    const double max_y = map_cfg_.boundaries.max_y.force_numerical_value_in(cm);
    const double max_elev = toCm(max_elevate_);
    const double max_adv  = toCm(max_advance_);

    if (z < min_z) {
        // Target just BELOW the map boundary to avoid entering occupied ground voxels.
        // The outer "near_map" check in nextStep() exits bootstrap once we arrive.
        const double target = min_z - 0.5;
        const double dist   = std::min(target - z, max_elev);
        if (dist <= 0.0) return {std::nullopt, std::nullopt, types::AlgorithmStatus::Working};
        const types::MovementCommand cmd{types::MovementCommandType::Elevate,
            types::RotationDirection::Left, 0.0 * horizontal_angle[deg],
            dist * isq::length[cm]};
        return {cmd, std::nullopt, types::AlgorithmStatus::Working};
    }
    if (z > max_z) {
        const double target = max_z + 0.5;
        const double dist   = -(std::min(z - target, max_elev));
        if (dist >= 0.0) return {std::nullopt, std::nullopt, types::AlgorithmStatus::Working};
        const types::MovementCommand cmd{types::MovementCommandType::Elevate,
            types::RotationDirection::Left, 0.0 * horizontal_angle[deg],
            dist * isq::length[cm]};
        return {cmd, std::nullopt, types::AlgorithmStatus::Working};
    }
    // Rotate to face map centre, then advance.
    const double cx = (min_x + max_x) / 2.0;
    const double cy = (min_y + max_y) / 2.0;
    const double target_deg  = std::atan2(cy - y, cx - x) * 180.0 / std::numbers::pi;
    const double heading_deg = state.heading.horizontal.force_numerical_value_in(deg);
    double delta = target_deg - heading_deg;
    while (delta >  180.0) delta -= 360.0;
    while (delta < -180.0) delta += 360.0;
    const double max_rot = toDeg(max_rotate_);
    if (std::abs(delta) > 1.0) {
        const double rot = std::clamp(delta, -max_rot, max_rot);
        const auto dir = rot >= 0.0 ? types::RotationDirection::Right
                                    : types::RotationDirection::Left;
        const types::MovementCommand cmd{types::MovementCommandType::Rotate,
            dir, std::abs(rot) * horizontal_angle[deg], 0.0 * cm};
        return {cmd, std::nullopt, types::AlgorithmStatus::Working};
    }
    const double dist = std::min(max_adv,
        std::sqrt((cx - x) * (cx - x) + (cy - y) * (cy - y)));
    const types::MovementCommand cmd{types::MovementCommandType::Advance,
        types::RotationDirection::Left, 0.0 * horizontal_angle[deg],
        dist * isq::length[cm]};
    return {cmd, std::nullopt, types::AlgorithmStatus::Working};
}
// [END CHANGE: Fix 1]

// ---------------------------------------------------------------------------
// Main step
// ---------------------------------------------------------------------------

types::MappingStepCommand MappingAlgorithmImpl::nextStep(
    const types::DroneState& state,
    const types::LidarScanResult* /*latest_scan*/) {

    if (!initialized_) initialize(state);
    if (done_) return {std::nullopt, std::nullopt, types::AlgorithmStatus::Finished};

    // [CHANGE: Fix 1 — Bootstrap navigation]
    // If the drone starts outside the output map array (e.g. house: drone at z=10 cm, map
    // at z=150–460 cm due to height_offset=150), move it close to the map boundary before
    // the first scan sweep so the lidar can reach the map.
    // For sc1/sc2/sc3/benchmark the drone always starts inside the map → bootstrap_done_
    // is set true on the very first call with zero overhead.
    //
    // NOTE: the house map's bottom 15 voxel layers are solid ground (value≠0 = Occupied).
    // The drone CANNOT enter the map; it must scan from just below the boundary.
    // navigateToMap therefore targets min_height − 0.5 cm (just outside the array), and
    // once the drone is within one voxel width of the boundary we exit bootstrap and let
    // the normal scan sweep fire.
    if (!bootstrap_done_) {
        if (stateOf(worldToVoxel(state.position)) == types::VoxelOccupancy::OutOfBounds) {
            const double z     = state.position.z.force_numerical_value_in(cm);
            const double min_z = map_cfg_.boundaries.min_height.force_numerical_value_in(cm);
            const double max_z = map_cfg_.boundaries.max_height.force_numerical_value_in(cm);
            const double x     = state.position.x.force_numerical_value_in(cm);
            const double min_x = map_cfg_.boundaries.min_x.force_numerical_value_in(cm);
            const double max_x = map_cfg_.boundaries.max_x.force_numerical_value_in(cm);
            const double y     = state.position.y.force_numerical_value_in(cm);
            const double min_y = map_cfg_.boundaries.min_y.force_numerical_value_in(cm);
            const double max_y = map_cfg_.boundaries.max_y.force_numerical_value_in(cm);
            // "Close enough" = within one voxel width of every boundary dimension.
            const bool near_map =
                z >= min_z - res_cm_ && z <= max_z + res_cm_ &&
                x >= min_x - res_cm_ && x <= max_x + res_cm_ &&
                y >= min_y - res_cm_ && y <= max_y + res_cm_;
            if (near_map) {
                bootstrap_done_ = true;   // reached the boundary — hand off to sweep
            } else {
                return navigateToMap(state);
            }
        } else {
            bootstrap_done_ = true;
        }
    }
    // [END CHANGE: Fix 1]

    const Orientation scan_req{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]};

    // 1) Drain any pending commands.
    if (!pending_commands_.empty()) {
        auto cmd = pending_commands_.front();
        pending_commands_.pop_front();
        return {cmd, scan_req, types::AlgorithmStatus::Working};
    }

    const Cell cur = worldToVoxel(state.position);

    // 2) On arrival at a new waypoint: do a 360° yaw sweep so the harness
    //    can carve the map and reveal navigable space around us.
    if (needs_scan_) {
        needs_scan_ = false;

        const double rot_step = std::min(rot_step_deg_, toDeg(max_rotate_));
        const int    n_steps  = static_cast<int>(std::ceil(360.0 / rot_step));
        for (int i = 0; i < n_steps; ++i) {
            pending_commands_.push_back({
                types::MovementCommandType::Rotate,
                types::RotationDirection::Right,
                rot_step * horizontal_angle[deg],
                0.0 * cm,
            });
        }

        auto cmd = pending_commands_.front();
        pending_commands_.pop_front();
        return {cmd, scan_req, types::AlgorithmStatus::Working};
    }

    // 3) Value-based frontier selection (Change 1/2) with retry (Change 3).
    Cell frontier = selectBestFrontier(cur);
    if (frontier.x == std::numeric_limits<int>::min()) {
        // First failure: the recent_targets_ guard may have hidden the last frontier.
        // Clear it and retry once before declaring done (Change 3).
        recent_targets_.clear();
        frontier = selectBestFrontier(cur);
        if (frontier.x == std::numeric_limits<int>::min()) {
            done_ = true;
            return {std::nullopt, std::nullopt,
                    types::AlgorithmStatus::FinishedWithUnmappableVoxels};
        }
    }

    // Track attempts per frontier.  Permanently-shadowed cells accumulate count >= 5
    // and are excluded from future selections, guaranteeing eventual termination.
    ++frontier_try_count_[frontier];

    // Push chosen target into the anti-oscillation guard (Change 2).
    recent_targets_.push_back(frontier);
    if (recent_targets_.size() > 8) recent_targets_.pop_front();

    double heading_deg = toDeg(state.heading.horizontal);
    enqueueNavigationTo(cur, frontier, heading_deg);
    needs_scan_ = true;

    if (pending_commands_.empty()) {
        // The frontier has a horizontal Unmapped face-neighbour blocking clearForBody;
        // the path stopped at the current cell.  Scan from here to reveal that neighbour.
        ++stuck_scan_count_;
        if (stuck_scan_count_ > 10) {
            // Still stuck after 10 consecutive scans with no movement: the remaining
            // frontiers are unreachable from any accessible position — declare done.
            done_ = true;
            return {std::nullopt, std::nullopt,
                    types::AlgorithmStatus::FinishedWithUnmappableVoxels};
        }
        needs_scan_ = false;
        const double rot_step = std::min(rot_step_deg_, toDeg(max_rotate_));
        const int    n_steps  = static_cast<int>(std::ceil(360.0 / rot_step));
        for (int i = 0; i < n_steps; ++i) {
            pending_commands_.push_back({
                types::MovementCommandType::Rotate,
                types::RotationDirection::Right,
                rot_step * horizontal_angle[deg],
                0.0 * cm,
            });
        }
    } else {
        stuck_scan_count_ = 0;  // moved successfully — no longer stuck
    }

    auto cmd = pending_commands_.front();
    pending_commands_.pop_front();
    return {cmd, scan_req, types::AlgorithmStatus::Working};
}

} // namespace drone_mapper
