#include <drone_mapper/MappingAlgorithmImpl.h>
#include <drone_mapper/IMap3D.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <map>
#include <numbers>
#include <queue>
#include <set>
#include <utility>

namespace drone_mapper {

namespace {
constexpr double toCm(PhysicalLength l)  { return l.force_numerical_value_in(cm); }
constexpr double toXcm(XLength l)        { return l.force_numerical_value_in(cm); }
constexpr double toYcm(YLength l)        { return l.force_numerical_value_in(cm); }
constexpr double toZcm(ZLength l)        { return l.force_numerical_value_in(cm); }
constexpr double toDeg(HorizontalAngle a){ return a.force_numerical_value_in(deg); }
} // namespace

void MappingAlgorithmImpl::initialize(const types::DroneState& state) {
    nav_step_    = drone_config_.max_advance;
    max_rotate_  = drone_config_.max_rotate;
    max_elevate_ = drone_config_.max_elevate;

    const auto map_cfg = output_map_.getMapConfig();
    min_x_      = map_cfg.boundaries.min_x;
    max_x_      = map_cfg.boundaries.max_x;
    min_y_      = map_cfg.boundaries.min_y;
    max_y_      = map_cfg.boundaries.max_y;
    min_height_ = map_cfg.boundaries.min_height;
    max_height_ = map_cfg.boundaries.max_height;

    const double start_z     = state.position.z.force_numerical_value_in(cm);
    const double max_elev_cm = toCm(max_elevate_);
    const double min_flyable = toZcm(min_height_) + max_elev_cm;
    const double max_flyable = toZcm(max_height_) - max_elev_cm;

    for (double h = start_z; h >= min_flyable - 0.5; h -= max_elev_cm)
        height_levels_cm_.push_back(h);
    for (double h = start_z + max_elev_cm; h <= max_flyable + 0.5; h += max_elev_cm)
        height_levels_cm_.push_back(h);

    if (height_levels_cm_.empty())
        height_levels_cm_.push_back(start_z);

    height_level_index_ = 0;
    pending_level_idx_  = -1;
    initialized_ = true;
    needs_scan_  = true;
}

MappingAlgorithmImpl::GridCell2D
MappingAlgorithmImpl::worldToGrid(const Position3D& pos) const {
    const double step = toCm(nav_step_) > 0.0 ? toCm(nav_step_) : 1.0;
    return {
        static_cast<int>(std::floor(pos.x.force_numerical_value_in(cm) / step)),
        static_cast<int>(std::floor(pos.y.force_numerical_value_in(cm) / step)),
    };
}

Position3D MappingAlgorithmImpl::gridToWorld(const GridCell2D& cell,
                                              double height_cm) const {
    const double step_cm = toCm(nav_step_);
    return {
        cell.x * step_cm * x_extent[cm],
        cell.y * step_cm * y_extent[cm],
        height_cm         * z_extent[cm],
    };
}

bool MappingAlgorithmImpl::isInBounds(const GridCell2D& cell) const {
    const double step_cm = toCm(nav_step_);
    const double wx = cell.x * step_cm;
    const double wy = cell.y * step_cm;
    return wx >= toXcm(min_x_) && wx <= toXcm(max_x_)
        && wy >= toYcm(min_y_) && wy <= toYcm(max_y_);
}

bool MappingAlgorithmImpl::isWalkable(const GridCell2D& cell, int level_idx) const {
    if (!isInBounds(cell)) return false;
    const Cell3D key{cell.x, cell.y, level_idx};
    const auto it = obstacle_cache_.find(key);
    if (it != obstacle_cache_.end()) return !it->second;
    return true;
}

void MappingAlgorithmImpl::expandFrontier(const GridCell2D& cell,
                                           int level_idx, double height_cm) {
    const std::array<GridCell2D, 4> neighbors = {{
        {cell.x + 1, cell.y},
        {cell.x - 1, cell.y},
        {cell.x,     cell.y + 1},
        {cell.x,     cell.y - 1},
    }};
    for (const auto& nb : neighbors) {
        const Cell3D c3d{nb.x, nb.y, level_idx};
        if (!visited_3d_.count(c3d) && !frontier_3d_.count(c3d)
            && isWalkable(nb, level_idx)
            && hasClearance(cell, nb, level_idx, height_cm))
            frontier_3d_.insert(c3d);
    }

    for (int delta : {-1, +1}) {
        const int lvl = level_idx + delta;
        if (lvl < 0 || lvl >= static_cast<int>(height_levels_cm_.size())) continue;
        const Cell3D key{cell.x, cell.y, lvl};
        if (obstacle_cache_.count(key) && obstacle_cache_.at(key)) continue;
        const Cell3D c3d{cell.x, cell.y, lvl};
        if (!visited_3d_.count(c3d) && !frontier_3d_.count(c3d))
            frontier_3d_.insert(c3d);
    }
}

MappingAlgorithmImpl::Cell3D
MappingAlgorithmImpl::findNearest3DFrontier(const GridCell2D& from,
                                              int from_level) const {
    Cell3D best{std::numeric_limits<int>::min(), 0, 0};
    int best_dist = std::numeric_limits<int>::max();

    for (const auto& c3d : frontier_3d_) {
        const int d = std::abs(c3d.x - from.x)
                    + std::abs(c3d.y - from.y)
                    + std::abs(c3d.z - from_level);
        if (d < best_dist) { best_dist = d; best = c3d; }
    }
    return best;
}

bool MappingAlgorithmImpl::hasClearance(const GridCell2D& from, const GridCell2D& to,
                                         int level_idx, double height_cm) const {
    const double radius_cm = toCm(drone_config_.radius);
    const double step_cm   = toCm(nav_step_);
    const double tx = to.x * step_cm;
    const double ty = to.y * step_cm;

    auto wall_at_xy = [&](double wx, double wy) -> bool {
        const int ix = static_cast<int>(std::round(wx));
        const int iy = static_cast<int>(std::round(wy));
        return fine_wall_xy_.count({ix, iy}) > 0;
    };

    const bool moving_in_x = (to.x != from.x);

    auto occupied = [&](double wx, double wy, double wz) {
        const Position3D pos{wx * x_extent[cm], wy * y_extent[cm], wz * z_extent[cm]};
        const GridCell2D cell = worldToGrid(pos);
        const Cell3D key{cell.x, cell.y, level_idx};
        const auto it = obstacle_cache_.find(key);
        return it != obstacle_cache_.end() && it->second;
    };

    if (moving_in_x) {
        return !wall_at_xy(tx, ty - radius_cm)
            && !wall_at_xy(tx, ty + radius_cm)
            && !occupied(tx, ty, height_cm - radius_cm)
            && !occupied(tx, ty, height_cm + radius_cm);
    } else {
        return !wall_at_xy(tx - radius_cm, ty)
            && !wall_at_xy(tx + radius_cm, ty)
            && !occupied(tx, ty, height_cm - radius_cm)
            && !occupied(tx, ty, height_cm + radius_cm);
    }
}

std::vector<MappingAlgorithmImpl::GridCell2D>
MappingAlgorithmImpl::findPath(const GridCell2D& from, const GridCell2D& to,
                                int level_idx, double /*height_cm*/) {
    if (from == to) return {from};

    auto heuristic = [](const GridCell2D& a, const GridCell2D& b) {
        return std::abs(a.x - b.x) + std::abs(a.y - b.y);
    };

    using Entry = std::pair<int, GridCell2D>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<>> open;
    std::map<GridCell2D, int>        g_score;
    std::map<GridCell2D, GridCell2D> parent;
    std::set<GridCell2D>             closed;

    g_score[from] = 0;
    parent[from]  = from;
    open.push({heuristic(from, to), from});

    while (!open.empty()) {
        auto [f, cur] = open.top(); open.pop();
        if (closed.count(cur)) continue;
        closed.insert(cur);

        if (cur == to) {
            std::vector<GridCell2D> path;
            for (GridCell2D n = to; !(n == from); n = parent[n])
                path.push_back(n);
            path.push_back(from);
            std::reverse(path.begin(), path.end());
            return path;
        }

        const std::array<GridCell2D, 4> neighbors = {{
            {cur.x + 1, cur.y}, {cur.x - 1, cur.y},
            {cur.x, cur.y + 1}, {cur.x, cur.y - 1},
        }};
        for (const auto& nb : neighbors) {
            if (closed.count(nb) || !isWalkable(nb, level_idx)) continue;
            const int tg = g_score[cur] + 1;
            if (!g_score.count(nb) || tg < g_score[nb]) {
                g_score[nb] = tg;
                parent[nb]  = cur;
                open.push({tg + heuristic(nb, to), nb});
            }
        }
    }
    return {};
}

void MappingAlgorithmImpl::enqueueRotateToAngle(double target_deg, double& current_deg) {
    double diff = target_deg - current_deg;
    while (diff >  180.0) diff -= 360.0;
    while (diff < -180.0) diff += 360.0;

    const auto dir = (diff >= 0.0) ? types::RotationDirection::Right
                                   : types::RotationDirection::Left;
    double remaining = std::abs(diff);

    const double max_rot_deg = toDeg(max_rotate_);
    while (remaining > 0.5) {
        const double step = std::min(remaining, max_rot_deg);
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

void MappingAlgorithmImpl::enqueueNavigationTo(const GridCell2D& from,
                                                const GridCell2D& to,
                                                int level_idx,
                                                double height_cm,
                                                double& sim_heading_deg) {
    const auto path = findPath(from, to, level_idx, height_cm);
    if (path.size() < 2) return;

    for (std::size_t i = 1; i < path.size(); ++i) {
        const GridCell2D& prev = path[i - 1];
        const GridCell2D& next = path[i];

        const double step_cm = toCm(nav_step_);
        const double dx = static_cast<double>(next.x - prev.x) * step_cm;
        const double dy = static_cast<double>(next.y - prev.y) * step_cm;
        double desired = std::atan2(dy, dx) * 180.0 / std::numbers::pi;
        if (desired < 0.0) desired += 360.0;

        enqueueRotateToAngle(desired, sim_heading_deg);

        double dist = std::sqrt(dx * dx + dy * dy);
        while (dist > 0.1) {
            const double step = std::min(dist, step_cm);
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

void MappingAlgorithmImpl::applyFiltered(const std::vector<types::MappedVoxel>& voxels) {
    for (const auto& v : voxels) {
        if (v.value != types::VoxelOccupancy::Occupied) continue;

        const double vz = v.position.z.force_numerical_value_in(cm);
        const double vx = v.position.x.force_numerical_value_in(cm);
        const double vy = v.position.y.force_numerical_value_in(cm);
        const GridCell2D cell = worldToGrid(v.position);

        fine_wall_xy_.emplace(static_cast<int>(std::round(vx)),
                              static_cast<int>(std::round(vy)));

        for (int lvl = 0; lvl < static_cast<int>(height_levels_cm_.size()); ++lvl) {
            const double level_z = height_levels_cm_[lvl];
            if (std::abs(vz - level_z) <= toCm(max_elevate_) + 0.5) {
                obstacle_cache_[{cell.x, cell.y, lvl}] = true;
                frontier_3d_.erase({cell.x, cell.y, lvl});
            }
        }
    }
}

types::MappingStepCommand MappingAlgorithmImpl::nextStep(
    const types::DroneState& state,
    const types::LidarScanResult* latest_scan) {

    if (!initialized_) initialize(state);
    if (done_) return {std::nullopt, std::nullopt, types::AlgorithmStatus::Finished};

    // Process incoming scan result — extract occupied hit positions inline
    if (latest_scan != nullptr) {
        constexpr double miss_threshold = std::numeric_limits<double>::max() / 2.0;
        std::vector<types::MappedVoxel> hits;
        for (const auto& hit : *latest_scan) {
            const double d_cm = hit.distance.force_numerical_value_in(cm);
            if (d_cm <= 0.0 || d_cm > miss_threshold) continue;
            const double horiz_deg =
                (state.heading.horizontal + hit.angle.horizontal).force_numerical_value_in(deg);
            const double alt_deg =
                (state.heading.altitude + hit.angle.altitude).force_numerical_value_in(deg);
            const double horiz_rad = horiz_deg * std::numbers::pi / 180.0;
            const double alt_rad   = alt_deg   * std::numbers::pi / 180.0;
            const double ca = std::cos(alt_rad);
            const double ox = state.position.x.force_numerical_value_in(cm);
            const double oy = state.position.y.force_numerical_value_in(cm);
            const double oz = state.position.z.force_numerical_value_in(cm);
            hits.push_back({{
                (ox + ca * std::cos(horiz_rad) * d_cm) * x_extent[cm],
                (oy + ca * std::sin(horiz_rad) * d_cm) * y_extent[cm],
                (oz + std::sin(alt_rad) * d_cm)        * z_extent[cm],
            }, types::VoxelOccupancy::Occupied});
        }
        applyFiltered(hits);
    }

    // Drain pending commands (rotation scan or navigation)
    if (!pending_commands_.empty()) {
        const auto cmd = pending_commands_.front();
        pending_commands_.pop_front();
        // Always request a scan so the algorithm keeps receiving obstacle data
        const Orientation scan_req{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]};
        return {cmd, scan_req, types::AlgorithmStatus::Working};
    }

    const GridCell2D cur_cell   = worldToGrid(state.position);

    if (needs_scan_) {
        // Arrived at a new position: update level, expand frontier, queue 360° rotation scan
        if (pending_level_idx_ >= 0) {
            height_level_index_ = static_cast<std::size_t>(pending_level_idx_);
            pending_level_idx_  = -1;
        }

        const int    cur_level  = static_cast<int>(height_level_index_);
        const double cur_height = height_levels_cm_[cur_level];

        needs_scan_ = false;
        visited_3d_.insert({cur_cell.x, cur_cell.y, cur_level});
        expandFrontier(cur_cell, cur_level, cur_height);

        const double rot_step = std::min(11.25, toDeg(max_rotate_));
        const int    n_steps  = static_cast<int>(std::ceil(360.0 / rot_step));
        for (int i = 0; i < n_steps; ++i) {
            pending_commands_.push_back({
                types::MovementCommandType::Rotate,
                types::RotationDirection::Right,
                rot_step * horizontal_angle[deg],
                0.0 * cm,
            });
        }

        const auto cmd = pending_commands_.front();
        pending_commands_.pop_front();
        const Orientation scan_req{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]};
        return {cmd, scan_req, types::AlgorithmStatus::Working};
    }

    // Navigation phase: find nearest unvisited frontier
    const int    cur_level  = static_cast<int>(height_level_index_);
    const double cur_height = height_levels_cm_[cur_level];

    const Cell3D target = findNearest3DFrontier(cur_cell, cur_level);

    if (target.x == std::numeric_limits<int>::min()) {
        done_ = true;
        return {std::nullopt, std::nullopt, types::AlgorithmStatus::Finished};
    }

    frontier_3d_.erase(target);
    visited_3d_.insert(target);  // prevent re-adding if drone stalls at current cell
    const GridCell2D target_cell{target.x, target.y};
    const int        target_level = target.z;

    double sim_heading = state.heading.horizontal.force_numerical_value_in(deg);
    enqueueNavigationTo(cur_cell, target_cell, cur_level, cur_height, sim_heading);

    if (target_level != cur_level) {
        const double target_h    = height_levels_cm_[target_level];
        const double max_elev_cm = toCm(max_elevate_);
        double diff = target_h - cur_height;
        while (std::abs(diff) > 0.1) {
            const double step = (diff > 0.0)
                ? std::min( diff,  max_elev_cm)
                : std::max( diff, -max_elev_cm);
            pending_commands_.push_back({
                types::MovementCommandType::Elevate,
                types::RotationDirection::Left,
                0.0 * horizontal_angle[deg],
                step * isq::length[cm],
            });
            diff -= step;
        }
        pending_level_idx_ = target_level;
    }

    needs_scan_ = true;

    if (pending_commands_.empty()) {
        done_ = true;
        return {std::nullopt, std::nullopt, types::AlgorithmStatus::FinishedWithUnmappableVoxels};
    }

    const auto cmd = pending_commands_.front();
    pending_commands_.pop_front();
    const Orientation scan_req{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]};
    return {cmd, scan_req, types::AlgorithmStatus::Working};
}

} // namespace drone_mapper
