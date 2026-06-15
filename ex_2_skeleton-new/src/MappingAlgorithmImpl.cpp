#include <drone_mapper/MappingAlgorithmImpl.h>
#include <drone_mapper/IMap3D.h>
#include <drone_mapper/ScanResultToVoxels.h>

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

void MappingAlgorithmImpl::initialize(const types::DroneState& state) {
    nav_step_cm_    = _drone_config.max_advance.force_numerical_value_in(cm);
    max_rotate_deg_ = _drone_config.max_rotate.force_numerical_value_in(deg);
    max_elevate_cm_ = _drone_config.max_elevate.force_numerical_value_in(cm);

    const auto map_cfg = _output_map.getMapConfig();
    min_x_cm_      = map_cfg.boundaries.min_x.force_numerical_value_in(cm);
    max_x_cm_      = map_cfg.boundaries.max_x.force_numerical_value_in(cm);
    min_y_cm_      = map_cfg.boundaries.min_y.force_numerical_value_in(cm);
    max_y_cm_      = map_cfg.boundaries.max_y.force_numerical_value_in(cm);
    min_height_cm_ = map_cfg.boundaries.min_height.force_numerical_value_in(cm);
    max_height_cm_ = map_cfg.boundaries.max_height.force_numerical_value_in(cm);

    const double start_z = state.position.z.force_numerical_value_in(cm);
    const double min_flyable = min_height_cm_ + max_elevate_cm_;
    const double max_flyable = max_height_cm_ - max_elevate_cm_;

    for (double h = start_z; h >= min_flyable - 0.5; h -= max_elevate_cm_)
        height_levels_cm_.push_back(h);
    for (double h = start_z + max_elevate_cm_; h <= max_flyable + 0.5; h += max_elevate_cm_)
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
    const double step = (nav_step_cm_ > 0.0) ? nav_step_cm_ : 1.0;
    return {
        static_cast<int>(std::floor(pos.x.force_numerical_value_in(cm) / step)),
        static_cast<int>(std::floor(pos.y.force_numerical_value_in(cm) / step)),
    };
}

Position3D MappingAlgorithmImpl::gridToWorld(const GridCell2D& cell,
                                              double height_cm) const {
    return {
        cell.x * nav_step_cm_ * x_extent[cm],
        cell.y * nav_step_cm_ * y_extent[cm],
        height_cm              * z_extent[cm],
    };
}

bool MappingAlgorithmImpl::isInBounds(const GridCell2D& cell) const {
    const double wx = cell.x * nav_step_cm_;
    const double wy = cell.y * nav_step_cm_;
    return wx >= min_x_cm_ && wx <= max_x_cm_
        && wy >= min_y_cm_ && wy <= max_y_cm_;
}

bool MappingAlgorithmImpl::isWalkable(const GridCell2D& cell, int level_idx) const {
    if (!isInBounds(cell)) return false;
    const auto key = std::make_pair(cell, level_idx);
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
        const Cell3D c3d{nb, level_idx};
        if (!visited_3d_.count(c3d) && !frontier_3d_.count(c3d)
            && isWalkable(nb, level_idx)
            && hasClearance(cell, nb, level_idx, height_cm))
            frontier_3d_.insert(c3d);
    }

    for (int delta : {-1, +1}) {
        const int lvl = level_idx + delta;
        if (lvl < 0 || lvl >= static_cast<int>(height_levels_cm_.size())) continue;
        const auto key = std::make_pair(cell, lvl);
        if (obstacle_cache_.count(key) && obstacle_cache_.at(key)) continue;
        const Cell3D c3d{cell, lvl};
        if (!visited_3d_.count(c3d) && !frontier_3d_.count(c3d))
            frontier_3d_.insert(c3d);
    }
}

MappingAlgorithmImpl::Cell3D
MappingAlgorithmImpl::findNearest3DFrontier(const GridCell2D& from,
                                              int from_level) const {
    Cell3D best{{std::numeric_limits<int>::min(), 0}, 0};
    int best_dist = std::numeric_limits<int>::max();

    for (const auto& c3d : frontier_3d_) {
        const int d = std::abs(c3d.first.x - from.x)
                    + std::abs(c3d.first.y - from.y)
                    + std::abs(c3d.second  - from_level);
        if (d < best_dist) { best_dist = d; best = c3d; }
    }
    return best;
}

bool MappingAlgorithmImpl::hasClearance(const GridCell2D& from, const GridCell2D& to,
                                         int level_idx, double height_cm) const {
    // radius is the actual sphere radius now (not half of dimensions)
    const double radius_cm = _drone_config.radius.force_numerical_value_in(cm);

    const double tx = to.x * nav_step_cm_;
    const double ty = to.y * nav_step_cm_;

    auto wall_at_xy = [&](double wx, double wy) -> bool {
        const int ix = static_cast<int>(std::round(wx));
        const int iy = static_cast<int>(std::round(wy));
        return fine_wall_xy_.count({ix, iy}) > 0;
    };

    const bool moving_in_x = (to.x != from.x);

    auto occupied = [&](double wx, double wy, double wz) {
        const Position3D pos{wx * x_extent[cm], wy * y_extent[cm], wz * z_extent[cm]};
        const GridCell2D cell = worldToGrid(pos);
        const auto key = std::make_pair(cell, level_idx);
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

    while (remaining > 0.5) {
        const double step = std::min(remaining, max_rotate_deg_);
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

        const double dx = static_cast<double>(next.x - prev.x) * nav_step_cm_;
        const double dy = static_cast<double>(next.y - prev.y) * nav_step_cm_;
        double desired = std::atan2(dy, dx) * 180.0 / std::numbers::pi;
        if (desired < 0.0) desired += 360.0;

        enqueueRotateToAngle(desired, sim_heading_deg);

        double dist = std::sqrt(dx * dx + dy * dy);
        while (dist > 0.1) {
            const double step = std::min(dist, nav_step_cm_);
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
            if (std::abs(vz - level_z) <= max_elevate_cm_ + 0.5) {
                obstacle_cache_[{cell, lvl}] = true;
                frontier_3d_.erase({cell, lvl});
            }
        }
    }
}

types::MappingStepCommand MappingAlgorithmImpl::nextStep(
    const types::DroneState& state,
    const types::LidarScanResult* latest_scan) {

    if (!initialized_) initialize(state);
    if (done_) return {std::nullopt, std::nullopt, types::AlgorithmStatus::Finished};

    // Process incoming scan result
    if (latest_scan != nullptr) {
        const auto voxels = ScanResultToVoxels::convert(
            state.position, state.heading, *latest_scan);
        applyFiltered(voxels);
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
        visited_3d_.insert({cur_cell, cur_level});
        expandFrontier(cur_cell, cur_level, cur_height);

        const double rot_step = std::min(11.25, max_rotate_deg_);
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

    if (target.first.x == std::numeric_limits<int>::min()) {
        done_ = true;
        return {std::nullopt, std::nullopt, types::AlgorithmStatus::Finished};
    }

    frontier_3d_.erase(target);
    visited_3d_.insert(target);  // prevent re-adding if drone stalls at current cell
    const auto [target_cell, target_level] = target;

    double sim_heading = state.heading.horizontal.force_numerical_value_in(deg);
    enqueueNavigationTo(cur_cell, target_cell, cur_level, cur_height, sim_heading);

    if (target_level != cur_level) {
        const double target_h = height_levels_cm_[target_level];
        double diff = target_h - cur_height;
        while (std::abs(diff) > 0.1) {
            const double step = (diff > 0.0)
                ? std::min( diff,  max_elevate_cm_)
                : std::max( diff, -max_elevate_cm_);
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
