#include "drone/MappingAlgorithm.h"
#include "drone/Drone.h"
#include <mp-units/systems/si/math.h>
#include <cmath>
#include <algorithm>
#include <functional>
#include <limits>
#include <map>

namespace drone {

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
MappingAlgorithm::MappingAlgorithm(Drone& drone, const DroneConfig* config,
                                   const MissionConfig* mission)
    : m_drone(drone), m_config(config), m_mission(mission) {
    if (mission) {
        m_minHeight = static_cast<int>(
            mission->minHeight.numerical_value_in(cm));
        m_maxHeight = static_cast<int>(
            mission->maxHeight.numerical_value_in(cm));

        // Initialize at start height
        Position3D startPos = m_drone.GetLocation();
        m_currentHeightLevel = static_cast<int>(
            startPos.z.numerical_value_in(cm));
    }

    if (mission) {
        m_resolution = mission->outputResXYCm;   // 1.0 cm default
    }

    // BFS step: one drone-advance-length in grid units.
    // This decouples the storage quantisation (m_resolution) from the
    // navigation step so that the drone physically moves each BFS iteration
    // regardless of how finely the map is stored.
    if (config) {
        const double advanceCm =
            config->maxAdvance.numerical_value_in(cm);
        m_bfsStep = std::max(1,
            static_cast<int>(std::ceil(advanceCm / m_resolution)));
    } else {
        m_bfsStep = static_cast<int>(std::ceil(50.0 / m_resolution));
    }
}

// ---------------------------------------------------------------------------
// Main algorithm entry point
// ---------------------------------------------------------------------------
void MappingAlgorithm::Run() {
    const double elevStep = m_config
        ? m_config->maxElevate.numerical_value_in(cm)
        : 30.0;

    // 1. Full scan at the starting position.
    ScanAndUpdate();

    // 2. Explore XY at the starting height slice.
    ExploreAtCurrentHeight();

    // 3. Descend from start height down to minHeight.
    while (true) {
        Position3D pos = m_drone.GetLocation();
        double curH = pos.z.numerical_value_in(cm);

        double nextH = curH - elevStep;
        if (nextH < static_cast<double>(m_minHeight) - 1.0) break;

        m_currentHeightLevel = static_cast<int>(nextH);
        if (!AdjustHeight(nextH * cm)) break;

        ScanAndUpdate();
        ExploreAtCurrentHeight();
    }

    // 4. Climb from current position (near minHeight) all the way up to maxHeight.
    while (true) {
        Position3D pos = m_drone.GetLocation();
        double curH = pos.z.numerical_value_in(cm);

        double nextH = curH + elevStep;
        if (nextH > static_cast<double>(m_maxHeight) + 1.0) break;

        m_currentHeightLevel = static_cast<int>(nextH);
        if (!AdjustHeight(nextH * cm)) break;

        ScanAndUpdate();
        ExploreAtCurrentHeight();
    }
}

// ---------------------------------------------------------------------------
// Scan and update: fire lidar at current heading + three 90° rotations so
// every BFS position covers a full 360° sweep.
// ---------------------------------------------------------------------------
void MappingAlgorithm::ScanAndUpdate() {
    // Eight-direction scans (0°, 45°, 90°, ... 315° relative to current heading).
    // We rotate the drone in place and call the single-direction helper each
    // time, ending back at the original heading.
    const double maxRotateDeg = m_config
        ? m_config->maxRotate.numerical_value_in(deg)
        : 45.0;

    for (int slice = 0; slice < 32; ++slice) {
        ScanSingleDirection();
        // Rotate 11.25° (in max-rotate-sized steps)
        double remaining = 11.25;
        while (remaining > 0.5) {
            double step = std::min(remaining, maxRotateDeg);
            m_drone.Rotate(step * deg);
            m_currentHeading += step;
            if (m_currentHeading >= 360.0) m_currentHeading -= 360.0;
            remaining -= step;
        }
    }
}

void MappingAlgorithm::ScanSingleDirection() {
    const ScanResults hits    = m_drone.Scan();
    const Position3D  dronePos = m_drone.GetLocation();

    const double droneX = dronePos.x.numerical_value_in(cm);
    const double droneY = dronePos.y.numerical_value_in(cm);
    const double droneZ = dronePos.z.numerical_value_in(cm);

    for (const auto& hit : hits) {
        const double d = hit.distance.numerical_value_in(cm);
        if (d <= 0.0) continue;

        // hit.angle is relative to drone heading — add m_currentHeading for world angle
        const HorizontalAngle absH = hit.angle.horizontal + m_currentHeading * deg;
        const double dirX = si::cos(hit.angle.altitude).numerical_value_in(mp::one)
                          * si::cos(absH).numerical_value_in(mp::one);
        const double dirY = si::cos(hit.angle.altitude).numerical_value_in(mp::one)
                          * si::sin(absH).numerical_value_in(mp::one);
        const double dirZ = si::sin(hit.angle.altitude).numerical_value_in(mp::one);

        // Record hit voxel as Occupied
        m_drone.RecordCell(
            (droneX + d * dirX) * x_extent[cm],
            (droneY + d * dirY) * y_extent[cm],
            (droneZ + d * dirZ) * z_extent[cm],
            MapValue::Occupied);

        // Mark empty cells along the ray every 10 cm
        for (double t = 10.0; t < d - 1.0; t += 10.0)
            m_drone.RecordCell(
                (droneX + t * dirX) * x_extent[cm],
                (droneY + t * dirY) * y_extent[cm],
                (droneZ + t * dirZ) * z_extent[cm],
                MapValue::Empty);
    }
}


// ---------------------------------------------------------------------------
// BFS frontier exploration at current height
// ---------------------------------------------------------------------------
void MappingAlgorithm::ExploreAtCurrentHeight() {
    Position3D currentPos = m_drone.GetLocation();
    GridCell3D currentCell = WorldToGrid(currentPos.x, currentPos.y, currentPos.z);

    m_visited.insert(currentCell);

    // Push unvisited walkable neighbors of `cell` onto the frontier,
    // skipping any whose passage is too narrow for the drone.
    const int S = m_bfsStep;
    auto expand = [&](const GridCell3D& cell) {
        for (const GridCell3D& nb : std::vector<GridCell3D>{
                {cell.x + S, cell.y,     cell.z},
                {cell.x - S, cell.y,     cell.z},
                {cell.x,     cell.y + S, cell.z},
                {cell.x,     cell.y - S, cell.z}})
            if (!m_visited.count(nb) && IsWalkable(nb) && HasClearance(cell, nb))
                m_frontier.push(nb);
    };

    expand(currentCell);

    while (!m_frontier.empty()) {
        GridCell3D target = m_frontier.front();
        m_frontier.pop();

        if (m_visited.count(target)) continue;

        // Use A* to find a path through known-empty space to the target
        std::vector<GridCell3D> path = FindPath(target);
        if (path.empty()) continue;  // currently unreachable — skip

        // Walk every step in the path (index 0 is current position, already visited)
        for (std::size_t i = 1; i < path.size(); ++i) {
            if (!MoveToCell(path[i])) return;  // collision — stop exploration
            m_visited.insert(path[i]);
            ScanAndUpdate();
            expand(path[i]);
        }
    }
}

// ---------------------------------------------------------------------------
// Height adjustment
// ---------------------------------------------------------------------------
bool MappingAlgorithm::ElevateToNextHeight() {
    m_currentHeightLevel += 30;
    return AdjustHeight(m_currentHeightLevel * cm);
}

// ---------------------------------------------------------------------------
// Helper: world to grid conversion
// ---------------------------------------------------------------------------
GridCell3D MappingAlgorithm::WorldToGrid(XLength x, YLength y, ZLength z) const {
    int gridX = static_cast<int>(x.numerical_value_in(cm) / m_resolution);
    int gridY = static_cast<int>(y.numerical_value_in(cm) / m_resolution);
    int gridZ = static_cast<int>(z.numerical_value_in(cm) / m_resolution);
    return {gridX, gridY, gridZ};
}

// ---------------------------------------------------------------------------
// Helper: grid to world conversion
// ---------------------------------------------------------------------------
Position3D MappingAlgorithm::GridToWorld(const GridCell3D& cell) const {
    return {
        cell.x * m_resolution * x_extent[cm],
        cell.y * m_resolution * y_extent[cm],
        cell.z * m_resolution * z_extent[cm]
    };
}

// ---------------------------------------------------------------------------
// Helper: check if cell is walkable (empty or unmapped)
// ---------------------------------------------------------------------------
bool MappingAlgorithm::IsWalkable(const GridCell3D& cell) const {
    Position3D wp = GridToWorld(cell);

    MapValue val = m_drone.QueryCell(wp.x, wp.y, wp.z);
    return val != MapValue::Occupied && val != MapValue::BeyondBounds;
}

// ---------------------------------------------------------------------------
// Helper: check the passage from `from` to `to` fits the drone's minimum
// pass dimensions (width and height).  Only uses cells already recorded in
// the drone's map — unknown cells are assumed passable.
// ---------------------------------------------------------------------------
bool MappingAlgorithm::HasClearance(const GridCell3D& from,
                                    const GridCell3D& to) const {
    if (!m_config) return true;

    const double halfW = m_config->minPassWidth.numerical_value_in(
                             cm) / 2.0;
    const double halfH = m_config->minPassHeight.numerical_value_in(
                             cm) / 2.0;

    Position3D wp = GridToWorld(to);
    const double tx = wp.x.numerical_value_in(cm);
    const double ty = wp.y.numerical_value_in(cm);
    const double tz = wp.z.numerical_value_in(cm);

    // Determine lateral axis: movement in X → lateral is Y; movement in Y → lateral is X
    const bool movingInX = (to.x != from.x);

    // Four boundary points of the minimum-pass cross-section at the target cell
    const double latOff1 = movingInX ? ty - halfW : tx - halfW;
    const double latOff2 = movingInX ? ty + halfW : tx + halfW;

    auto occupied = [&](double cx, double cy, double cz) {
        return m_drone.QueryCell(cx * x_extent[cm],
                                 cy * y_extent[cm],
                                 cz * z_extent[cm]) == MapValue::Occupied;
    };

    if (movingInX) {
        return !occupied(tx, latOff1, tz)        // left edge
            && !occupied(tx, latOff2, tz)        // right edge
            && !occupied(tx, ty,      tz - halfH) // bottom edge
            && !occupied(tx, ty,      tz + halfH); // top edge
    } else {
        return !occupied(latOff1, ty, tz)        // left edge
            && !occupied(latOff2, ty, tz)        // right edge
            && !occupied(tx,      ty, tz - halfH) // bottom edge
            && !occupied(tx,      ty, tz + halfH); // top edge
    }
}

// ---------------------------------------------------------------------------
// A* pathfinding — finds shortest path through known-empty cells
// ---------------------------------------------------------------------------
std::vector<GridCell3D> MappingAlgorithm::FindPath(const GridCell3D& target) {
    Position3D currentPos = m_drone.GetLocation();
    GridCell3D start = WorldToGrid(currentPos.x, currentPos.y, currentPos.z);

    if (start == target) return {start};

    // Manhattan distance heuristic (admissible for uniform-cost grid)
    auto heuristic = [](const GridCell3D& a, const GridCell3D& b) {
        return std::abs(a.x - b.x) + std::abs(a.y - b.y) + std::abs(a.z - b.z);
    };

    // Min-heap ordered by f = g + h
    using Entry = std::pair<int, GridCell3D>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> openSet;

    std::map<GridCell3D, int>        gScore;
    std::map<GridCell3D, GridCell3D> parent;
    std::set<GridCell3D>             closed;

    gScore[start] = 0;
    parent[start] = start;
    openSet.push({heuristic(start, target), start});

    while (!openSet.empty()) {
        auto [f, current] = openSet.top();
        openSet.pop();

        if (closed.count(current)) continue;
        closed.insert(current);

        if (current == target) {
            // Reconstruct path from target back to start
            std::vector<GridCell3D> path;
            for (GridCell3D node = target; !(node == start); node = parent[node])
                path.push_back(node);
            path.push_back(start);
            std::reverse(path.begin(), path.end());
            return path;
        }

        for (const GridCell3D& nb : std::vector<GridCell3D>{
                {current.x + m_bfsStep, current.y,           current.z},
                {current.x - m_bfsStep, current.y,           current.z},
                {current.x,             current.y + m_bfsStep, current.z},
                {current.x,             current.y - m_bfsStep, current.z}}) {
            if (closed.count(nb) || !IsWalkable(nb)) continue;

            int tentativeG = gScore[current] + m_bfsStep;
            if (!gScore.count(nb) || tentativeG < gScore[nb]) {
                gScore[nb] = tentativeG;
                parent[nb] = current;
                openSet.push({tentativeG + heuristic(nb, target), nb});
            }
        }
    }

    return {};  // no path found
}

// ---------------------------------------------------------------------------
// Movement: navigate to a target cell
// ---------------------------------------------------------------------------
bool MappingAlgorithm::MoveToCell(const GridCell3D& target) {
    Position3D wp = GridToWorld(target);

    if (!AdjustHeight(wp.z.numerical_value_in(cm) * cm)) {
        return false;
    }

    const double maxAdvanceCm = m_config
        ? m_config->maxAdvance.numerical_value_in(cm) : 50.0;

    for (;;) {
        // Re-rotate each iteration so we face target even after overshoot
        if (!RotateToFace(wp.x, wp.y)) return false;

        Position3D cur = m_drone.GetLocation();
        const double dx     = wp.x.numerical_value_in(cm) - cur.x.numerical_value_in(cm);
        const double dy     = wp.y.numerical_value_in(cm) - cur.y.numerical_value_in(cm);
        const double distCm = std::sqrt(dx * dx + dy * dy);

        if (distCm < 0.1) break;

        // Advance at most maxAdvance, but never more than remaining distance
        const double step = std::min(maxAdvanceCm, distCm);
        if (m_drone.Advance(step * cm) == MoveResult::CollisionDetected) {
            return false;
        }
    }

    return true;
}

// ---------------------------------------------------------------------------
// Rotation helper
// ---------------------------------------------------------------------------
bool MappingAlgorithm::RotateToFace(XLength targetX, YLength targetY) {
    Position3D currentPos = m_drone.GetLocation();

    double dx = targetX.numerical_value_in(cm) -
                currentPos.x.numerical_value_in(cm);
    double dy = targetY.numerical_value_in(cm) -
                currentPos.y.numerical_value_in(cm);

    // Desired heading (0 = +X, 90 = +Y)
    double desiredHeading = HorizontalAngle{si::atan2(dy * cm, dx * cm)}.numerical_value_in(deg);
    if (desiredHeading < 0) desiredHeading += 360.0;

    double maxRotateDeg = m_config
        ? m_config->maxRotate.numerical_value_in(deg)
        : 45.0;

    double angleDiff = desiredHeading - m_currentHeading;

    // Normalize to [-180, 180]
    if (angleDiff > 180.0) angleDiff -= 360.0;
    if (angleDiff < -180.0) angleDiff += 360.0;

    // Step in maxRotate-sized chunks so m_currentHeading stays in sync with
    // the driver's actual heading (which clamps each call to maxRotate).
    while (std::abs(angleDiff) > 1.0) {
        double step = angleDiff > 0
            ? std::min(angleDiff,  maxRotateDeg)
            : std::max(angleDiff, -maxRotateDeg);

        if (m_drone.Rotate(step *deg) == MoveResult::CollisionDetected) {
            return false;
        }
        m_currentHeading += step;
        if (m_currentHeading >= 360.0) m_currentHeading -= 360.0;
        if (m_currentHeading <    0.0) m_currentHeading += 360.0;

        angleDiff = desiredHeading - m_currentHeading;
        if (angleDiff > 180.0) angleDiff -= 360.0;
        if (angleDiff < -180.0) angleDiff += 360.0;
    }

    return true;
}

// ---------------------------------------------------------------------------
// Height adjustment helper
// ---------------------------------------------------------------------------
bool MappingAlgorithm::AdjustHeight(PhysicalLength targetHeight) {
    const double maxElevate = m_config
        ? m_config->maxElevate.numerical_value_in(cm)
        : 30.0;

    while (true) {
        Position3D currentPos = m_drone.GetLocation();
        double currentH = currentPos.z.numerical_value_in(cm);
        double targetH  = targetHeight.numerical_value_in(cm);
        double diff     = targetH - currentH;

        if (std::abs(diff) < 0.1) break;  // close enough

        double step = (diff > 0)
            ? std::min(diff,  maxElevate)
            : std::max(diff, -maxElevate);

        if (m_drone.Elevate(step * cm) ==
            MoveResult::CollisionDetected) {
            return false;
        }
    }
    return true;
}

}  // namespace drone
