#pragma once

#include <queue>
#include <set>
#include <vector>
#include <utility>
#include <cmath>
#include "types/Units.h"
#include "types/MapValue.h"
#include "io/ConfigParser.h"

namespace drone {

class Drone;

// ---------------------------------------------------------------------------
// GridCell3D — voxel coordinates (grid indices, not world coordinates)
// ---------------------------------------------------------------------------
struct GridCell3D {
    int x, y, z;  // grid indices in cm

    bool operator==(const GridCell3D& other) const {
        return x == other.x && y == other.y && z == other.z;
    }

    bool operator<(const GridCell3D& other) const {
        if (x != other.x) return x < other.x;
        if (y != other.y) return y < other.y;
        return z < other.z;
    }
};

// ---------------------------------------------------------------------------
// MappingAlgorithm
//
// Autonomous 3D building mapping using:
// - Lidar scan-and-update: convert lidar hits to world voxels
// - BFS frontier: queue of unvisited cells
// - A* path planning: navigate to frontier cells
// - Height exploration: after XY frontier exhausted, move to next height level
//
// Returns when all reachable cells have been visited or collision occurs.
// ---------------------------------------------------------------------------
class MappingAlgorithm {
public:
    explicit MappingAlgorithm(Drone& drone, const DroneConfig* config = nullptr,
                             const MissionConfig* mission = nullptr);

    // Execute the mapping algorithm.
    // Returns normally if exploration completes successfully.
    // Does nothing and returns on first collision.
    void Run();

private:
    Drone& m_drone;
    const DroneConfig* m_config;
    const MissionConfig* m_mission;

    std::set<GridCell3D> m_visited;         // cells already explored
    std::queue<GridCell3D> m_frontier;      // cells to explore (BFS)
    int m_minHeight{0}, m_maxHeight{300};   // height bounds in cm
    int m_currentHeightLevel{150};          // cm
    double m_resolution{1.0};               // grid cell size in cm (storage quantisation)
    int    m_bfsStep{100};                  // BFS neighbour distance in grid units (~maxAdvance)
    double m_currentHeading{0.0};           // drone's current heading in degrees

    // -----------------------------------------------------------------------
    // Main algorithm phases
    // -----------------------------------------------------------------------

    // Scan lidar in all four cardinal directions and record all hit cells
    void ScanAndUpdate();
    // Single-direction scan (one lidar shot along current heading)
    void ScanSingleDirection();

    // BFS exploration of frontier cells
    void ExploreAtCurrentHeight();

    // Move to next unvisited height level
    bool ElevateToNextHeight();

    // -----------------------------------------------------------------------
    // Helper methods
    // -----------------------------------------------------------------------

    // Convert world position (cm) to grid indices
    GridCell3D WorldToGrid(XLength x, YLength y, ZLength z) const;

    // Convert grid indices back to world position (cm)
    Position3D GridToWorld(const GridCell3D& cell) const;

    // Check if a cell is occupied (visited or already mapped as occupied)
    bool IsWalkable(const GridCell3D& cell) const;

    // Check that the passage from `from` to `to` is wide/tall enough for the drone
    bool HasClearance(const GridCell3D& from, const GridCell3D& to) const;

    // Find path from current position to target using A*
    std::vector<GridCell3D> FindPath(const GridCell3D& target);

    // Execute a series of moves to reach the target cell
    bool MoveToCell(const GridCell3D& target);

    // Rotate to face a given direction
    bool RotateToFace(XLength targetX, YLength targetY);

    // Handle height changes (positive = up, negative = down)
    bool AdjustHeight(PhysicalLength targetHeight);
};

} // namespace drone
