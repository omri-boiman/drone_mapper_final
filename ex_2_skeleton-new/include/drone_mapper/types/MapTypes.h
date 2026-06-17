#pragma once

#include <drone_mapper/Units.h>

namespace drone_mapper::types {

enum class VoxelOccupancy {
    PotentiallyOccupied = -3,
    OutOfBounds = -2,
    Unmapped = -1,
    Empty = 0,
    Occupied = 1,
};

// Changed: moved from mission types because map bounds now belong to map configuration.
struct MappingBounds {
    XLength min_x{};
    XLength max_x{};
    YLength min_y{};
    YLength max_y{};
    ZLength min_height{};
    ZLength max_height{};
};

struct MappedVoxel {
    Position3D position{};
    VoxelOccupancy value = VoxelOccupancy::Unmapped;
};

// Changed: added to keep boundaries, offset, and resolution together on IMap3D.
// Note - IMap3D should be able to construct with a default MapConfig (aka no MapConfig. See Map3DImpl.h).
struct MapConfig {
    MappingBounds boundaries{};
    Position3D offset{};
    PhysicalLength resolution{};
};
struct ComparisonMapConfig {
    MapConfig map_config{};
};

struct GridCell3D {
    int x = 0, y = 0, z = 0;
    bool operator==(const GridCell3D& o) const noexcept = default;
    bool operator<(const GridCell3D& o) const noexcept {
        if (x != o.x) return x < o.x;
        if (y != o.y) return y < o.y;
        return z < o.z;
    }
};

} // namespace drone_mapper::types
