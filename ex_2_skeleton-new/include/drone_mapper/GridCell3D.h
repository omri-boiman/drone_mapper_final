#pragma once

#include <cstddef>
#include <functional>

namespace drone_mapper::types {

// Internal integer grid-cell key used by MappingAlgorithmImpl and MapsComparison.
// Not part of the staff-provided interface — kept out of types/MapTypes.h on purpose.
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

namespace std {
template <>
struct hash<drone_mapper::types::GridCell3D> {
    std::size_t operator()(const drone_mapper::types::GridCell3D& c) const noexcept {
        std::size_t h = std::hash<int>{}(c.x);
        h ^= std::hash<int>{}(c.y) + 0x9e3779b9u + (h << 6) + (h >> 2);
        h ^= std::hash<int>{}(c.z) + 0x9e3779b9u + (h << 6) + (h >> 2);
        return h;
    }
};
} // namespace std
