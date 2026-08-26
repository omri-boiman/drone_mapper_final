#pragma once

#include <cstddef>
#include <functional>

namespace user_common_211781141_325049575 {

// Internal integer grid-cell key used by the Algorithm project's mapping algorithm and by
// the Simulator project's map-comparison scoring. Shared between two projects, so it lives
// here rather than being duplicated (or, worse, smuggled into the staff-owned common/ headers).
struct GridCell3D {
    int x = 0, y = 0, z = 0;
    bool operator==(const GridCell3D& o) const noexcept = default;
    bool operator<(const GridCell3D& o) const noexcept {
        if (x != o.x) return x < o.x;
        if (y != o.y) return y < o.y;
        return z < o.z;
    }
};

} // namespace user_common_211781141_325049575

namespace std {
template <>
struct hash<user_common_211781141_325049575::GridCell3D> {
    std::size_t operator()(const user_common_211781141_325049575::GridCell3D& c) const noexcept {
        std::size_t h = std::hash<int>{}(c.x);
        h ^= std::hash<int>{}(c.y) + 0x9e3779b9u + (h << 6) + (h >> 2);
        h ^= std::hash<int>{}(c.z) + 0x9e3779b9u + (h << 6) + (h >> 2);
        return h;
    }
};
} // namespace std
