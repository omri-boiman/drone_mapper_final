#pragma once

#include <unordered_map>
#include "interfaces/IMap3D.h"
#include "io/MapIO.h"

namespace drone {

class CellMap final : public IMap3D {
public:
    explicit CellMap(const ParsedMap& parsed);

    [[nodiscard]] int get(const Position3D& pos) const override;

private:
    struct Key {
        int x, y, z;
        bool operator==(const Key& o) const noexcept {
            return x == o.x && y == o.y && z == o.z;
        }
    };
    struct KeyHash {
        std::size_t operator()(const Key& k) const noexcept {
            // FNV-style mixing
            std::size_t h = 2166136261u;
            h ^= static_cast<std::size_t>(k.x); h *= 16777619u;
            h ^= static_cast<std::size_t>(k.y); h *= 16777619u;
            h ^= static_cast<std::size_t>(k.z); h *= 16777619u;
            return h;
        }
    };

    static Key makeKey(const Position3D& pos);

    std::unordered_map<Key, int, KeyHash> m_cells;
};

} // namespace drone
