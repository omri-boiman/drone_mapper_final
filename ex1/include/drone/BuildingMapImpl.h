#pragma once

#include <unordered_map>
#include <vector>
#include <utility>
#include "interfaces/IBuildingMap.h"
#include "io/ConfigParser.h"
#include "io/MapIO.h"

namespace drone {

class BuildingMapImpl : public IBuildingMap {
public:
    explicit BuildingMapImpl(const MissionConfig& mission);

    MapValue Get(XLength x, YLength y, ZLength z) const override;
    void     Set(XLength x, YLength y, ZLength z, MapValue value) override;

    const std::vector<std::pair<double,double>>& polygon()   const { return m_polygon; }
    ZLength minHeight() const { return m_minHeight; }
    ZLength maxHeight() const { return m_maxHeight; }

    std::vector<MapCell> GetAllCells() const;

private:
    struct Key {
        int ix, iy, iz;
        bool operator==(const Key& o) const noexcept {
            return ix == o.ix && iy == o.iy && iz == o.iz;
        }
    };
    struct KeyHash {
        std::size_t operator()(const Key& k) const noexcept {
            std::size_t seed = static_cast<std::size_t>(k.ix);
            seed ^= static_cast<std::size_t>(k.iy) + 0x9e3779b9u + (seed << 6) + (seed >> 2);
            seed ^= static_cast<std::size_t>(k.iz) + 0x9e3779b9u + (seed << 6) + (seed >> 2);
            return seed;
        }
    };

    Key  MakeKey(double xCm, double yCm, double zCm) const;
    bool IsInBounds(XLength x, YLength y, ZLength z) const;
    bool IsInsidePolygon(double xCm, double yCm) const;

    std::unordered_map<Key, MapValue, KeyHash> m_cells;

    const std::vector<std::pair<double,double>>& m_polygon;
    ZLength m_minHeight;
    ZLength m_maxHeight;
    double  m_xyCellCm;
    double  m_hCellCm;
};

} // namespace drone
