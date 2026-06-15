#include "drone/BuildingMapImpl.h"

#include <cmath>

namespace drone {

BuildingMapImpl::BuildingMapImpl(const MissionConfig& mission)
    : m_polygon(mission.boundaryPolygon)
    , m_minHeight(mission.minHeight)
    , m_maxHeight(mission.maxHeight)
    , m_xyCellCm(mission.outputResXYCm)
    , m_hCellCm(mission.outputResHCm)
{}

BuildingMapImpl::Key BuildingMapImpl::MakeKey(double xCm, double yCm, double zCm) const
{
    return {
        static_cast<int>(std::floor(xCm / m_xyCellCm)),
        static_cast<int>(std::floor(yCm / m_xyCellCm)),
        static_cast<int>(std::floor(zCm / m_hCellCm))
    };
}

bool BuildingMapImpl::IsInsidePolygon(double xCm, double yCm) const
{
    const std::size_t n = m_polygon.size();
    if (n < 3) return false;

    bool inside = false;
    for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
        const double xi = m_polygon[i].first,  yi = m_polygon[i].second;
        const double xj = m_polygon[j].first,  yj = m_polygon[j].second;

        const bool crosses = ((yi > yCm) != (yj > yCm)) &&
                             (xCm < (xj - xi) * (yCm - yi) / (yj - yi) + xi);
        if (crosses) inside = !inside;
    }
    return inside;
}

bool BuildingMapImpl::IsInBounds(XLength x, YLength y, ZLength z) const
{
    if (z < m_minHeight || z > m_maxHeight) return false;
    return IsInsidePolygon(x.numerical_value_in(cm), y.numerical_value_in(cm));
}

MapValue BuildingMapImpl::Get(XLength x, YLength y, ZLength z) const
{
    if (!IsInBounds(x, y, z)) return MapValue::BeyondBounds;

    const auto it = m_cells.find(MakeKey(
        x.numerical_value_in(cm),
        y.numerical_value_in(cm),
        z.numerical_value_in(cm)));
    return (it != m_cells.end()) ? it->second : MapValue::NotMapped;
}

void BuildingMapImpl::Set(XLength x, YLength y, ZLength z, MapValue value)
{
    if (!IsInBounds(x, y, z)) return;

    m_cells[MakeKey(
        x.numerical_value_in(cm),
        y.numerical_value_in(cm),
        z.numerical_value_in(cm))] = value;
}

std::vector<MapCell> BuildingMapImpl::GetAllCells() const
{
    std::vector<MapCell> result;
    result.reserve(m_cells.size());
    for (const auto& [key, value] : m_cells) {
        const double xCm = key.ix * m_xyCellCm;
        const double yCm = key.iy * m_xyCellCm;
        const double zCm = key.iz * m_hCellCm;
        result.push_back({xCm * cm, yCm * cm, zCm * cm, value});
    }
    return result;
}

} // namespace drone
