#include "simulation/CellMap.h"

#include <cmath>

namespace drone {

CellMap::CellMap(const ParsedMap& parsed)
{
    for (const auto& cell : parsed.cells) {
        if (cell.value == MapValue::Occupied) {
            const int ix = static_cast<int>(std::floor(cell.x.numerical_value_in(cm)));
            const int iy = static_cast<int>(std::floor(cell.y.numerical_value_in(cm)));
            const int iz = static_cast<int>(std::floor(cell.z.numerical_value_in(cm)));
            m_cells[{ix, iy, iz}] = 1;
        }
    }
}

CellMap::Key CellMap::makeKey(const Position3D& pos)
{
    return {
        static_cast<int>(std::floor(pos.x.numerical_value_in(cm))),
        static_cast<int>(std::floor(pos.y.numerical_value_in(cm))),
        static_cast<int>(std::floor(pos.z.numerical_value_in(cm))),
    };
}

int CellMap::get(const Position3D& pos) const
{
    const auto it = m_cells.find(makeKey(pos));
    return (it != m_cells.end()) ? it->second : 0;
}

} // namespace drone
