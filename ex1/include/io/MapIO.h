#pragma once

#include <filesystem>
#include <vector>
#include "types/MapValue.h"
#include "types/Units.h"
#include "io/ErrorLogger.h"

namespace drone {

struct MapBounds {
    PhysicalLength xmin{0 * cm}, xmax{0 * cm};
    PhysicalLength ymin{0 * cm}, ymax{0 * cm};
    PhysicalLength zmin{0 * cm}, zmax{0 * cm};
};

struct MapCell {
    PhysicalLength x{0 * cm};
    PhysicalLength y{0 * cm};
    PhysicalLength z{0 * cm};
    MapValue       value{MapValue::NotMapped};
};

struct ParsedMap {
    bool                 valid{false};
    MapBounds            bounds;
    std::vector<MapCell> cells;
};

ParsedMap ParseMapFile(const std::filesystem::path& filePath,
                       ErrorLogger& logger);

bool WriteMapFile(const std::filesystem::path& filePath,
                  const MapBounds&              bounds,
                  const std::vector<MapCell>&   cells);

} // namespace drone
