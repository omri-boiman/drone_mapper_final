#pragma once

#include <cstddef>
#include <vector>

#include "types/Units.h"

namespace drone {

struct LidarConfig {
    PhysicalLength beam_length_min{};
    PhysicalLength beam_length_max{};
    PhysicalLength circle_spacing{};
    std::size_t    fov_circles{};
};

struct LidarHit {
    PhysicalLength distance{};
    Orientation    angle{};
};

using ScanResults = std::vector<LidarHit>;

class ILidarSensor {
public:
    virtual ~ILidarSensor() = default;

    [[nodiscard]] virtual ScanResults scan(Orientation scan_orientation) const = 0;
};

} // namespace drone
