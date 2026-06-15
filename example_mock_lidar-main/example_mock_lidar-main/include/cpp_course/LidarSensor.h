#pragma once

#include <cstddef>
#include <vector>

#include <cpp_course/Units.h>

namespace cpp_course {

// Static scan settings for a LiDAR-like sensor.
struct LidarConfig {
    PhysicalLength beam_length_min{};
    PhysicalLength beam_length_max{};
    PhysicalLength circle_spacing{};
    std::size_t fov_circles{};
};

// A single LiDAR return. The angle is relative to the requested scan orientation.
struct LidarHit {
    PhysicalLength distance{};
    Orientation angle{};
};

typedef std::vector<LidarHit> ScanResults;

class ILidarSensor {
public:
    virtual ~ILidarSensor() = default;

    // Scans around the requested relative orientation and returns all beam hits.
    [[nodiscard]] virtual ScanResults scan(Orientation scan_orientation) const = 0;
};

} // namespace cpp_course
