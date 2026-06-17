#pragma once

#include <drone_mapper/IGPS.h>
#include <drone_mapper/ILidar.h>
#include <drone_mapper/IMap3D.h>

namespace drone_mapper {

class MockLidar final : public ILidar {
public:
    MockLidar(types::LidarConfigData config, const IMap3D& map, const IGPS& gps);

    [[nodiscard]] types::LidarScanResult scan(Orientation scan_orientation) const override;

private:
    [[nodiscard]] PhysicalLength traceBeam(const Orientation& beam) const;

    types::LidarConfigData config_;
    const IMap3D& map_;
    const IGPS& gps_;
    PhysicalLength step_;       // cached: 0.1 * map resolution, constant for lifetime
    double step_cm_;            // cached as double to avoid repeated unit extraction
    double max_dist_cm_;        // cached: config_.z_max in cm
    double min_dist_cm_;        // cached: config_.z_min in cm
};

} // namespace drone_mapper
