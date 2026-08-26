#pragma once

#include <Common/IGPS.h>
#include <Common/ILidar.h>
#include <Common/IMap3D.h>

namespace simulator {

using namespace common;

class MockLidar final : public ILidar {
public:
    MockLidar(common::types::LidarConfigData config, const IMap3D& map, const IGPS& gps);

    [[nodiscard]] common::types::LidarScanResult scan(Orientation scan_orientation) const override;
    [[nodiscard]] common::types::LidarConfigData config() const override;

private:
    [[nodiscard]] PhysicalLength traceBeam(const Orientation& beam) const;

    common::types::LidarConfigData config_;
    const IMap3D& map_;
    const IGPS& gps_;
};

} // namespace simulator
