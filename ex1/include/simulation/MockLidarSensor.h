#pragma once

#include <optional>

#include "interfaces/IMap3D.h"
#include "interfaces/ILidarSensor.h"
#include "interfaces/IPositionSensor.h"

namespace drone {

class MockLidarSensor final : public ILidarSensor {
public:
    MockLidarSensor(LidarConfig config, const IMap3D& map, const IPositionSensor& pos_sensor);

    [[nodiscard]] ScanResults scan(Orientation rel_scan_orientation) const override;

    [[nodiscard]] const LidarConfig& config() const noexcept;

private:
    [[nodiscard]] std::optional<PhysicalLength> traceBeam(const Orientation& beam) const;

    LidarConfig            config_;
    const IMap3D&          map_;
    const IPositionSensor& pos_sensor_;
};

} // namespace drone
