#pragma once

#include <Common/IDroneMovement.h>
#include <Common/IMap3D.h>
#include <Common/types/DroneTypes.h>
#include <Simulator/MockGPS.h>

namespace simulator {

using namespace common;

class MockMovement final : public IDroneMovement {
public:
    MockMovement(MockGPS& gps, const IMap3D& hidden_map, const common::types::DroneConfigData& config);

    common::types::MovementResult rotate(common::types::RotationDirection direction, HorizontalAngle angle) override;
    common::types::MovementResult advance(PhysicalLength distance) override;
    common::types::MovementResult elevate(PhysicalLength distance) override;

private:
    MockGPS& gps_;
    const IMap3D& hidden_map_;
    const common::types::DroneConfigData& config_;
};

} // namespace simulator
