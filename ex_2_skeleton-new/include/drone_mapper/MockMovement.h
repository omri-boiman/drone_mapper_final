#pragma once

#include <drone_mapper/IDroneMovement.h>
#include <drone_mapper/IMap3D.h>
#include <drone_mapper/MockGPS.h>
#include <drone_mapper/types/DroneTypes.h>

namespace drone_mapper {

class MockMovement final : public IDroneMovement {
public:
    MockMovement(MockGPS& gps, const IMap3D& hidden_map, const types::DroneConfigData& config);

    types::MovementResult rotate(types::RotationDirection direction, HorizontalAngle angle) override;
    types::MovementResult advance(PhysicalLength distance) override;
    types::MovementResult elevate(PhysicalLength distance) override;

private:
    MockGPS& gps_;
    const IMap3D& hidden_map_;
    const types::DroneConfigData& config_;
};

} // namespace drone_mapper
