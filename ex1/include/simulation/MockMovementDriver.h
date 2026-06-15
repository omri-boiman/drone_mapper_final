#pragma once

#include <memory>
#include "interfaces/IMovementDriver.h"
#include "interfaces/IMap3D.h"
#include "simulation/SimulationState.h"
#include "io/ConfigParser.h"

namespace drone {

class MockMovementDriver : public IMovementDriver {
public:
    MockMovementDriver(std::shared_ptr<SimulationState> state,
                       const DroneConfig&               config,
                       const IMap3D&                    map);

    MoveResult Rotate(HorizontalAngle angle)    override;
    MoveResult Advance(PhysicalLength distance)  override;
    MoveResult Elevate(PhysicalLength distance)  override;

private:
    std::shared_ptr<SimulationState> m_state;
    const DroneConfig&               m_config;
    const IMap3D&                    m_map;
};

} // namespace drone
