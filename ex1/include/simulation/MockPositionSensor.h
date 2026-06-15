#pragma once

#include <memory>
#include "interfaces/IPositionSensor.h"
#include "simulation/SimulationState.h"

namespace drone {

class MockPositionSensor final : public IPositionSensor {
public:
    explicit MockPositionSensor(std::shared_ptr<const SimulationState> state);

    [[nodiscard]] Position3D  position() const override;
    [[nodiscard]] Orientation heading()  const override;

private:
    std::shared_ptr<const SimulationState> m_state;
};

} // namespace drone
