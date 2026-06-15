#include "simulation/MockPositionSensor.h"

namespace drone {

MockPositionSensor::MockPositionSensor(std::shared_ptr<const SimulationState> state)
    : m_state(std::move(state)) {}

Position3D MockPositionSensor::position() const {
    return m_state->position;
}

Orientation MockPositionSensor::heading() const {
    return m_state->orientation;
}

} // namespace drone
