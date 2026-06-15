#include "simulation/MockMovementDriver.h"

#include <cmath>
#include <numbers>

namespace drone {

namespace {

HorizontalAngle ClampAngle(HorizontalAngle value, HorizontalAngle limit)
{
    if (value >  limit) return  limit;
    if (value < -limit) return -limit;
    return value;
}

PhysicalLength ClampDist(PhysicalLength value, PhysicalLength limit)
{
    if (value >  limit) return  limit;
    if (value < -limit) return -limit;
    return value;
}

HorizontalAngle WrapHeading(HorizontalAngle angle)
{
    double raw = std::fmod(angle.numerical_value_in(deg), 360.0);
    if (raw < 0.0) raw += 360.0;
    return raw * deg;
}

} // anonymous namespace

MockMovementDriver::MockMovementDriver(std::shared_ptr<SimulationState> state,
                                       const DroneConfig&               config,
                                       const IMap3D&                    map)
    : m_state(std::move(state))
    , m_config(config)
    , m_map(map)
{}

MoveResult MockMovementDriver::Rotate(HorizontalAngle angle)
{
    m_state->orientation.horizontal =
        WrapHeading(m_state->orientation.horizontal + ClampAngle(angle, m_config.maxRotate));
    return MoveResult::Success;
}

MoveResult MockMovementDriver::Advance(PhysicalLength distance)
{
    const PhysicalLength distClamped = ClampDist(distance, m_config.maxAdvance);
    const double distCm = distClamped.numerical_value_in(cm);

    const double headingRad =
        m_state->orientation.horizontal.numerical_value_in(deg)
        * std::numbers::pi / 180.0;

    const double dx = distCm * std::cos(headingRad);
    const double dy = distCm * std::sin(headingRad);

    const double startX = m_state->position.x.numerical_value_in(cm);
    const double startY = m_state->position.y.numerical_value_in(cm);
    const double startZ = m_state->position.z.numerical_value_in(cm);

    const int steps = static_cast<int>(std::abs(distCm)) + 1;
    for (int i = 1; i <= steps; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(steps);
        const Position3D sample{
            (startX + t * dx) * x_extent[cm],
            (startY + t * dy) * y_extent[cm],
            startZ            * z_extent[cm],
        };
        if (m_map.get(sample) != 0) {
            return MoveResult::CollisionDetected;
        }
    }

    m_state->position.x = (startX + dx) * x_extent[cm];
    m_state->position.y = (startY + dy) * y_extent[cm];
    return MoveResult::Success;
}

MoveResult MockMovementDriver::Elevate(PhysicalLength distance)
{
    const PhysicalLength distClamped = ClampDist(distance, m_config.maxElevate);
    const double distCm = distClamped.numerical_value_in(cm);

    const double startX = m_state->position.x.numerical_value_in(cm);
    const double startY = m_state->position.y.numerical_value_in(cm);
    const double startZ = m_state->position.z.numerical_value_in(cm);

    const int steps = static_cast<int>(std::abs(distCm)) + 1;
    for (int i = 1; i <= steps; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(steps);
        const Position3D sample{
            startX              * x_extent[cm],
            startY              * y_extent[cm],
            (startZ + t * distCm) * z_extent[cm],
        };
        if (m_map.get(sample) != 0) {
            return MoveResult::CollisionDetected;
        }
    }

    m_state->position.z = (startZ + distCm) * z_extent[cm];
    return MoveResult::Success;
}

} // namespace drone
