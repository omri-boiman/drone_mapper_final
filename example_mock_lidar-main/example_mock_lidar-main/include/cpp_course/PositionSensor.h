#pragma once

#include <cpp_course/Units.h>

namespace cpp_course {

// Supplies the current world-space pose of the sensor platform.
class IPositionSensor {
public:
    virtual ~IPositionSensor() = default;

    [[nodiscard]] virtual Position3D position() const = 0;
    [[nodiscard]] virtual Orientation heading() const = 0;
};

} // namespace cpp_course
