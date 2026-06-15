#pragma once

#include "types/Units.h"

namespace drone {

class IPositionSensor {
public:
    virtual ~IPositionSensor() = default;

    [[nodiscard]] virtual Position3D  position() const = 0;
    [[nodiscard]] virtual Orientation heading()  const = 0;
};

} // namespace drone
