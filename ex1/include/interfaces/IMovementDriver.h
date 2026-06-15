#pragma once

#include "types/Units.h"

namespace drone {

enum class MoveResult {
    Success,
    CollisionDetected
};

class IMovementDriver {
public:
    virtual ~IMovementDriver() = default;

    virtual MoveResult Rotate(HorizontalAngle angle)    = 0;
    virtual MoveResult Advance(PhysicalLength distance)  = 0;
    virtual MoveResult Elevate(PhysicalLength distance)  = 0;
};

} // namespace drone
