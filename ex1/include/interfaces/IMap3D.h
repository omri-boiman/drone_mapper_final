#pragma once

#include "types/Units.h"

namespace drone {

class IMap3D {
public:
    virtual ~IMap3D() = default;

    [[nodiscard]] virtual int get(const Position3D& pos) const = 0;
};

} // namespace drone
