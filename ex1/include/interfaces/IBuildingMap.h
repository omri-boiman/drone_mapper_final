#pragma once

#include "types/Units.h"
#include "types/MapValue.h"

namespace drone {

class IBuildingMap {
public:
    virtual ~IBuildingMap() = default;

    virtual MapValue Get(XLength x, YLength y, ZLength z) const = 0;
    virtual void     Set(XLength x, YLength y, ZLength z, MapValue value) = 0;
};

} // namespace drone
