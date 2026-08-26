#pragma once

#include <Common/IMap3D.h>
#include <Common/Types.h>

namespace simulator {

using namespace common;

class MapsComparison {
public:
    [[nodiscard]] static std::vector<double> compare(const IMap3D& origin,
                                                     const std::vector<IMap3D*> targets); //currently should work with at least 1 target
};

} // namespace simulator
