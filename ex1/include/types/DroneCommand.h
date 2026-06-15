#pragma once

#include <variant>
#include "types/Units.h"

namespace drone {

struct RotateCmd  { HorizontalAngle angle; };
struct AdvanceCmd { PhysicalLength   distance; };
struct ElevateCmd { PhysicalLength   distance; };
struct ScanCmd    { Orientation      scan_orientation{}; };
struct GetLocationCmd {};
struct FinishedCmd    {};

using DroneCommand = std::variant<
    RotateCmd,
    AdvanceCmd,
    ElevateCmd,
    ScanCmd,
    GetLocationCmd,
    FinishedCmd
>;

} // namespace drone
