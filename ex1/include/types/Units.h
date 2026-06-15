#pragma once

#include <mp-units/framework.h>
#include <mp-units/systems/isq.h>
#include <mp-units/systems/si/unit_symbols.h>

namespace drone {

namespace mp  = mp_units;
namespace isq = mp_units::isq;
namespace si  = mp_units::si;

using mp_units::si::unit_symbols::deg;
using mp_units::si::unit_symbols::cm;

// Distinct quantity specs for each spatial axis — required to build a type-safe
// Position3D where x, y, z are not interchangeable at compile time.
QUANTITY_SPEC(x_extent, isq::length);
QUANTITY_SPEC(y_extent, isq::length);
QUANTITY_SPEC(z_extent, isq::length);

using PhysicalLength = mp::quantity<isq::length[cm], double>;  // generic distance in cm
using XLength        = mp::quantity<x_extent[cm],   double>;
using YLength        = mp::quantity<y_extent[cm],   double>;
using ZLength        = mp::quantity<z_extent[cm],   double>;

struct Position3D {
    XLength x{};
    YLength y{};
    ZLength z{};   // vertical axis (was "height" in v1)
};

// Distinct angle specs so horizontal and altitude angles are not interchangeable.
QUANTITY_SPEC(horizontal_angle, isq::angular_measure);
QUANTITY_SPEC(altitude_angle,   isq::angular_measure);

using HorizontalAngle = mp::quantity<horizontal_angle[deg], double>;
using Altitude        = mp::quantity<altitude_angle[deg],   double>;

struct Orientation {
    HorizontalAngle horizontal{};  // rotation in XY plane (was "heading" in v1)
    Altitude        altitude{};    // tilt above/below horizontal (was "pitch" in v1)
};

} // namespace drone
