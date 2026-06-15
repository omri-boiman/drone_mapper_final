#pragma once

#include <cpp_course/Units.h>

#include <mp-units/systems/si/math.h>

#include <cmath>
#include <tuple>

namespace cpp_course::test {

inline Position3D make_position(double x_cm, double y_cm, double z_cm) {
    return {
        x_cm * x_extent[cm],
        y_cm * y_extent[cm],
        z_cm * z_extent[cm],
    };
}

inline Orientation make_orientation(double horizontal_deg, double altitude_deg) {
    return {
        horizontal_deg * horizontal_angle[deg],
        altitude_deg * altitude_angle[deg],
    };
}

inline Orientation absolute_beam(const Orientation& heading, const Orientation& relative_beam) {
    return {
        heading.horizontal + relative_beam.horizontal,
        heading.altitude + relative_beam.altitude,
    };
}

inline Position3D hit_to_position(const Position3D& origin,
                                  const Orientation& absolute_beam_orientation,
                                  PhysicalLength distance) {
    const auto cos_altitude = si::cos(absolute_beam_orientation.altitude);
    const auto dx = cos_altitude * si::cos(absolute_beam_orientation.horizontal);
    const auto dy = cos_altitude * si::sin(absolute_beam_orientation.horizontal);
    const auto dz = si::sin(absolute_beam_orientation.altitude);

    return {
        origin.x + dx.force_numerical_value_in(mp::one) * distance.force_numerical_value_in(cm) * x_extent[cm],
        origin.y + dy.force_numerical_value_in(mp::one) * distance.force_numerical_value_in(cm) * y_extent[cm],
        origin.z + dz.force_numerical_value_in(mp::one) * distance.force_numerical_value_in(cm) * z_extent[cm],
    };
}

inline std::tuple<int, int, int> voxel_coordinates(const Position3D& position) {
    return {
        static_cast<int>(std::floor(position.x.force_numerical_value_in(cm))),
        static_cast<int>(std::floor(position.y.force_numerical_value_in(cm))),
        static_cast<int>(std::floor(position.z.force_numerical_value_in(cm))),
    };
}

inline double degrees(HorizontalAngle angle) {
    return angle.force_numerical_value_in(deg);
}

inline double degrees(Altitude angle) {
    return angle.force_numerical_value_in(deg);
}

inline double wrap_degrees(double angle) {
    while (angle < 0.0) {
        angle += 360.0;
    }
    while (angle >= 360.0) {
        angle -= 360.0;
    }
    return angle;
}

} // namespace cpp_course::test
