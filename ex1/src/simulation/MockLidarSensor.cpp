#include "simulation/MockLidarSensor.h"

#include <mp-units/systems/si/math.h>

#include <algorithm>
#include <optional>

namespace drone {

namespace {

[[nodiscard]] std::size_t beams_on_circle(std::size_t circle_index) {
    std::size_t count = 1;
    for (std::size_t i = 0; i < circle_index; ++i) {
        count *= 4;
    }
    return count;
}

[[nodiscard]] HorizontalAngle horizontal_delta(PhysicalLength offset, PhysicalLength distance) {
    return HorizontalAngle{si::atan2(offset, distance)};
}

[[nodiscard]] Altitude altitude_delta(PhysicalLength offset, PhysicalLength distance) {
    return Altitude{si::atan2(offset, distance)};
}

} // namespace

MockLidarSensor::MockLidarSensor(LidarConfig config,
                                 const IMap3D& map,
                                 const IPositionSensor& pos_sensor)
    : config_(config), map_(map), pos_sensor_(pos_sensor) {}

ScanResults MockLidarSensor::scan(Orientation rel_scan_orientation) const {
    ScanResults results;
    if (config_.fov_circles == 0) {
        return results;
    }

    const Orientation sensor_heading = pos_sensor_.heading();
    const Orientation beam_0{
        rel_scan_orientation.horizontal,
        rel_scan_orientation.altitude,
    };
    // scan beam_0: trace in absolute direction, return relative angle
    const Orientation beam_0_abs{
        beam_0.horizontal + sensor_heading.horizontal,
        beam_0.altitude + sensor_heading.altitude,
    };
    if (auto dist = traceBeam(beam_0_abs)) {
        results.push_back(LidarHit{*dist, beam_0});
    }

    for (std::size_t circle = 1; circle < config_.fov_circles; ++circle) {
        const std::size_t beam_count = beams_on_circle(circle);
        const PhysicalLength radius = static_cast<double>(circle) * config_.circle_spacing;

        for (std::size_t i = 0; i < beam_count; ++i) {
            const auto theta = (360.0 * static_cast<double>(i) / static_cast<double>(beam_count)) * deg;
            const PhysicalLength horizontal_offset = radius * si::cos(theta);
            const PhysicalLength altitude_offset   = radius * si::sin(theta);

            const Orientation offset{
                horizontal_delta(horizontal_offset, config_.beam_length_min),
                altitude_delta(altitude_offset, config_.beam_length_min),
            };

            const Orientation abs_circle_beam{
                beam_0.horizontal + offset.horizontal + sensor_heading.horizontal,
                beam_0.altitude   + offset.altitude   + sensor_heading.altitude,
            };
            const Orientation circle_beam{
                beam_0.horizontal + offset.horizontal,
                beam_0.altitude   + offset.altitude,
            };
            if (auto dist = traceBeam(abs_circle_beam)) {
                results.push_back(LidarHit{*dist, circle_beam});
            }
        }
    }

    return results;
}

const LidarConfig& MockLidarSensor::config() const noexcept {
    return config_;
}

std::optional<PhysicalLength> MockLidarSensor::traceBeam(const Orientation& beam_orientation) const {
    const Position3D origin = pos_sensor_.position();

    const auto cos_altitude = si::cos(beam_orientation.altitude);
    const auto dx = cos_altitude * si::cos(beam_orientation.horizontal);
    const auto dy = cos_altitude * si::sin(beam_orientation.horizontal);
    const auto dz = si::sin(beam_orientation.altitude);

    const PhysicalLength step        = PhysicalLength{0.1 * cm};
    const PhysicalLength min_distance = std::min(config_.beam_length_min, step);

    for (PhysicalLength distance = min_distance; distance <= config_.beam_length_max; distance += step) {
        const Position3D sample{
            origin.x + dx.force_numerical_value_in(mp::one) * distance.force_numerical_value_in(cm) * x_extent[cm],
            origin.y + dy.force_numerical_value_in(mp::one) * distance.force_numerical_value_in(cm) * y_extent[cm],
            origin.z + dz.force_numerical_value_in(mp::one) * distance.force_numerical_value_in(cm) * z_extent[cm],
        };

        if (map_.get(sample) != 0) {
            if (distance < config_.beam_length_min) {
                return PhysicalLength{0 * cm};
            }
            return distance;
        }
    }

    return std::nullopt;
}

} // namespace drone
