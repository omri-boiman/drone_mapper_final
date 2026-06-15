#include <cpp_course/MockLidarSensor.h>

#include <mp-units/systems/si/math.h>

#include <algorithm>
#include <optional>

namespace cpp_course {

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
    // beam_0 directed at the orientation of the scan!
    const Orientation& beam_0 = rel_scan_orientation; // "alias" for rel_scan_orientation!

    // absolute heading of beam 0 - for beam tracing
    const Orientation beam_0_abs{
        beam_0.horizontal + sensor_heading.horizontal,
        beam_0.altitude + sensor_heading.altitude,
    };
    // tracing beam_0
    if (auto dist = traceBeam(beam_0_abs)) { // if anything returned = beam hit
        const LidarHit hit{*dist,beam_0}; // returning relative orientation!
        results.push_back(hit); 
    }
    for (std::size_t circle = 1; circle < config_.fov_circles; ++circle) {
        const std::size_t beam_count = beams_on_circle(circle);
        const PhysicalLength radius = static_cast<double>(circle) * config_.circle_spacing / 2.0;

        for (std::size_t i = 0; i < beam_count; ++i) {
            const auto theta = (360.0 * static_cast<double>(i) / static_cast<double>(beam_count)) * deg; // generates the beam's angle from beam 0
            const PhysicalLength horizontal_offset = radius * si::cos(theta);
            const PhysicalLength altitude_offset = radius * si::sin(theta);
            
            const Orientation offset{
                horizontal_delta(horizontal_offset, config_.beam_length_min),
                altitude_delta(altitude_offset, config_.beam_length_min),
            };

            // Mock knows the abs circle beam, but regular does not! So we return without the sensorheading offset
            const Orientation abs_circle_beam{
                beam_0.horizontal + offset.horizontal + sensor_heading.horizontal,
                beam_0.altitude + offset.altitude + sensor_heading.altitude,
            };
            // Inefficient - can you think of a better way?
             const Orientation circle_beam{
                beam_0.horizontal + offset.horizontal,
                beam_0.altitude + offset.altitude,
            };
            if (auto dist = traceBeam(abs_circle_beam)) { // if anything returned = beam hit
                const LidarHit hit{*dist,circle_beam};
                results.push_back(hit); // drone needs to both convert to abs orientation AND cartesian!
            }
        }
    }

    return results;
}

const LidarConfig& MockLidarSensor::config() const noexcept {
    return config_;
}

std::optional<PhysicalLength> MockLidarSensor::traceBeam(const Orientation& beam_orientation) const {
    const Position3D origin = pos_sensor_.position(); // real Lidar will have its own way of calculating distance (timing reflections)

    // Convert the beam orientation into a unit direction vector.
    // Horizontal rotates in the X/Y plane; altitude tilts the ray above/below that plane.
    const auto cos_altitude = si::cos(beam_orientation.altitude);

    // Coordinate transformation - spherical to cartesian
    const auto dx = cos_altitude * si::cos(beam_orientation.horizontal);
    const auto dy = cos_altitude * si::sin(beam_orientation.horizontal);
    const auto dz = si::sin(beam_orientation.altitude);

    // March along the ray from the configured near distance to the far distance,
    // sampling one point every centimeter.
    const PhysicalLength step = PhysicalLength{0.1 * cm};
    const PhysicalLength min_distance = std::min(config_.beam_length_min, step);

    for (PhysicalLength distance = min_distance; distance <= config_.beam_length_max; distance += step) {
        // P(distance) = origin + direction * distance.
        // The direction components are dimensionless, while Position3D uses separate
        // X/Y/Z quantity specs, so each axis is rebuilt with its own extent type.
        const Position3D sample{
            origin.x + dx.force_numerical_value_in(mp::one) * distance.force_numerical_value_in(cm) * x_extent[cm],
            origin.y + dy.force_numerical_value_in(mp::one) * distance.force_numerical_value_in(cm) * y_extent[cm],
            origin.z + dz.force_numerical_value_in(mp::one) * distance.force_numerical_value_in(cm) * z_extent[cm],
        };

        if (map_.get(sample) != 0) {
            if (distance < config_.beam_length_min){
                return PhysicalLength{0*cm}; // too close
            }
            return distance; // stops at the first hit - 1 beam 1 hit
        }
    }

    return std::nullopt;
}

} // namespace cpp_course
