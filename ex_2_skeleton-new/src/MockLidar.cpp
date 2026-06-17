#include <drone_mapper/MockLidar.h>

#include <mp-units/systems/si/math.h>

#include <algorithm>
#include <limits>
#include <thread>
#include <vector>

namespace drone_mapper {

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

[[nodiscard]] AltitudeAngle altitude_delta(PhysicalLength offset, PhysicalLength distance) {
    return AltitudeAngle{si::atan2(offset, distance)};
}

} // namespace

// E: cache step and z-limits at construction to avoid getMapConfig() per beam
MockLidar::MockLidar(types::LidarConfigData config, const IMap3D& map, const IGPS& gps)
    : config_(config)
    , map_(map)
    , gps_(gps)
    , step_(0.1 * map.getMapConfig().resolution)
    , step_cm_(step_.force_numerical_value_in(cm))
    , max_dist_cm_(config.z_max.force_numerical_value_in(cm))
    , min_dist_cm_(config.z_min.force_numerical_value_in(cm))
{}

types::LidarScanResult MockLidar::scan(Orientation scan_orientation) const {
    if (config_.fov_circles == 0) {
        return {};
    }

    // Pre-compute total beam count so we can pre-size the result vector
    std::size_t total = 1; // center beam
    for (std::size_t c = 1; c < config_.fov_circles; ++c) {
        total += beams_on_circle(c);
    }

    // B: build all (absolute, relative) beam pairs, then trace in parallel
    struct BeamJob {
        Orientation abs_beam;
        Orientation rel_beam;
    };
    std::vector<BeamJob> jobs;
    jobs.reserve(total);

    const Orientation sensor_heading = gps_.heading();
    const Orientation center_beam_abs{
        scan_orientation.horizontal + sensor_heading.horizontal,
        scan_orientation.altitude + sensor_heading.altitude,
    };
    jobs.push_back({center_beam_abs, scan_orientation});

    for (std::size_t circle = 1; circle < config_.fov_circles; ++circle) {
        const std::size_t beam_count = beams_on_circle(circle);
        const PhysicalLength radius = static_cast<double>(circle) * config_.d;

        for (std::size_t i = 0; i < beam_count; ++i) {
            const auto theta = (360.0 * static_cast<double>(i) / static_cast<double>(beam_count)) * deg;
            const PhysicalLength horizontal_offset = radius * si::cos(theta);
            const PhysicalLength altitude_offset   = radius * si::sin(theta);

            const Orientation offset{
                horizontal_delta(horizontal_offset, config_.z_min),
                altitude_delta(altitude_offset, config_.z_min),
            };
            const Orientation relative_beam{
                scan_orientation.horizontal + offset.horizontal,
                scan_orientation.altitude + offset.altitude,
            };
            const Orientation absolute_beam{
                relative_beam.horizontal + sensor_heading.horizontal,
                relative_beam.altitude + sensor_heading.altitude,
            };
            jobs.push_back({absolute_beam, relative_beam});
        }
    }

    // Pre-sized result vector — each thread writes to its own index range, no races
    types::LidarScanResult results(total);

    const std::size_t n_threads = std::max(1u, std::thread::hardware_concurrency());
    const std::size_t chunk = (total + n_threads - 1) / n_threads;

    std::vector<std::thread> threads;
    threads.reserve(n_threads);

    for (std::size_t t = 0; t < n_threads; ++t) {
        const std::size_t begin = t * chunk;
        if (begin >= total) break;
        const std::size_t end = std::min(begin + chunk, total);

        threads.emplace_back([this, &jobs, &results, begin, end]() {
            for (std::size_t i = begin; i < end; ++i) {
                results[i] = types::LidarHit{traceBeam(jobs[i].abs_beam), jobs[i].rel_beam};
            }
        });
    }
    for (auto& th : threads) th.join();

    return results;
}

PhysicalLength MockLidar::traceBeam(const Orientation& beam_orientation) const {
    const Position3D origin = gps_.position();
    const auto cos_altitude = si::cos(beam_orientation.altitude);
    const auto dx = cos_altitude * si::cos(beam_orientation.horizontal);
    const auto dy = cos_altitude * si::sin(beam_orientation.horizontal);
    const auto dz = si::sin(beam_orientation.altitude);

    // Hoist all constants outside the inner loop
    const double dir_x = dx.force_numerical_value_in(mp::one);
    const double dir_y = dy.force_numerical_value_in(mp::one);
    const double dir_z = dz.force_numerical_value_in(mp::one);
    const double ox = origin.x.force_numerical_value_in(cm);
    const double oy = origin.y.force_numerical_value_in(cm);
    const double oz = origin.z.force_numerical_value_in(cm);

    for (double dist_cm = 0.0; dist_cm <= max_dist_cm_; dist_cm += step_cm_) {
        const Position3D sample{
            (ox + dir_x * dist_cm) * x_extent[cm],
            (oy + dir_y * dist_cm) * y_extent[cm],
            (oz + dir_z * dist_cm) * z_extent[cm],
        };
        if (map_.atVoxel(sample) == types::VoxelOccupancy::Occupied) {
            if (dist_cm < min_dist_cm_) {
                return 0.0 * cm;
            }
            return dist_cm * cm;
        }
    }
    return std::numeric_limits<double>::max() * cm;
}

} // namespace drone_mapper
