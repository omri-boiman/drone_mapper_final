#include <cpp_course/MockLidarSensor.h>
#include <cpp_course/PositionSensor.h>
#include <cpp_course/Units.h>
#include <cpp_course/VoxelGrid.h>

#include <mp-units/systems/si/math.h>

#include <cmath>
#include <exception>
#include <iostream>
#include <set>
#include <string>
#include <tuple>

namespace {

class MockPositionSensor final : public cpp_course::IPositionSensor {
public:
    MockPositionSensor(cpp_course::Position3D position, cpp_course::Orientation heading)
        : position_(position), heading_(heading) {}

    [[nodiscard]] cpp_course::Position3D position() const override {
        return position_;
    }

    [[nodiscard]] cpp_course::Orientation heading() const override {
        return heading_;
    }

private:
    cpp_course::Position3D position_;
    cpp_course::Orientation heading_;
};

void print_orientation(const std::string& label, const cpp_course::Orientation& orientation) {
    std::cout << label
              << " horizontal=" << orientation.horizontal
              << ", altitude=" << orientation.altitude << "\n";
}

cpp_course::Orientation absolute_beam(const cpp_course::Orientation& heading,
                                      const cpp_course::Orientation& relative_beam) {
    return {
        heading.horizontal + relative_beam.horizontal,
        heading.altitude + relative_beam.altitude,
    };
}

cpp_course::Position3D hit_to_position(const cpp_course::Position3D& origin,
                                       const cpp_course::Orientation& absolute_beam,
                                       cpp_course::PhysicalLength distance) {
    const auto cos_altitude = cpp_course::si::cos(absolute_beam.altitude);
    const auto dx = cos_altitude * cpp_course::si::cos(absolute_beam.horizontal);
    const auto dy = cos_altitude * cpp_course::si::sin(absolute_beam.horizontal);
    const auto dz = cpp_course::si::sin(absolute_beam.altitude);

    return {
        origin.x + dx.force_numerical_value_in(cpp_course::mp::one) *
                       distance.force_numerical_value_in(cpp_course::cm) * cpp_course::x_extent[cpp_course::cm],
        origin.y + dy.force_numerical_value_in(cpp_course::mp::one) *
                       distance.force_numerical_value_in(cpp_course::cm) * cpp_course::y_extent[cpp_course::cm],
        origin.z + dz.force_numerical_value_in(cpp_course::mp::one) *
                       distance.force_numerical_value_in(cpp_course::cm) * cpp_course::z_extent[cpp_course::cm],
    };
}

std::tuple<int, int, int> voxel_coordinates(const cpp_course::Position3D& position) {
    const int x = static_cast<int>(std::floor(position.x.force_numerical_value_in(cpp_course::cm)));
    const int y = static_cast<int>(std::floor(position.y.force_numerical_value_in(cpp_course::cm)));
    const int z = static_cast<int>(std::floor(position.z.force_numerical_value_in(cpp_course::cm)));
    return {x, y, z};
}

} // namespace

int main(int argc, char** argv) {
    using namespace cpp_course;

    const std::string map_path = (argc >= 2) ? argv[1] : "data_maps/single_voxel_x2_y4_z2.npy";

    try {
        VoxelGrid map(map_path);

        const MockPositionSensor position_sensor{
            Position3D{
                3.0 * x_extent[cm],
                4.0 * y_extent[cm],
                4.0 * z_extent[cm],
            },
            Orientation{
                0.0 * horizontal_angle[deg],
                0.0 * altitude_angle[deg],
            },
        };

        const LidarConfig config{
            1.0 * cm,
            20.0 * cm,
            0.5 * cm,
            5,
        };

        const MockLidarSensor lidar(config, map, position_sensor);

        const Orientation scans[] = {
            {0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]},
            {90.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]},
            {180.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]},
            {270.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]},
        };

        std::cout << "Mock lidar test using current project conventions\n";
        std::cout << "  map: " << map_path << "\n";
        std::cout << "  map shape [X, Y, Z]: ["
                  << map.x_size() << ", "
                  << map.y_size() << ", "
                  << map.z_size() << "]\n";
        std::cout << "  map units: 1 voxel == 1 cm\n";
        std::cout << "  drone position: "
                  << position_sensor.position().x << ", "
                  << position_sensor.position().y << ", "
                  << position_sensor.position().z << "\n";
        print_orientation("  drone heading:", position_sensor.heading());
        std::cout << "  beam length range: " << config.beam_length_min
                  << " to " << config.beam_length_max << "\n";
        std::cout << "  circle spacing: " << config.circle_spacing << "\n";
        std::cout << "  fov circles: " << config.fov_circles << "\n\n";

        bool all_hits_match_map = true;
        std::set<std::tuple<int, int, int>> unique_hit_points;

        for (std::size_t i = 0; i < 4; ++i) {
            std::cout << "Scan " << i << "\n";
            print_orientation("  requested relative scan:", scans[i]);

            const ScanResults hits = lidar.scan(scans[i]);
            std::cout << "  hit count: " << hits.size() << "\n";

            for (std::size_t hit_index = 0; hit_index < hits.size(); ++hit_index) {
                const Orientation beam_abs = absolute_beam(position_sensor.heading(), hits[hit_index].angle);
                const Position3D hit_position =
                    hit_to_position(position_sensor.position(), beam_abs, hits[hit_index].distance);
                const auto [x, y, z] = voxel_coordinates(hit_position);
                const int map_value = map.get(hit_position);
                const bool hit_matches_map = (map_value != 0);

                unique_hit_points.insert({x, y, z});
                all_hits_match_map = all_hits_match_map && hit_matches_map;

                std::cout << "  hit " << hit_index
                          << ": distance=" << hits[hit_index].distance
                          << ", returned beam horizontal=" << hits[hit_index].angle.horizontal
                          << ", returned beam altitude=" << hits[hit_index].angle.altitude
                          << ", reconstructed voxel=(" << x << "," << y << "," << z << ")"
                          << ", map value=" << map_value
                          << " -> " << (hit_matches_map ? "PASS" : "FAIL") << "\n";
            }

            std::cout << "\n";
        }

        std::cout << "Reconstructed hit points agree with map occupancy: "
                  << (all_hits_match_map ? "PASS" : "FAIL") << "\n";
        std::cout << "Unique hit voxel coordinates:\n";
        for (const auto& [x, y, z] : unique_hit_points) {
            std::cout << "  (" << x << "," << y << "," << z << ")\n";
        }

        if (!all_hits_match_map) {
            return 1;
        }
    } catch (const std::exception& error) {
        std::cerr << "Error: " << error.what() << "\n";
        return 1;
    }

    return 0;
}
