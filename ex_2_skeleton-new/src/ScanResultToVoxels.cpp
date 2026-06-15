#include <drone_mapper/ScanResultToVoxels.h>

#include <mp-units/systems/si/math.h>

#include <limits>

namespace drone_mapper {

std::vector<types::MappedVoxel> ScanResultToVoxels::convert(const Position3D& scan_origin,
                                                            const Orientation& drone_heading,
                                                            const types::LidarScanResult& scan) {
    std::vector<types::MappedVoxel> voxels;

    constexpr double step_cm = 1.0;
    constexpr double miss_threshold = 1e15; // std::numeric_limits<double>::max() means no hit

    for (const auto& lidar_hit : scan) {
        const double hit_cm = lidar_hit.distance.force_numerical_value_in(cm);
        const bool is_hit = hit_cm < miss_threshold;

        const Orientation abs_beam{
            drone_heading.horizontal + lidar_hit.angle.horizontal,
            drone_heading.altitude   + lidar_hit.angle.altitude,
        };

        const auto cos_alt = si::cos(abs_beam.altitude);
        const double dir_x = (cos_alt * si::cos(abs_beam.horizontal)).force_numerical_value_in(mp::one);
        const double dir_y = (cos_alt * si::sin(abs_beam.horizontal)).force_numerical_value_in(mp::one);
        const double dir_z = si::sin(abs_beam.altitude).force_numerical_value_in(mp::one);

        const double origin_x = scan_origin.x.force_numerical_value_in(cm);
        const double origin_y = scan_origin.y.force_numerical_value_in(cm);
        const double origin_z = scan_origin.z.force_numerical_value_in(cm);

        if (!is_hit) continue;

        const double empty_end = hit_cm - step_cm;

        for (double d = 0.0; d <= empty_end; d += step_cm) {
            const Position3D pos{
                (origin_x + dir_x * d) * x_extent[cm],
                (origin_y + dir_y * d) * y_extent[cm],
                (origin_z + dir_z * d) * z_extent[cm],
            };
            voxels.push_back({pos, types::VoxelOccupancy::Empty});
        }

        if (is_hit && hit_cm > 0.0) {
            const Position3D hit_pos{
                (origin_x + dir_x * hit_cm) * x_extent[cm],
                (origin_y + dir_y * hit_cm) * y_extent[cm],
                (origin_z + dir_z * hit_cm) * z_extent[cm],
            };
            voxels.push_back({hit_pos, types::VoxelOccupancy::Occupied});
        }
    }

    return voxels;
}

} // namespace drone_mapper
