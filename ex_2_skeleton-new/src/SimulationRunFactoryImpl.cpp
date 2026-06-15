#include <drone_mapper/SimulationRunFactoryImpl.h>

#include <drone_mapper/DroneControlImpl.h>
#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MappingAlgorithmImpl.h>
#include <drone_mapper/MissionControlImpl.h>
#include <drone_mapper/MockGPS.h>
#include <drone_mapper/MockLidar.h>
#include <drone_mapper/MockMovement.h>
#include <drone_mapper/SimulationRunImpl.h>

#include <filesystem>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace drone_mapper {

namespace {

std::pair<PhysicalLength, types::ResolutionRequestStatus>
resolveOutputResolution(const types::MissionConfigData& mission) {
    const double factor = mission.output_mapping_resolution_factor;
    if (factor < 1.0) {
        return {mission.gps_resolution, types::ResolutionRequestStatus::IgnoredTooSmall};
    }
    return {mission.gps_resolution * factor, types::ResolutionRequestStatus::Accepted};
}

} // namespace

std::unique_ptr<ISimulationRun>
SimulationRunFactoryImpl::create(const types::SimulationConfigData& simulation,
                                 const types::MissionConfigData& mission,
                                 const types::DroneConfigData& drone,
                                 const types::LidarConfigData& lidar,
                                 const std::filesystem::path& output_path) {
    // Hidden map: load with offset so world coords align with map_axes_offset
    auto hidden_map = std::make_unique<Map3DImpl>(
        simulation.map_filename,
        simulation.map_resolution,
        simulation.map_offset);

    const auto hidden_cfg = hidden_map->getMapConfig();

    // Output map: same boundaries and offset as hidden map, at output resolution
    const auto [out_res, res_status] = resolveOutputResolution(mission);
    auto output_map = std::make_unique<Map3DImpl>(
        hidden_cfg.boundaries, out_res, hidden_cfg.offset);

    // Mock hardware
    auto gps = std::make_unique<MockGPS>(
        simulation.initial_drone_position,
        Orientation{simulation.initial_angle, 0.0 * altitude_angle[deg]});
    auto movement   = std::make_unique<MockMovement>(*gps, *hidden_map, drone);
    auto lidar_impl = std::make_unique<MockLidar>(lidar, *hidden_map, *gps);

    // Algorithm uses bounds from the hidden map
    auto algorithm = std::make_unique<MappingAlgorithmImpl>(
        mission, drone, hidden_cfg.boundaries);

    // Unique output filename per (sim × mission × drone × lidar) combo
    std::filesystem::create_directories(output_path);
    const std::string fname =
        simulation.map_filename.stem().string() + "_" +
        std::to_string(static_cast<int>(
            mission.gps_resolution.force_numerical_value_in(cm))) + "cm_" +
        std::to_string(static_cast<int>(
            drone.dimensions.force_numerical_value_in(cm))) + "d_" +
        std::to_string(static_cast<int>(
            lidar.z_max.force_numerical_value_in(cm))) + "lz.npy";
    const std::filesystem::path output_map_file = output_path / fname;

    auto drone_ctrl = std::make_unique<DroneControlImpl>(
        drone, mission, *lidar_impl, *gps, *movement, *output_map, *algorithm);

    auto mission_ctrl = std::make_unique<MissionControlImpl>(
        mission, drone, *hidden_map, *output_map, *drone_ctrl, output_map_file);

    return std::make_unique<SimulationRunImpl>(
        std::move(hidden_map),
        std::move(output_map),
        std::move(gps),
        std::move(movement),
        std::move(lidar_impl),
        std::move(algorithm),
        std::move(drone_ctrl),
        std::move(mission_ctrl),
        simulation,
        mission,
        output_map_file,
        res_status);
}

} // namespace drone_mapper
