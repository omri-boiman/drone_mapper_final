#include <Simulator/SimulationRunFactoryImpl.h>

#include <Simulator/Map3DImpl.h>
#include <Simulator/MockGPS.h>
#include <Simulator/MockLidar.h>
#include <Simulator/MockMovement.h>
#include <Simulator/SimulationRunImpl.h>

#include <filesystem>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace simulator {

using namespace common;

namespace {

std::pair<PhysicalLength, types::ResolutionRequestStatus>
resolveOutputResolution(const common::types::MissionConfigData& mission) {
    const double factor = mission.output_mapping_resolution_factor;
    if (factor < 1.0) {
        return {mission.gps_resolution, types::ResolutionRequestStatus::IgnoredTooSmall};
    }
    return {mission.gps_resolution * factor, types::ResolutionRequestStatus::Accepted};
}

} // namespace

SimulationRunFactoryImpl::SimulationRunFactoryImpl(
    common::MappingAlgorithmFactory algorithm_factory,
    common::MissionControlFactory mission_control_factory,
    bool verbose)
    : algorithm_factory_(std::move(algorithm_factory)),
      mission_control_factory_(std::move(mission_control_factory)),
      verbose_(verbose) {}

std::unique_ptr<ISimulationRun>
SimulationRunFactoryImpl::create(const types::SimulationConfigData& simulation,
                                 const common::types::MissionConfigData& mission,
                                 const common::types::DroneConfigData& drone,
                                 const common::types::LidarConfigData& lidar,
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
        Orientation{simulation.initial_angle, 0.0 * altitude_angle[deg]},
        mission.gps_resolution);
    auto movement   = std::make_unique<MockMovement>(*gps, *hidden_map, drone);
    auto lidar_impl = std::make_unique<MockLidar>(lidar, *hidden_map, *gps);

    // Algorithm gets full configs and output map; derives bounds from output_map.getMapConfig()
    auto algorithm = algorithm_factory_(
        common::MappingAlgorithmDependencies{mission, lidar, drone, *output_map});

    // Unique output filename per (sim × mission × drone × lidar) combo
    std::filesystem::create_directories(output_path);
    const std::string fname =
        simulation.map_filename.stem().string() + "_" +
        std::to_string(static_cast<int>(
            mission.gps_resolution.force_numerical_value_in(cm))) + "cm_" +
        std::to_string(static_cast<int>(
            drone.radius.force_numerical_value_in(cm))) + "r_" +
        std::to_string(static_cast<int>(
            lidar.z_max.force_numerical_value_in(cm))) + "lz.npy";
    const std::filesystem::path output_map_file = output_path / fname;

    // MissionControl builds its own internal drone-control from these raw
    // dependencies -- the Simulator never sees IDroneControl at all.
    auto mission_control = mission_control_factory_(
        common::MissionControlDependencies{
            mission, drone, *lidar_impl, *gps, *movement, *output_map, *algorithm,
            output_map_file, verbose_});

    return std::make_unique<SimulationRunImpl>(
        std::move(hidden_map),
        std::move(output_map),
        std::move(gps),
        std::move(movement),
        std::move(lidar_impl),
        std::move(algorithm),
        std::move(mission_control),
        simulation,
        mission,
        output_map_file,
        res_status);
}

} // namespace simulator
