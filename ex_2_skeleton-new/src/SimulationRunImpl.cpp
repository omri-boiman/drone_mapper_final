#include <drone_mapper/SimulationRunImpl.h>

#include <drone_mapper/MapsComparison.h>

#include <stdexcept>
#include <utility>
#include <vector>

namespace drone_mapper {

SimulationRunImpl::SimulationRunImpl(std::unique_ptr<const IMap3D> hidden_map,
                                     std::unique_ptr<IMutableMap3D> output_map,
                                     std::unique_ptr<IGPS> gps,
                                     std::unique_ptr<IDroneMovement> movement,
                                     std::unique_ptr<ILidar> lidar,
                                     std::unique_ptr<IMappingAlgorithm> mapping_algorithm,
                                     std::unique_ptr<IDroneControl> drone_control,
                                     std::unique_ptr<IMissionControl> mission_control,
                                     types::SimulationConfigData simulation_config,
                                     types::MissionConfigData mission_config,
                                     std::filesystem::path output_map_file,
                                     types::ResolutionRequestStatus resolution_status)
    : hidden_map_(std::move(hidden_map)),
      output_map_(std::move(output_map)),
      gps_(std::move(gps)),
      movement_(std::move(movement)),
      lidar_(std::move(lidar)),
      mapping_algorithm_(std::move(mapping_algorithm)),
      drone_control_(std::move(drone_control)),
      mission_control_(std::move(mission_control)),
      simulation_config_(std::move(simulation_config)),
      mission_config_(std::move(mission_config)),
      output_map_file_(std::move(output_map_file)),
      resolution_status_(resolution_status) {
    if (!hidden_map_ || !output_map_ || !gps_ || !movement_ ||
        !lidar_ || !mapping_algorithm_ || !drone_control_ || !mission_control_) {
        throw std::invalid_argument("SimulationRunImpl requires injected dependencies.");
    }
}

types::SimulationResult SimulationRunImpl::run() {
    types::MissionRunResult mission_result = mission_control_->runMission();

    double score = -1.0;
    if (mission_result.status != types::MissionRunStatus::Error) {
        auto scores = MapsComparison::compare(*hidden_map_, {output_map_.get()});
        score = scores.empty() ? -1.0 : scores[0];
    }

    return types::SimulationResult{
        simulation_config_,
        mission_config_,
        resolution_status_,
        {mission_result},
        output_map_file_,
        output_map_->getMapConfig(),
        score,
    };
}

} // namespace drone_mapper
