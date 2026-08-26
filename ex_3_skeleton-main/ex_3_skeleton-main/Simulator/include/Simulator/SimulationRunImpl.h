#pragma once

#include <Common/IDroneMovement.h>
#include <Common/IGPS.h>
#include <Common/ILidar.h>
#include <Common/IMap3D.h>
#include <Common/IMappingAlgorithm.h>
#include <Common/IMissionControl.h>
#include <Common/IMutableMap3D.h>
#include <Simulator/ISimulationRun.h>

#include <filesystem>
#include <memory>

namespace simulator {

using namespace common;

class SimulationRunImpl final : public ISimulationRun {
public:
    SimulationRunImpl(std::unique_ptr<const IMap3D> hidden_map,
                      std::unique_ptr<IMutableMap3D> output_map,
                      std::unique_ptr<IGPS> gps,
                      std::unique_ptr<IDroneMovement> movement,
                      std::unique_ptr<ILidar> lidar,
                      std::unique_ptr<IMappingAlgorithm> mapping_algorithm,
                      std::unique_ptr<IMissionControl> mission_control,
                      types::SimulationConfigData simulation_config,
                      common::types::MissionConfigData mission_config,
                      std::filesystem::path output_map_file,
                      types::ResolutionRequestStatus resolution_status);

    [[nodiscard]] types::SimulationResult run() override;

private:
    // Declaration order matters for destruction order (reverse of declaration):
    // mission_control_ holds references (via its own internally-built drone
    // control) into gps_/movement_/lidar_/output_map_/mapping_algorithm_, so it
    // must be destroyed BEFORE them -- hence it is declared LAST.
    std::unique_ptr<const IMap3D> hidden_map_;
    std::unique_ptr<IMutableMap3D> output_map_;
    std::unique_ptr<IGPS> gps_;
    std::unique_ptr<IDroneMovement> movement_;
    std::unique_ptr<ILidar> lidar_;
    std::unique_ptr<IMappingAlgorithm> mapping_algorithm_;
    std::unique_ptr<IMissionControl> mission_control_;
    types::SimulationConfigData simulation_config_;
    common::types::MissionConfigData mission_config_;
    std::filesystem::path output_map_file_;
    types::ResolutionRequestStatus resolution_status_;
};

} // namespace simulator
