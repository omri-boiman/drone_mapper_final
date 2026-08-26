#pragma once

#include <Common/MappingAlgorithmFactory.h>
#include <Common/MissionControlFactory.h>
#include <Simulator/ISimulationRunFactory.h>

namespace simulator {

// Owns the two already-loaded plugin factories (retrieved once from the dlopen'd
// .so files) and constructs a fresh algorithm/mission-control instance per run,
// per spec: "Creating an Algorithm instance / MissionControl instance from their
// factories should be cheap. Do not cache instances, just recreate them."
class SimulationRunFactoryImpl final : public ISimulationRunFactory {
public:
    SimulationRunFactoryImpl(common::MappingAlgorithmFactory algorithm_factory,
                             common::MissionControlFactory mission_control_factory,
                             bool verbose = false);

    [[nodiscard]] std::unique_ptr<ISimulationRun>
    create(const types::SimulationConfigData& simulation,
           const common::types::MissionConfigData& mission,
           const common::types::DroneConfigData& drone,
           const common::types::LidarConfigData& lidar,
           const std::filesystem::path& output_path) override;

private:
    common::MappingAlgorithmFactory algorithm_factory_;
    common::MissionControlFactory mission_control_factory_;
    bool verbose_ = false;
};

} // namespace simulator
