#pragma once

#include <Simulator/ISimulation.h>
#include <Simulator/ISimulationRunFactory.h>

#include <memory>

namespace simulator {

class SimulationManager final : public ISimulation {
public:
    explicit SimulationManager(std::unique_ptr<ISimulationRunFactory> run_factory);

    [[nodiscard]] types::SimulationManagerReport run(const types::SimulationCompositionData& composition,
                                              const std::filesystem::path& output_path) override;

private:
    std::unique_ptr<ISimulationRunFactory> run_factory_;
};

} // namespace simulator
