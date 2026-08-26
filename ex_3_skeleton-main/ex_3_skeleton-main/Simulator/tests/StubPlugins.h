#pragma once

#include <Common/IMappingAlgorithm.h>
#include <Common/IMissionControl.h>
#include <Common/MappingAlgorithmFactory.h>
#include <Common/MissionControlFactory.h>

#include <filesystem>
#include <utility>

// Simulator's own tests exercise SimulationManager/SimulationRunFactoryImpl/
// SimulationRunImpl wiring in isolation, via trivial stub algorithm/mission-
// control factories -- NOT the real Algorithm/MissionControl projects. Those
// are separate, dynamically-loaded plugin projects in ex3 (built as their own
// .so files), so Simulator's own test binary should not statically link
// against their sources. The real end-to-end behavior (real algorithm + real
// mission control, reproducing the ex2 baseline scores exactly) is verified
// separately by actually running the built simulator_<ids> executable against
// the built .so files -- see project notes.

namespace simulator::test {

using namespace common;

// Signals Finished on the very first call -- enough to exercise the wiring
// (a full mission completes in 0 steps) without depending on real algorithm
// behavior.
class StubFinishedAlgorithm final : public IMappingAlgorithm {
public:
    using IMappingAlgorithm::IMappingAlgorithm;
    common::types::MappingStepCommand nextStep(const common::types::DroneState&,
                                       const common::types::LidarScanResult*) override {
        return {std::nullopt, std::nullopt, common::types::AlgorithmStatus::Finished};
    }
};

inline MappingAlgorithmFactory stubAlgorithmFactory() {
    return [](MappingAlgorithmDependencies deps) -> std::unique_ptr<IMappingAlgorithm> {
        return std::make_unique<StubFinishedAlgorithm>(std::move(deps));
    };
}

// Minimal mission-control stub: drives the (stub) algorithm in the same
// step-loop shape as the real MissionControlImpl, without verbose logging or
// its own internal DroneControlImpl (not needed here -- the algorithm signals
// Finished immediately, so no movement/scan ever happens).
class StubMissionControl final : public IMissionControl {
public:
    explicit StubMissionControl(MissionControlDependencies deps)
        : mission_(deps.mission_config),
          output_map_(deps.output_map),
          output_map_file_(std::move(deps.output_map_file)),
          mapping_algorithm_(deps.mapping_algorithm) {}

    common::types::MissionRunResult runMission() override {
        std::size_t steps = 0;
        common::types::MissionRunStatus status = common::types::MissionRunStatus::Completed;
        while (true) {
            const auto cmd = mapping_algorithm_.nextStep(common::types::DroneState{}, nullptr);
            if (cmd.status == common::types::AlgorithmStatus::Finished ||
                cmd.status == common::types::AlgorithmStatus::FinishedWithUnmappableVoxels) {
                status = common::types::MissionRunStatus::Completed;
                break;
            }
            ++steps;
            if (steps >= mission_.max_steps) {
                status = common::types::MissionRunStatus::MaxSteps;
                break;
            }
        }
        output_map_.save(output_map_file_);
        return {status, steps, {}};
    }

private:
    common::types::MissionConfigData mission_;
    IMutableMap3D& output_map_;
    std::filesystem::path output_map_file_;
    IMappingAlgorithm& mapping_algorithm_;
};

inline MissionControlFactory stubMissionControlFactory() {
    return [](MissionControlDependencies deps) -> std::unique_ptr<IMissionControl> {
        return std::make_unique<StubMissionControl>(std::move(deps));
    };
}

} // namespace simulator::test
