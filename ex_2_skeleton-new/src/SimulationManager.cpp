#include <drone_mapper/SimulationManager.h>

#include <chrono>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace drone_mapper {

namespace {

std::string utcNow() {
    const auto now = std::chrono::system_clock::now();
    const auto tt  = std::chrono::system_clock::to_time_t(now);
    std::ostringstream oss;
    oss << std::put_time(std::gmtime(&tt), "%Y-%m-%dT%H:%M:%SZ");
    return oss.str();
}

} // namespace

SimulationManager::SimulationManager(std::unique_ptr<ISimulationRunFactory> run_factory)
    : run_factory_(std::move(run_factory)) {
    if (!run_factory_)
        throw std::invalid_argument("SimulationManager requires a run factory.");
}

types::SimulationManagerReport SimulationManager::run(
    const types::SimulationCompositionData& composition,
    const std::filesystem::path& output_path) {

    std::filesystem::create_directories(output_path);
    const std::filesystem::path results_dir = output_path / "output_results";
    std::filesystem::create_directories(results_dir);

    types::SimulationManagerReport report;
    report.generated_at_utc = utcNow();
    report.metric           = "output_map_accuracy";
    report.score_range      = {0.0, 100.0};
    report.error_score      = -1;

    for (const auto& simulation : composition.simulations) {
        for (const auto& mission : composition.missions) {
            for (const auto& drone : composition.drones) {
                for (const auto& lidar : composition.lidars) {
                    try {
                        auto run = run_factory_->create(
                            simulation, mission, drone, lidar, results_dir);
                        if (!run) throw std::runtime_error("factory returned null run");
                        report.runs.push_back(run->run());
                    } catch (const std::exception& e) {
                        types::SimulationResult err_result;
                        err_result.simulation_config        = simulation;
                        err_result.mission_config           = mission;
                        err_result.resolution_request_status = types::ResolutionRequestStatus::Ignored;
                        err_result.mission_results          = {{
                            types::MissionRunStatus::Error, 0,
                            {{e.what(), e.what()}}
                        }};
                        err_result.mission_score = -1.0;
                        report.runs.push_back(std::move(err_result));
                    }
                }
            }
        }
    }

    return report;
}

} // namespace drone_mapper
