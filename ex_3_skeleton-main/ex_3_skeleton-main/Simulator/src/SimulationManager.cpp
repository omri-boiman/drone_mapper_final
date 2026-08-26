#include <Simulator/SimulationManager.h>

#include <UserCommon/ErrorLogger.h>

#include <chrono>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace simulator {

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

    user_common_211781141_325049575::ErrorLogger error_log(results_dir / "error.log");

    types::SimulationManagerReport report;
    report.composition_file = composition.composition_file;
    report.generated_at_utc = utcNow();
    report.metric           = "output_map_accuracy";
    report.score_range      = {0.0, 100.0};
    report.error_score      = -1;

    for (const auto& [simulation, missions] : composition.simulation_mission_groups) {
        for (const auto& mission : missions) {
            for (const auto& drone : composition.drone_configs) {
                for (const auto& lidar : composition.lidar_configs) {
                    try {
                        auto run = run_factory_->create(
                            simulation, mission, drone, lidar, results_dir);
                        if (!run) throw std::runtime_error("factory returned null run");
                        auto result = run->run();
                        for (const auto& mr : result.mission_results)
                            for (const auto& err : mr.errors)
                                error_log.log(err.message);
                        report.runs.push_back(std::move(result));
                    } catch (const std::exception& e) {
                        error_log.log(e.what());
                        types::SimulationResult err_result;
                        err_result.simulation_config        = simulation;
                        err_result.mission_config           = mission;
                        err_result.resolution_request_status = types::ResolutionRequestStatus::Ignored;
                        err_result.mission_results          = {{
                            common::types::MissionRunStatus::Error, 0,
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

} // namespace simulator
