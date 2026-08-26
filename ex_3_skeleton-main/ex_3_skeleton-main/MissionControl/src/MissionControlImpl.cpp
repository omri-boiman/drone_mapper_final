#include <MissionControl/MissionControlImpl.h>

#include <Common/MissionControlRegistration.h>
#include <MissionControl/DroneControlImpl.h>

#include <chrono>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <utility>

namespace mission_control_211781141_325049575 {

namespace {

std::string statusStr(types::DroneStepStatus s) {
    switch (s) {
        case types::DroneStepStatus::Continue:  return "Continue";
        case types::DroneStepStatus::Completed: return "Completed";
        case types::DroneStepStatus::Error:     return "Error";
    }
    return "Unknown";
}

std::string missionStatusStr(types::MissionRunStatus s) {
    switch (s) {
        case types::MissionRunStatus::Completed: return "Completed";
        case types::MissionRunStatus::MaxSteps:  return "MaxSteps";
        case types::MissionRunStatus::Error:     return "Error";
    }
    return "Unknown";
}

std::string utcNow() {
    const auto now = std::chrono::system_clock::now();
    const auto tt  = std::chrono::system_clock::to_time_t(now);
    std::ostringstream oss;
    oss << std::put_time(std::gmtime(&tt), "%Y-%m-%dT%H:%M:%SZ");
    return oss.str();
}

double distanceCm(const Position3D& a, const Position3D& b) {
    const double dx = a.x.force_numerical_value_in(cm) - b.x.force_numerical_value_in(cm);
    const double dy = a.y.force_numerical_value_in(cm) - b.y.force_numerical_value_in(cm);
    const double dz = a.z.force_numerical_value_in(cm) - b.z.force_numerical_value_in(cm);
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

} // namespace

MissionControlImpl_211781141_325049575::MissionControlImpl_211781141_325049575(
    common::MissionControlDependencies dependencies)
    : mission_(dependencies.mission_config),
      output_map_(dependencies.output_map),
      output_map_file_(std::move(dependencies.output_map_file)),
      verbose_(dependencies.verbose),
      drone_control_(std::make_unique<DroneControlImpl>(
          dependencies.drone_config,
          dependencies.mission_config,
          dependencies.lidar.config(),
          dependencies.lidar,
          dependencies.gps,
          dependencies.movement,
          dependencies.output_map,
          dependencies.mapping_algorithm)) {}

types::MissionRunResult MissionControlImpl_211781141_325049575::runMission() {
    std::ofstream verbose_log;
    const auto mission_start = std::chrono::steady_clock::now();
    if (verbose_) {
        auto path = output_map_file_;
        path.replace_extension();
        path += "_verbose.log";
        std::filesystem::create_directories(path.parent_path());
        verbose_log.open(path, std::ios::out | std::ios::trunc);
        if (verbose_log.is_open()) {
            verbose_log << "=== mission start " << utcNow() << " ===\n"
                        << "max_steps=" << mission_.max_steps
                        << " gps_resolution_cm=" << mission_.gps_resolution.force_numerical_value_in(cm)
                        << " output_map_file=" << output_map_file_.string() << "\n\n";
        }
    }

    std::size_t steps = 0;
    types::MissionRunStatus status = types::MissionRunStatus::Completed;
    std::vector<types::ErrorRef> errors;
    double total_distance_cm = 0.0;
    Position3D prev_pos = drone_control_->state().position;

    while (true) {
        const types::DroneStepResult result = drone_control_->step();

        if (verbose_log.is_open()) {
            const auto s = drone_control_->state();
            const double step_distance_cm = distanceCm(s.position, prev_pos);
            total_distance_cm += step_distance_cm;
            const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - mission_start).count();

            verbose_log << "step=" << s.step_index
                        << " pos=(" << s.position.x.force_numerical_value_in(cm) << ","
                                    << s.position.y.force_numerical_value_in(cm) << ","
                                    << s.position.z.force_numerical_value_in(cm) << ")"
                        << " heading=" << s.heading.horizontal.force_numerical_value_in(deg)
                        << " moved_cm=" << step_distance_cm
                        << " elapsed_ms=" << elapsed_ms
                        << " status=" << statusStr(result.status);
            if (!result.message.empty()) verbose_log << " message=" << result.message;
            verbose_log << '\n';
            prev_pos = s.position;
        }

        if (result.status == types::DroneStepStatus::Completed) {
            status = types::MissionRunStatus::Completed;
            break;
        }
        if (result.status == types::DroneStepStatus::Error) {
            status = types::MissionRunStatus::Error;
            errors.push_back({result.message, result.message});
            break;
        }

        ++steps;
        if (steps >= mission_.max_steps) {
            status = types::MissionRunStatus::MaxSteps;
            break;
        }
    }

    output_map_.save(output_map_file_);

    if (verbose_log.is_open()) {
        const auto total_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - mission_start).count();
        verbose_log << "\n=== mission end " << utcNow() << " ===\n"
                    << "final_status=" << missionStatusStr(status)
                    << " total_steps=" << steps
                    << " total_distance_cm=" << total_distance_cm
                    << " total_elapsed_ms=" << total_ms
                    << " error_count=" << errors.size() << "\n";
    }

    return {status, steps, std::move(errors)};
}

REGISTER_MISSION_CONTROL(MissionControlImpl_211781141_325049575);

} // namespace mission_control_211781141_325049575
