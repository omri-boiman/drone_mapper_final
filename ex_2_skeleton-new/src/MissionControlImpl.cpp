#include <drone_mapper/MissionControlImpl.h>

#include <utility>

namespace drone_mapper {

MissionControlImpl::MissionControlImpl(types::MissionConfigData mission,
                                       types::DroneConfigData drone,
                                       const IMap3D& hidden_map,
                                       IMutableMap3D& output_map,
                                       IDroneControl& drone_control,
                                       std::filesystem::path output_map_file)
    : mission_(std::move(mission)),
      drone_(std::move(drone)),
      hidden_map_(hidden_map),
      output_map_(output_map),
      drone_control_(drone_control),
      output_map_file_(std::move(output_map_file)) {}

types::MissionRunResult MissionControlImpl::runMission() {
    std::size_t steps = 0;
    types::MissionRunStatus status = types::MissionRunStatus::Completed;
    std::vector<types::ErrorRef> errors;

    while (true) {
        const types::DroneStepResult result = drone_control_.step();

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

    return {status, steps, std::move(errors)};
}

} // namespace drone_mapper
