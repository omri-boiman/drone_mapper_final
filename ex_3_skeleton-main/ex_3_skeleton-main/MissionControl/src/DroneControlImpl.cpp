#include <MissionControl/DroneControlImpl.h>
#include <MissionControl/ScanResultToVoxels.h>

#include <stdexcept>
#include <utility>

namespace mission_control_211781141_325049575 {

DroneControlImpl::DroneControlImpl(types::DroneConfigData drone,
                                   types::MissionConfigData mission,
                                   types::LidarConfigData lidar_config,
                                   ILidar& lidar,
                                   IGPS& gps,
                                   IDroneMovement& movement,
                                   IMutableMap3D& output_map,
                                   IMappingAlgorithm& mapping_algorithm)
    : drone_(std::move(drone)),
      mission_(std::move(mission)),
      lidar_config_(lidar_config),
      lidar_(lidar),
      gps_(gps),
      movement_(movement),
      output_map_(output_map),
      mapping_algorithm_(mapping_algorithm) {}

types::DroneStepResult DroneControlImpl::step() {
    const types::DroneState cur_state = state();

    const types::LidarScanResult* scan_ptr =
        latest_scan_.has_value() ? &latest_scan_.value() : nullptr;

    const types::MappingStepCommand cmd = mapping_algorithm_.nextStep(cur_state, scan_ptr);
    latest_scan_.reset();

    if (cmd.status == types::AlgorithmStatus::Finished ||
        cmd.status == types::AlgorithmStatus::FinishedWithUnmappableVoxels) {
        ++step_index_;
        return {types::DroneStepStatus::Completed};
    }

    // Execute movement first (spec: movement before scan when both provided)
    if (cmd.movement.has_value()) {
        const auto& mv = *cmd.movement;
        types::MovementResult result{true, {}};
        try {
            switch (mv.type) {
                case types::MovementCommandType::Rotate:
                    result = movement_.rotate(mv.rotation, mv.angle);
                    break;
                case types::MovementCommandType::Advance:
                    result = movement_.advance(mv.distance);
                    break;
                case types::MovementCommandType::Elevate:
                    result = movement_.elevate(mv.distance);
                    break;
                default:
                    break;
            }
        } catch (const std::exception& e) {
            // Mandatory per "Common issues and handling": a movement driver may throw
            // on a colliding movement; the drone controller catches and reports it.
            ++step_index_;
            return {types::DroneStepStatus::Error, e.what()};
        }
        ++step_index_;
        if (!result)
            return {types::DroneStepStatus::Error, result.message};
    } else {
        ++step_index_;
    }

    // Execute scan after movement
    if (cmd.scan_orientation.has_value()) {
        const types::DroneState post_move_state = state();
        const types::LidarScanResult scan = lidar_.scan(*cmd.scan_orientation);
        latest_scan_ = scan;

        ScanResultToVoxels::applyToMap(
            output_map_, post_move_state.position, post_move_state.heading, scan, lidar_config_);
    }

    return {types::DroneStepStatus::Continue};
}

types::DroneState DroneControlImpl::state() const {
    return {gps_.position(), gps_.heading(), step_index_};
}

} // namespace mission_control_211781141_325049575
