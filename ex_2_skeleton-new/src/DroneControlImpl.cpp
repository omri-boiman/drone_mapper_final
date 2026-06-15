#include <drone_mapper/DroneControlImpl.h>
#include <drone_mapper/ScanResultToVoxels.h>

#include <utility>

namespace drone_mapper {

DroneControlImpl::DroneControlImpl(types::DroneConfigData drone,
                                   types::MissionConfigData mission,
                                   ILidar& lidar,
                                   IGPS& gps,
                                   IDroneMovement& movement,
                                   IMutableMap3D& output_map,
                                   IMappingAlgorithm& mapping_algorithm)
    : drone_(std::move(drone)),
      mission_(std::move(mission)),
      lidar_(lidar),
      gps_(gps),
      movement_(movement),
      output_map_(output_map),
      mapping_algorithm_(mapping_algorithm) {}

types::DroneStepResult DroneControlImpl::step() {
    const types::DroneState cur_state = state();

    const types::LidarScanResult scan = lidar_.scan(
        Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]});

    const auto voxels = ScanResultToVoxels::convert(
        cur_state.position, cur_state.heading, scan);

    for (const auto& v : voxels)
        output_map_.set(v.position, v.value);

    mapping_algorithm_.applyVoxelUpdates(voxels);

    const types::MovementCommand cmd =
        mapping_algorithm_.nextMove(cur_state, scan);

    if (cmd.type == types::MovementCommandType::Hover) {
        ++step_index_;
        return {types::DroneStepStatus::Completed};
    }

    types::MovementResult result{true, {}};
    switch (cmd.type) {
        case types::MovementCommandType::Rotate:
            result = movement_.rotate(cmd.rotation, cmd.angle);
            break;
        case types::MovementCommandType::Advance:
            result = movement_.advance(cmd.distance);
            break;
        case types::MovementCommandType::Elevate:
            result = movement_.elevate(cmd.distance);
            break;
        default:
            break;
    }

    ++step_index_;

    if (!result)
        return {types::DroneStepStatus::Error, result.message};

    return {types::DroneStepStatus::Continue};
}

types::DroneState DroneControlImpl::state() const {
    return {gps_.position(), gps_.heading(), step_index_};
}

} // namespace drone_mapper
