#pragma once

#include <Common/IDroneMovement.h>
#include <Common/IGPS.h>
#include <Common/ILidar.h>
#include <Common/IMappingAlgorithm.h>
#include <Common/IMutableMap3D.h>
#include <Common/Types.h>
#include <MissionControl/IDroneControl.h>

#include <optional>

namespace mission_control_211781141_325049575 {

using namespace common;

class DroneControlImpl final : public mission_control::IDroneControl {
public:
    DroneControlImpl(types::DroneConfigData drone,
                     types::MissionConfigData mission,
                     types::LidarConfigData lidar_config,
                     ILidar& lidar,
                     IGPS& gps,
                     IDroneMovement& movement,
                     IMutableMap3D& output_map,
                     IMappingAlgorithm& mapping_algorithm);

    [[nodiscard]] types::DroneStepResult step() override;
    [[nodiscard]] types::DroneState state() const override;

private:
    types::DroneConfigData drone_;
    types::MissionConfigData mission_;
    types::LidarConfigData lidar_config_;
    ILidar& lidar_;
    IGPS& gps_;
    IDroneMovement& movement_;
    IMutableMap3D& output_map_;
    IMappingAlgorithm& mapping_algorithm_;
    std::size_t step_index_ = 0;
    std::optional<types::LidarScanResult> latest_scan_{};
};

} // namespace mission_control_211781141_325049575
