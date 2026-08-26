#pragma once

#include <Common/IMissionControl.h>
#include <Common/MissionControlFactory.h>
#include <MissionControl/IDroneControl.h>

#include <filesystem>
#include <fstream>
#include <memory>

namespace mission_control_211781141_325049575 {

using namespace common;

class MissionControlImpl_211781141_325049575 final : public common::IMissionControl {
public:
    explicit MissionControlImpl_211781141_325049575(common::MissionControlDependencies dependencies);

    [[nodiscard]] types::MissionRunResult runMission() override;

private:
    types::MissionConfigData mission_;
    IMutableMap3D& output_map_;
    std::filesystem::path output_map_file_;
    bool verbose_ = false;
    std::unique_ptr<mission_control::IDroneControl> drone_control_;
};

} // namespace mission_control_211781141_325049575
