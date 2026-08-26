#pragma once

#include <Common/IMutableMap3D.h>
#include <Common/Types.h>

namespace mission_control_211781141_325049575 {

using namespace common;

class ScanResultToVoxels {
public:
    // Applies a LiDAR scan directly to the output map.
    //
    // The converter writes only scan observation states:
    // Occupied, Empty, and PotentiallyOccupied (A new state).
    static void applyToMap(IMutableMap3D& output_map,
                           const Position3D& scan_origin,
                           const Orientation& drone_heading,
                           const types::LidarScanResult& scan,
                           const types::LidarConfigData& lidar_config);
};

} // namespace mission_control_211781141_325049575
