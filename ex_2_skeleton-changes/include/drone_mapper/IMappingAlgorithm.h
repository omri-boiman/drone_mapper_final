#pragma once

#include <drone_mapper/Types.h>

#include <vector>

namespace drone_mapper {

class IMap3D;

// **Do not change this interface.**
class IMappingAlgorithm {
public:
    virtual ~IMappingAlgorithm() = default;
    IMappingAlgorithm(const types::DroneConfigData drone_config, const IMap3D& output_map)
        : _drone_config(drone_config),
          _output_map(output_map) {}
    [[nodiscard]] virtual types::MappingStepCommand nextStep(const types::DroneState& state,
                                                             const types::LidarScanResult* latest_scan) = 0; // latest_scan can be a null pointer!
    // Signature changes 
    //virtual void applyVoxelUpdates(const std::vector<types::MappedVoxel> voxels) = 0; REMOVED in 12.6


// Added in 12.6
protected:
    const types::DroneConfigData _drone_config;
    const IMap3D& _output_map;
    
};

} // namespace drone_mapper
