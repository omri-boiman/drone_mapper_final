#pragma once

#include <drone_mapper/IMappingAlgorithm.h>

namespace drone_mapper {

class MappingAlgorithmImpl final : public IMappingAlgorithm {
public:
    //explicit MappingAlgorithmImpl(const types::DroneConfigData drone_config, const IMap3D& output_map);
    using IMappingAlgorithm::IMappingAlgorithm;
    [[nodiscard]] types::MappingStepCommand nextStep(const types::DroneState& state,
                                                     const types::LidarScanResult* latest_scan) override;
};

} // namespace drone_mapper
