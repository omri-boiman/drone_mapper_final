#include <drone_mapper/MappingAlgorithmImpl.h>

namespace drone_mapper {

// MappingAlgorithmImpl::MappingAlgorithmImpl(const types::DroneConfigData drone_config, const IMap3D& output_map)
//     : IMappingAlgorithm(drone_config, output_map) {}

types::MappingStepCommand MappingAlgorithmImpl::nextStep(const types::DroneState& state,
                                                       const types::LidarScanResult* latest_scan) {
    (void)state;
    (void)latest_scan;
    return types::MappingStepCommand{};
}

} // namespace drone_mapper
