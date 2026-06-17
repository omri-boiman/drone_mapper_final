#pragma once

#include <drone_mapper/types/MapTypes.h>
#include <drone_mapper/types/SimulationTypes.h>

#include <filesystem>
#include <vector>

namespace drone_mapper {

class YamlConfigParser {
public:
    // Internal struct carrying the skeleton composition data plus parallel file paths
    // (one path per entry in each vector, in the same order).
    // Do NOT modify the skeleton types — paths live here instead.
    struct CompositionWithPaths {
        types::SimulationCompositionData data;
        std::vector<std::filesystem::path> sim_paths;
        std::vector<std::filesystem::path> mission_paths;
        std::vector<std::filesystem::path> drone_paths;
        std::vector<std::filesystem::path> lidar_paths;
    };

    static CompositionWithPaths parseCompositionWithPaths(const std::filesystem::path& path);

    static types::SimulationConfigData parseSingleSimulation(const std::filesystem::path& path);
    static types::MissionConfigData    parseMission(const std::filesystem::path& path);
    static types::DroneConfigData      parseDrone(const std::filesystem::path& path);
    static types::LidarConfigData      parseLidar(const std::filesystem::path& path);

    // Parse comparison_config YAML; returns {origin, target} MapConfig pair
    static std::pair<types::ComparisonMapConfig, types::ComparisonMapConfig>
    parseComparisonConfig(const std::filesystem::path& path);
};

} // namespace drone_mapper
