#pragma once

#include <Common/types/MapTypes.h>
#include <Simulator/SimulationTypes.h>

#include <filesystem>
#include <vector>

namespace simulator {

// Not part of any staff-provided interface — implementation detail of comparison-config
// parsing, kept as its own struct rather than added to the staff's SimulationTypes.h.
struct ComparisonMapConfig {
    common::types::MapConfig map_config{};
};

class YamlConfigParser {
public:
    // Internal struct carrying the skeleton composition data plus parallel file paths.
    // Do NOT modify the skeleton types — paths live here instead.
    struct CompositionWithPaths {
        types::SimulationCompositionData data;
        // Parallel to data.simulation_mission_groups — one entry per sim.
        std::vector<std::filesystem::path> sim_paths;
        // mission_paths_per_sim[si][mi] = path for the mi-th mission of the si-th sim.
        std::vector<std::vector<std::filesystem::path>> mission_paths_per_sim;
        std::vector<std::filesystem::path> drone_paths;
        std::vector<std::filesystem::path> lidar_paths;
    };

    static CompositionWithPaths parseCompositionWithPaths(const std::filesystem::path& path);

    static types::SimulationConfigData      parseSingleSimulation(const std::filesystem::path& path);
    static common::types::MissionConfigData parseMission(const std::filesystem::path& path);
    static common::types::DroneConfigData   parseDrone(const std::filesystem::path& path);
    static common::types::LidarConfigData   parseLidar(const std::filesystem::path& path);

    // Parse comparison_config YAML; returns {origin, target} MapConfig pair
    static std::pair<ComparisonMapConfig, ComparisonMapConfig>
    parseComparisonConfig(const std::filesystem::path& path);
};

} // namespace simulator
