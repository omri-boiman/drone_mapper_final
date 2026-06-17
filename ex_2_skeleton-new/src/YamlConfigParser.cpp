#include <drone_mapper/YamlConfigParser.h>

#include <yaml-cpp/yaml.h>

#include <stdexcept>
#include <string>

namespace drone_mapper {

// ── helpers ───────────────────────────────────────────────────────────────────

namespace {

double nodeDouble(const YAML::Node& n, double def = 0.0) {
    if (!n || !n.IsDefined()) return def;
    return n.as<double>(def);
}

std::size_t nodeSize(const YAML::Node& n, std::size_t def = 0) {
    if (!n || !n.IsDefined()) return def;
    return n.as<std::size_t>(def);
}

int nodeInt(const YAML::Node& n, int def = 0) {
    if (!n || !n.IsDefined()) return def;
    return n.as<int>(def);
}

Position3D readOffset(const YAML::Node& n) {
    return Position3D{
        nodeDouble(n["x_offset"], 0.0) * x_extent[cm],
        nodeDouble(n["y_offset"], 0.0) * y_extent[cm],
        nodeDouble(n["height_offset"], 0.0) * z_extent[cm],
    };
}

types::MappingBounds readBounds(const YAML::Node& n) {
    return types::MappingBounds{
        nodeDouble(n["x_boundary"]["min_cm"],      0.0) * x_extent[cm],
        nodeDouble(n["x_boundary"]["max_cm"],    500.0) * x_extent[cm],
        nodeDouble(n["y_boundary"]["min_cm"],      0.0) * y_extent[cm],
        nodeDouble(n["y_boundary"]["max_cm"],    500.0) * y_extent[cm],
        nodeDouble(n["height_boundary"]["min_cm"],  0.0) * z_extent[cm],
        nodeDouble(n["height_boundary"]["max_cm"],300.0) * z_extent[cm],
    };
}

} // namespace

// ── public parsers ────────────────────────────────────────────────────────────

types::DroneConfigData YamlConfigParser::parseDrone(const std::filesystem::path& path) {
    const YAML::Node doc = YAML::LoadFile(path.string());
    const YAML::Node dc  = doc["drone_config"];
    return {
        nodeDouble(dc["dimensions_cm"],  30.0) / 2.0 * cm,
        nodeDouble(dc["max_rotate_deg"], 45.0) * horizontal_angle[deg],
        nodeDouble(dc["max_advance_cm"], 50.0) * cm,
        nodeDouble(dc["max_elevate_cm"], 40.0) * cm,
    };
}

types::LidarConfigData YamlConfigParser::parseLidar(const std::filesystem::path& path) {
    const YAML::Node doc = YAML::LoadFile(path.string());
    const YAML::Node lc  = doc["lidar_config"];
    return {
        nodeDouble(lc["z_min_cm"],   20.0) * cm,
        nodeDouble(lc["z_max_cm"],  120.0) * cm,
        nodeDouble(lc["d_cm"],        2.5) * cm,
        nodeSize(  lc["fov_circles"],   5),
    };
}

types::MissionConfigData YamlConfigParser::parseMission(const std::filesystem::path& path) {
    const YAML::Node doc = YAML::LoadFile(path.string());
    const YAML::Node mc  = doc["mission_config"];
    // boundaries are parsed but ignored — bounds now come from the hidden map
    return {
        nodeSize(  mc["max_steps"],                          2400),
        nodeDouble(mc["gps_resolution_cm"],                  10.0) * cm,
        static_cast<double>(nodeInt(mc["output_mapping_resolution_factor"], 1)),
    };
}

types::SimulationConfigData YamlConfigParser::parseSingleSimulation(
    const std::filesystem::path& path) {
    const YAML::Node doc = YAML::LoadFile(path.string());
    const YAML::Node sc  = doc["simulation_config"];
    const YAML::Node dp  = sc["initial_drone_position"];
    const YAML::Node ao  = sc["map_axes_offset"];

    return {
        sc["map_filename"].as<std::string>(""),
        nodeDouble(sc["map_resolution_cm"], 10.0) * cm,
        readOffset(ao),
        Position3D{
            nodeDouble(dp["x_cm"],       0.0) * x_extent[cm],
            nodeDouble(dp["y_cm"],       0.0) * y_extent[cm],
            nodeDouble(dp["height_cm"], 150.0) * z_extent[cm],
        },
        nodeDouble(sc["initial_angle_deg"], 0.0) * horizontal_angle[deg],
    };
}

YamlConfigParser::CompositionWithPaths
YamlConfigParser::parseCompositionWithPaths(const std::filesystem::path& path) {
    const YAML::Node doc  = YAML::LoadFile(path.string());
    const YAML::Node root = doc["simulation_compositions"];
    const auto base       = path.parent_path();

    CompositionWithPaths result;
    result.data.composition_file = path;

    // Drone configs (global)
    for (const auto& entry : root["drone_configs"]) {
        const std::filesystem::path p = base / entry.as<std::string>();
        result.data.drones.push_back(parseDrone(p));
        result.drone_paths.push_back(p);
    }

    // Lidar configs (global)
    for (const auto& entry : root["lidar_configs"]) {
        const std::filesystem::path p = base / entry.as<std::string>();
        result.data.lidars.push_back(parseLidar(p));
        result.lidar_paths.push_back(p);
    }

    // Simulations + per-simulation missions
    for (const auto& sim_entry : root["simulations"]) {
        const std::filesystem::path sim_path =
            base / sim_entry["simulation_config"].as<std::string>();
        result.data.simulations.push_back(parseSingleSimulation(sim_path));
        result.sim_paths.push_back(sim_path);

        for (const auto& mc_entry : sim_entry["mission_configs"]) {
            const std::filesystem::path mp = base / mc_entry.as<std::string>();
            result.data.missions.push_back(parseMission(mp));
            result.mission_paths.push_back(mp);
        }
    }

    if (result.data.drones.empty())
        throw std::runtime_error("YamlConfigParser: no drone_configs in " + path.string());
    if (result.data.lidars.empty())
        throw std::runtime_error("YamlConfigParser: no lidar_configs in " + path.string());

    return result;
}

std::pair<types::ComparisonMapConfig, types::ComparisonMapConfig>
YamlConfigParser::parseComparisonConfig(const std::filesystem::path& path) {
    const YAML::Node doc = YAML::LoadFile(path.string());
    const YAML::Node cc  = doc["comparison_config"];

    auto parseOne = [](const YAML::Node& n) -> types::ComparisonMapConfig {
        types::MapConfig cfg;
        cfg.resolution = nodeDouble(n["map_res_cm"], 10.0) * cm;
        cfg.offset     = readOffset(n["map_offset"]);
        if (n["map_boundaries"]) {
            cfg.boundaries = readBounds(n["map_boundaries"]);
        }
        return types::ComparisonMapConfig{cfg};
    };

    return {parseOne(cc["original"]), parseOne(cc["target"])};
}

} // namespace drone_mapper
