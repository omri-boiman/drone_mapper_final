#pragma once

#include <Simulator/SimulationTypes.h>
#include <Simulator/YamlConfigParser.h>

#include <filesystem>

namespace simulator {

class ScoreReportWriter {
public:
    // `filename` lets comparative/competitive mode give each plugin's report a
    // distinct name (e.g. "simulation_output_manager1.so.yaml") inside the same
    // shared output folder, per spec point 7 of both modes.
    static void write(const types::SimulationManagerReport& report,
                      const YamlConfigParser::CompositionWithPaths& paths,
                      const std::filesystem::path& output_path,
                      const std::string& filename = "simulation_output.yaml");
};

} // namespace simulator
