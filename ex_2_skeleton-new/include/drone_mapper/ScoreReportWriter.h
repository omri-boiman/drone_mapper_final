#pragma once

#include <drone_mapper/YamlConfigParser.h>
#include <drone_mapper/types/SimulationTypes.h>

#include <filesystem>

namespace drone_mapper {

class ScoreReportWriter {
public:
    static void write(const types::SimulationManagerReport& report,
                      const std::filesystem::path& composition_file,
                      const YamlConfigParser::CompositionWithPaths& paths,
                      const std::filesystem::path& output_path);
};

} // namespace drone_mapper
