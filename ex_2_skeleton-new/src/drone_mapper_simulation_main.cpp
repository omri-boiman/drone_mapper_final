#include <drone_mapper/ScoreReportWriter.h>
#include <drone_mapper/SimulationManager.h>
#include <drone_mapper/SimulationRunFactoryImpl.h>
#include <drone_mapper/YamlConfigParser.h>

#include <filesystem>
#include <iostream>
#include <memory>
#include <stdexcept>

namespace {

std::filesystem::path resolveSimYaml(const char* arg) {
    const std::filesystem::path p(arg);
    if (p.is_absolute()) return p;
    return std::filesystem::current_path() / p;
}

} // namespace

int main(int argc, char** argv) {
    try {
        const std::filesystem::path yaml_path =
            (argc >= 2) ? resolveSimYaml(argv[1])
                        : (std::filesystem::current_path() / "simulation.yaml");

        const std::filesystem::path output_path =
            (argc >= 3) ? std::filesystem::path{argv[2]}
                        : std::filesystem::current_path();

        const auto cwp = drone_mapper::YamlConfigParser::parseCompositionWithPaths(yaml_path);

        auto run_factory = std::make_unique<drone_mapper::SimulationRunFactoryImpl>();
        drone_mapper::SimulationManager manager{std::move(run_factory)};

        const auto report = manager.run(cwp.data, output_path);

        drone_mapper::ScoreReportWriter::write(
            report, cwp.data.composition_file, cwp, output_path);

        std::cout << "Completed " << report.runs.size()
                  << " run(s). Results in " << output_path << "\n";
        return 0;

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}
