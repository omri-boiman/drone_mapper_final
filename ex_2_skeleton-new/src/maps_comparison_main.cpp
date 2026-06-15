#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MapsComparison.h>
#include <drone_mapper/YamlConfigParser.h>

#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>

int main(int argc, char** argv) {
    try {
        if (argc < 3 || argc > 4) {
            std::cout << -1;
            std::cerr << "Usage: maps_comparison <origin_map> <target_map> [comparison_config=<path>]\n";
            return 1;
        }

        // Default: both maps share offset (0,0,0) and resolution inferred from NPY size
        drone_mapper::Map3DImpl origin(argv[1], 10.0 * drone_mapper::cm);
        drone_mapper::Map3DImpl target(argv[2], 10.0 * drone_mapper::cm);

        if (argc == 4) {
            const std::string arg3 = argv[3];
            if (arg3.starts_with("comparison_config=")) {
                const std::filesystem::path cfg_path = arg3.substr(18);
                const auto [orig_cfg, tgt_cfg] =
                    drone_mapper::YamlConfigParser::parseComparisonConfig(cfg_path);

                // Reload with explicit resolution and offset from comparison config
                drone_mapper::Map3DImpl origin2(argv[1],
                    orig_cfg.map_config.resolution,
                    orig_cfg.map_config.offset);
                drone_mapper::Map3DImpl target2(argv[2],
                    tgt_cfg.map_config.resolution,
                    tgt_cfg.map_config.offset);

                const auto scores =
                    drone_mapper::MapsComparison::compare(origin2, {&target2});
                std::cout << (scores.empty() ? -1.0 : scores[0]);
                return 0;
            }
        }

        const auto scores = drone_mapper::MapsComparison::compare(origin, {&target});
        std::cout << (scores.empty() ? -1.0 : scores[0]);
        return 0;

    } catch (const std::exception& e) {
        std::cout << -1;
        std::cerr << e.what() << "\n";
        return 1;
    }
}
