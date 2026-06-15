#include <filesystem>
#include <iomanip>
#include <iostream>
#include <memory>

#include "types/Units.h"
#include "io/ErrorLogger.h"
#include "io/ConfigParser.h"
#include "io/MapIO.h"
#include "io/Scorer.h"
#include "simulation/SimulationState.h"
#include "simulation/CellMap.h"
#include "simulation/MockPositionSensor.h"
#include "simulation/MockMovementDriver.h"
#include "simulation/MockLidarSensor.h"
#include "drone/BuildingMapImpl.h"
#include "drone/Drone.h"
#include "drone/MappingAlgorithm.h"

static drone::LidarConfig MakeLidarConfig(const drone::DroneConfig& dc)
{
    return {dc.lidarBeamMin, dc.lidarBeamMax, dc.lidarCircleSpacing, dc.lidarFovCircles};
}

int main(int argc, char* argv[])
{
    const std::filesystem::path ioPath =
        (argc >= 2) ? std::filesystem::path(argv[1])
                    : std::filesystem::current_path();

    drone::ErrorLogger logger;

    drone::DroneConfig droneConfig;
    const bool droneConfigOk =
        drone::ParseDroneConfig(ioPath / "drone_config.txt", droneConfig, logger);
    if (!droneConfigOk) {
        std::cerr << "Fatal: could not open drone_config.txt in " << ioPath << "\n";
        logger.Flush(ioPath);
        return 1;
    }

    drone::MissionConfig missionConfig;
    const bool missionConfigOk =
        drone::ParseMissionConfig(ioPath / "mission_config.txt", missionConfig, logger);
    if (!missionConfigOk) {
        std::cerr << "Fatal: could not open mission_config.txt in " << ioPath << "\n";
        logger.Flush(ioPath);
        return 1;
    }

    if (missionConfig.outputResXYCm != 1.0 || missionConfig.outputResHCm != 1.0) {
        std::cerr << "Fatal: unsupported output resolution ("
                  << missionConfig.outputResXYCm << " cm XY, "
                  << missionConfig.outputResHCm  << " cm H). "
                  << "Only 1.0 cm resolution is supported.\n";
        return 1;
    }

    const drone::ParsedMap parsedMap =
        drone::ParseMapFile(ioPath / "map_input.txt", logger);
    if (!parsedMap.valid) {
        std::cerr << "Fatal: could not parse map_input.txt in " << ioPath << "\n";
        logger.Flush(ioPath);
        return 1;
    }

    if (logger.HasErrors()) {
        logger.Flush(ioPath);
    }

    // Phase 6: wire simulation objects
    auto state = std::make_shared<drone::SimulationState>();
    state->position    = { missionConfig.startX, missionConfig.startY, missionConfig.startHeight };
    state->orientation = {};

    drone::CellMap            cellMap(parsedMap);
    drone::MockPositionSensor posSensor(state);
    drone::MockMovementDriver driver(state, droneConfig, cellMap);
    drone::MockLidarSensor    lidar(MakeLidarConfig(droneConfig), cellMap, posSensor);
    drone::BuildingMapImpl    buildingMap(missionConfig);
    drone::Drone              theDrone(lidar, posSensor, driver, buildingMap);

    std::cout << "Starting mapping mission...\n";
    std::cout << "  Start pos: ("
              << missionConfig.startX.numerical_value_in(drone::cm)      << ", "
              << missionConfig.startY.numerical_value_in(drone::cm)      << ", "
              << missionConfig.startHeight.numerical_value_in(drone::cm) << ") cm\n";
    std::cout << "  Height range: "
              << missionConfig.minHeight.numerical_value_in(drone::cm)
              << " - "
              << missionConfig.maxHeight.numerical_value_in(drone::cm) << " cm\n";
    std::cout << "  Ground-truth cells: " << parsedMap.cells.size() << "\n";

    drone::MappingAlgorithm algo(theDrone, &droneConfig, &missionConfig);
    algo.Run();

    // Collect occupied cells only and write output
    std::vector<drone::MapCell> allCells = buildingMap.GetAllCells();
    std::vector<drone::MapCell> cells;
    cells.reserve(allCells.size());
    for (const auto& c : allCells)
        if (c.value == drone::MapValue::Occupied)
            cells.push_back(c);

    const drone::MapBounds bounds{
        parsedMap.bounds.xmin, parsedMap.bounds.xmax,
        parsedMap.bounds.ymin, parsedMap.bounds.ymax,
        parsedMap.bounds.zmin, parsedMap.bounds.zmax
    };

    const std::filesystem::path outPath = ioPath / "map_output.txt";
    if (!drone::WriteMapFile(outPath, bounds, cells)) {
        std::cerr << "Error: failed to write map_output.txt\n";
        return 1;
    }
    std::cout << "Output written to " << outPath << "\n";
    std::cout << "  Mapped cells: " << cells.size() << "\n";

    const double score = drone::ComputeF1Score(
        cells, parsedMap.cells,
        missionConfig.outputResXYCm, missionConfig.outputResHCm);
    std::cout << std::fixed << std::setprecision(1);
    std::cout << "  Score: " << score << " / 100\n";

    return 0;
}
