#pragma once

#include <filesystem>
#include <vector>
#include <utility>
#include "io/ErrorLogger.h"
#include "types/Units.h"

namespace drone {

struct DroneConfig {
    PhysicalLength minPassWidth  {60.0  * cm};
    PhysicalLength minPassLength {60.0  * cm};
    PhysicalLength minPassHeight {120.0 * cm};

    // Professor's LidarConfig fields
    PhysicalLength lidarBeamMin      {20.0   * cm};
    PhysicalLength lidarBeamMax      {1000.0 * cm};
    PhysicalLength lidarCircleSpacing{10.0   * cm};
    std::size_t    lidarFovCircles   {3};

    HorizontalAngle maxRotate  {45.0 * deg};
    PhysicalLength  maxAdvance {50.0 * cm};
    PhysicalLength  maxElevate {30.0 * cm};
};

struct MissionConfig {
    std::vector<std::pair<double, double>> boundaryPolygon;

    ZLength minHeight {0.0   * cm};
    ZLength maxHeight {300.0 * cm};

    double outputResXYCm {1.0};
    double outputResHCm  {1.0};

    XLength startX      {0.0   * cm};
    YLength startY      {0.0   * cm};
    ZLength startHeight {150.0 * cm};
};

bool ParseDroneConfig(const std::filesystem::path& filePath,
                      DroneConfig&                 out,
                      ErrorLogger&                 logger);

bool ParseMissionConfig(const std::filesystem::path& filePath,
                        MissionConfig&               out,
                        ErrorLogger&                 logger);

} // namespace drone
