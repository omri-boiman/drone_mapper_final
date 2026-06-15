#include "io/MapIO.h"

#include <fstream>
#include <sstream>
#include <iostream>
#include <string>

namespace drone {

static MapValue IntToMapValue(int v)
{
    switch (v) {
        case  0: return MapValue::Empty;
        case  1: return MapValue::Occupied;
        case -1: return MapValue::NotMapped;
        case -2: return MapValue::BeyondBounds;
        default: return MapValue::NotMapped;
    }
}

static int MapValueToInt(MapValue v) { return static_cast<int>(v); }

static bool ParseBoundsToken(const std::string& token,
                              const std::string& key,
                              double&            out)
{
    const std::string prefix = key + "=";
    if (token.rfind(prefix, 0) != 0) return false;
    try {
        out = std::stod(token.substr(prefix.size()));
        return true;
    } catch (...) {
        return false;
    }
}

ParsedMap ParseMapFile(const std::filesystem::path& filePath, ErrorLogger& logger)
{
    ParsedMap result;

    std::ifstream file(filePath);
    if (!file.is_open()) {
        std::cerr << "Error: cannot open map file: " << filePath << "\n";
        return result;
    }

    bool foundVersion = false;
    bool foundBounds  = false;
    int  lineNum      = 0;
    std::string line;

    while (std::getline(file, line)) {
        ++lineNum;

        const auto start = line.find_first_not_of(" \t\r");
        if (start == std::string::npos || line[start] == '#') continue;
        const std::string trimmed = line.substr(start);

        if (trimmed.rfind("VERSION", 0) == 0) { foundVersion = true; continue; }
        if (trimmed == "END") break;

        if (trimmed.rfind("BOUNDS", 0) == 0) {
            std::istringstream ss(trimmed.substr(6));
            std::string token;
            double xmin{0}, xmax{0}, ymin{0}, ymax{0}, zmin{0}, zmax{0};
            while (ss >> token) {
                bool matched =
                    ParseBoundsToken(token, "xmin", xmin) ||
                    ParseBoundsToken(token, "xmax", xmax) ||
                    ParseBoundsToken(token, "ymin", ymin) ||
                    ParseBoundsToken(token, "ymax", ymax) ||
                    ParseBoundsToken(token, "hmin", zmin) ||
                    ParseBoundsToken(token, "hmax", zmax);
                if (!matched)
                    logger.Log("map file line " + std::to_string(lineNum) +
                               ": unrecognised BOUNDS token '" + token + "' — ignored");
            }
            result.bounds = {
                xmin * cm, xmax * cm,
                ymin * cm, ymax * cm,
                zmin * cm, zmax * cm
            };
            foundBounds = true;
            continue;
        }

        if (trimmed.rfind("CELL", 0) == 0) {
            std::istringstream ss(trimmed.substr(4));
            double rawX{}, rawY{}, rawZ{};
            int    rawValue{};
            if (!(ss >> rawX >> rawY >> rawZ >> rawValue)) {
                logger.Log("map file line " + std::to_string(lineNum) +
                           ": malformed CELL entry — skipped");
                continue;
            }
            result.cells.push_back({rawX * cm, rawY * cm, rawZ * cm,
                                    IntToMapValue(rawValue)});
            continue;
        }

        logger.Log("map file line " + std::to_string(lineNum) +
                   ": unrecognised token '" + trimmed.substr(0, 20) + "' — skipped");
    }

    if (!foundVersion) {
        std::cerr << "Error: map file missing VERSION header: " << filePath << "\n";
        return result;
    }
    if (!foundBounds)
        logger.Log("map file: BOUNDS line missing — defaulting to all zeros");

    result.valid = true;
    return result;
}

bool WriteMapFile(const std::filesystem::path& filePath,
                  const MapBounds&              bounds,
                  const std::vector<MapCell>&   cells)
{
    std::ofstream file(filePath);
    if (!file.is_open()) {
        std::cerr << "Error: cannot write map file: " << filePath << "\n";
        return false;
    }

    file << std::fixed;
    file.precision(2);

    const auto toCm = [](PhysicalLength v) { return v.numerical_value_in(cm); };

    file << "VERSION 1\n";
    file << "BOUNDS"
         << " xmin=" << toCm(bounds.xmin) << " xmax=" << toCm(bounds.xmax)
         << " ymin=" << toCm(bounds.ymin) << " ymax=" << toCm(bounds.ymax)
         << " hmin=" << toCm(bounds.zmin) << " hmax=" << toCm(bounds.zmax)
         << "\n";

    for (const auto& cell : cells) {
        file << "CELL "
             << toCm(cell.x) << " "
             << toCm(cell.y) << " "
             << toCm(cell.z) << " "
             << MapValueToInt(cell.value) << "\n";
    }

    file << "END\n";
    return true;
}

} // namespace drone
