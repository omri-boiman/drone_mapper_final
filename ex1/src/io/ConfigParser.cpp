#include "io/ConfigParser.h"

#include <fstream>
#include <sstream>
#include <iostream>
#include <string>
#include <algorithm>

namespace drone {

static std::string Trim(const std::string& s)
{
    const auto a = s.find_first_not_of(" \t\r\n");
    if (a == std::string::npos) return {};
    const auto b = s.find_last_not_of(" \t\r\n");
    return s.substr(a, b - a + 1);
}

using KVMap = std::vector<std::pair<std::string, std::string>>;

static KVMap LoadKV(const std::filesystem::path& filePath,
                    ErrorLogger& logger,
                    bool& opened)
{
    KVMap result;
    std::ifstream file(filePath);
    if (!file.is_open()) {
        std::cerr << "Error: cannot open config file: " << filePath << "\n";
        opened = false;
        return result;
    }
    opened = true;

    int lineNum = 0;
    std::string line;
    while (std::getline(file, line)) {
        ++lineNum;
        const std::string t = Trim(line);
        if (t.empty() || t[0] == '#') continue;

        const auto eq = t.find('=');
        if (eq == std::string::npos) {
            logger.Log(filePath.filename().string() + " line " +
                       std::to_string(lineNum) + ": no '=' found — skipped");
            continue;
        }
        result.emplace_back(Trim(t.substr(0, eq)), Trim(t.substr(eq + 1)));
    }
    return result;
}

static std::string Find(const KVMap& kv, const std::string& key)
{
    for (const auto& [k, v] : kv) {
        if (k == key) return v;
    }
    return {};
}

static double GetDouble(const KVMap& kv, const std::string& key,
                        double defaultVal, const std::string& file,
                        ErrorLogger& logger)
{
    const std::string val = Find(kv, key);
    if (val.empty()) {
        logger.Log(file + ": missing key '" + key +
                   "' — using default " + std::to_string(defaultVal));
        return defaultVal;
    }
    try {
        return std::stod(val);
    } catch (...) {
        logger.Log(file + ": bad value for '" + key + "' ('" + val +
                   "') — using default " + std::to_string(defaultVal));
        return defaultVal;
    }
}

static PhysicalLength GetCm(const KVMap& kv, const std::string& key,
                             PhysicalLength defaultVal, const std::string& file,
                             ErrorLogger& logger)
{
    const double raw = GetDouble(kv, key, defaultVal.numerical_value_in(cm), file, logger);
    return raw * cm;
}

static HorizontalAngle GetDeg(const KVMap& kv, const std::string& key,
                               HorizontalAngle defaultVal, const std::string& file,
                               ErrorLogger& logger)
{
    const double raw = GetDouble(kv, key, defaultVal.numerical_value_in(deg), file, logger);
    return raw * deg;
}

static std::size_t GetSizeT(const KVMap& kv, const std::string& key,
                             std::size_t defaultVal, const std::string& file,
                             ErrorLogger& logger)
{
    const std::string val = Find(kv, key);
    if (val.empty()) {
        logger.Log(file + ": missing key '" + key +
                   "' — using default " + std::to_string(defaultVal));
        return defaultVal;
    }
    try {
        const long long v = std::stoll(val);
        if (v < 0) throw std::range_error("negative");
        return static_cast<std::size_t>(v);
    } catch (...) {
        logger.Log(file + ": bad value for '" + key + "' ('" + val +
                   "') — using default " + std::to_string(defaultVal));
        return defaultVal;
    }
}

static std::vector<std::pair<double,double>>
ParsePolygon(const std::string& raw, const std::string& file, ErrorLogger& logger)
{
    std::vector<std::pair<double,double>> poly;
    if (raw.empty()) {
        logger.Log(file + ": missing key 'boundary_polygon' — polygon will be empty");
        return poly;
    }

    std::string s = raw;
    s.erase(std::remove(s.begin(), s.end(), ' '), s.end());

    std::istringstream ss(s);
    std::string token;
    int idx = 0;
    while (std::getline(ss, token, ')')) {
        ++idx;
        if (token.empty()) continue;
        if (!token.empty() && token.front() == ',') token = token.substr(1);
        if (!token.empty() && token.front() == '(') token = token.substr(1);

        const auto comma = token.find(',');
        if (comma == std::string::npos) {
            logger.Log(file + ": bad polygon vertex #" + std::to_string(idx) +
                       " '" + token + "' — skipped");
            continue;
        }
        try {
            poly.emplace_back(std::stod(token.substr(0, comma)),
                              std::stod(token.substr(comma + 1)));
        } catch (...) {
            logger.Log(file + ": bad polygon vertex #" + std::to_string(idx) +
                       " '" + token + "' — skipped");
        }
    }

    if (poly.size() < 3) {
        logger.Log(file + ": boundary_polygon has fewer than 3 vertices — "
                          "mapping boundaries will be unreliable");
    }
    return poly;
}

bool ParseDroneConfig(const std::filesystem::path& filePath,
                      DroneConfig&                 out,
                      ErrorLogger&                 logger)
{
    bool opened = false;
    const KVMap kv = LoadKV(filePath, logger, opened);
    if (!opened) return false;

    const std::string f = filePath.filename().string();

    out.minPassWidth       = GetCm (kv, "min_pass_width_cm",        out.minPassWidth,       f, logger);
    out.minPassLength      = GetCm (kv, "min_pass_length_cm",       out.minPassLength,      f, logger);
    out.minPassHeight      = GetCm (kv, "min_pass_height_cm",       out.minPassHeight,      f, logger);
    out.lidarBeamMin       = GetCm (kv, "lidar_beam_min_cm",        out.lidarBeamMin,       f, logger);
    out.lidarBeamMax       = GetCm (kv, "lidar_beam_max_cm",        out.lidarBeamMax,       f, logger);
    out.lidarCircleSpacing = GetCm (kv, "lidar_circle_spacing_cm",  out.lidarCircleSpacing, f, logger);
    out.lidarFovCircles    = GetSizeT(kv, "lidar_fov_circles",      out.lidarFovCircles,    f, logger);
    out.maxRotate          = GetDeg(kv, "max_rotate_deg",           out.maxRotate,          f, logger);
    out.maxAdvance         = GetCm (kv, "max_advance_cm",           out.maxAdvance,         f, logger);
    out.maxElevate         = GetCm (kv, "max_elevate_cm",           out.maxElevate,         f, logger);

    return true;
}

bool ParseMissionConfig(const std::filesystem::path& filePath,
                        MissionConfig&               out,
                        ErrorLogger&                 logger)
{
    bool opened = false;
    const KVMap kv = LoadKV(filePath, logger, opened);
    if (!opened) return false;

    const std::string f = filePath.filename().string();

    out.boundaryPolygon = ParsePolygon(Find(kv, "boundary_polygon"), f, logger);
    out.minHeight       = ZLength{GetDouble(kv, "min_height_cm",  out.minHeight.numerical_value_in(cm),  f, logger) * cm};
    out.maxHeight       = ZLength{GetDouble(kv, "max_height_cm",  out.maxHeight.numerical_value_in(cm),  f, logger) * cm};
    out.outputResXYCm   = GetDouble(kv, "output_resolution_xy_cm", out.outputResXYCm, f, logger);
    out.outputResHCm    = GetDouble(kv, "output_resolution_h_cm",  out.outputResHCm,  f, logger);
    out.startX          = XLength{GetDouble(kv, "start_x_cm",      out.startX.numerical_value_in(cm),      f, logger) * cm};
    out.startY          = YLength{GetDouble(kv, "start_y_cm",      out.startY.numerical_value_in(cm),      f, logger) * cm};
    out.startHeight     = ZLength{GetDouble(kv, "start_height_cm", out.startHeight.numerical_value_in(cm), f, logger) * cm};

    return true;
}

} // namespace drone
