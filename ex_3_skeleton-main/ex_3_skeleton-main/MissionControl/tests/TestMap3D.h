#pragma once

#include <Common/IMutableMap3D.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <vector>

namespace mission_control_211781141_325049575::test {

using namespace common;

// Minimal in-memory IMutableMap3D test double, shared by test_drone_control.cpp
// and test_mission_control.cpp. save() actually writes a (tiny, content-free)
// file so "output map was saved" tests can check for its existence.
class TestMap3D final : public IMutableMap3D {
public:
    TestMap3D(const types::MappingBounds& bounds, PhysicalLength resolution, Position3D offset = {})
        : config_{bounds, offset, resolution} {
        const double res = resolution.force_numerical_value_in(cm);
        const double range_x = bounds.max_x.force_numerical_value_in(cm) - bounds.min_x.force_numerical_value_in(cm);
        const double range_y = bounds.max_y.force_numerical_value_in(cm) - bounds.min_y.force_numerical_value_in(cm);
        const double range_z = bounds.max_height.force_numerical_value_in(cm) - bounds.min_height.force_numerical_value_in(cm);
        x_size_ = std::max(std::size_t{1}, static_cast<std::size_t>(std::ceil(range_x / res)));
        y_size_ = std::max(std::size_t{1}, static_cast<std::size_t>(std::ceil(range_y / res)));
        z_size_ = std::max(std::size_t{1}, static_cast<std::size_t>(std::ceil(range_z / res)));
        data_.assign(x_size_ * y_size_ * z_size_, static_cast<int8_t>(types::VoxelOccupancy::Unmapped));
    }

    types::VoxelOccupancy atVoxel(const Position3D& pos) const override {
        std::size_t xi = 0, yi = 0, zi = 0;
        if (!toIndex(pos, xi, yi, zi)) return types::VoxelOccupancy::OutOfBounds;
        return static_cast<types::VoxelOccupancy>(data_[flatIndex(xi, yi, zi)]);
    }
    types::MapConfig getMapConfig() const override { return config_; }
    bool isInBounds(const Position3D& pos) const override {
        std::size_t xi = 0, yi = 0, zi = 0;
        return toIndex(pos, xi, yi, zi);
    }
    void set(const Position3D& pos, types::VoxelOccupancy value) override {
        std::size_t xi = 0, yi = 0, zi = 0;
        if (!toIndex(pos, xi, yi, zi)) return;
        data_[flatIndex(xi, yi, zi)] = static_cast<int8_t>(value);
    }
    void save(const std::filesystem::path& path) const override {
        std::filesystem::create_directories(path.parent_path());
        std::ofstream(path) << "test";
    }

private:
    std::size_t flatIndex(std::size_t xi, std::size_t yi, std::size_t zi) const noexcept {
        return xi * y_size_ * z_size_ + yi * z_size_ + zi;
    }
    bool toIndex(const Position3D& pos, std::size_t& xi, std::size_t& yi, std::size_t& zi) const noexcept {
        const double res = config_.resolution.force_numerical_value_in(cm);
        if (res <= 0.0) return false;
        const double rx = (pos.x.force_numerical_value_in(cm) - config_.boundaries.min_x.force_numerical_value_in(cm)) / res;
        const double ry = (pos.y.force_numerical_value_in(cm) - config_.boundaries.min_y.force_numerical_value_in(cm)) / res;
        const double rz = (pos.z.force_numerical_value_in(cm) - config_.boundaries.min_height.force_numerical_value_in(cm)) / res;
        if (rx < 0.0 || ry < 0.0 || rz < 0.0) return false;
        xi = static_cast<std::size_t>(std::floor(rx));
        yi = static_cast<std::size_t>(std::floor(ry));
        zi = static_cast<std::size_t>(std::floor(rz));
        return xi < x_size_ && yi < y_size_ && zi < z_size_;
    }

    types::MapConfig config_;
    std::vector<int8_t> data_;
    std::size_t x_size_ = 0, y_size_ = 0, z_size_ = 0;
};

} // namespace mission_control_211781141_325049575::test
