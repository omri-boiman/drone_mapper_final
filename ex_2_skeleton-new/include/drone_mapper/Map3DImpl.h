#pragma once

#include <TinyNPY.h>

#include <drone_mapper/IMutableMap3D.h>

#include <cstdint>
#include <filesystem>
#include <memory>
#include <vector>

namespace drone_mapper {

class Map3DImpl final : public IMutableMap3D {
public:
    Map3DImpl(const std::filesystem::path& path, PhysicalLength resolution);
    Map3DImpl(const std::filesystem::path& path, PhysicalLength resolution, Position3D offset);
    Map3DImpl(const types::MappingBounds& bounds, PhysicalLength resolution, Position3D offset = {});

    [[nodiscard]] types::VoxelOccupancy atVoxel(const Position3D& pos) const override;
    [[nodiscard]] types::MapConfig getMapConfig() const override;
    [[nodiscard]] bool isInBounds(const Position3D& pos) const override;
    void set(const Position3D& pos, types::VoxelOccupancy value) override;
    void save(const std::filesystem::path& path) const override;

private:
    [[nodiscard]] std::size_t flatIndex(std::size_t xi, std::size_t yi, std::size_t zi) const noexcept;
    [[nodiscard]] bool toIndex(const Position3D& pos,
                               std::size_t& xi, std::size_t& yi, std::size_t& zi) const noexcept;

    std::shared_ptr<NpyArray> map_;
    types::MapConfig config_;
    std::vector<int8_t> output_data_;
    bool is_output_map_ = false;
    std::size_t x_size_ = 0, y_size_ = 0, z_size_ = 0;
};

} // namespace drone_mapper
