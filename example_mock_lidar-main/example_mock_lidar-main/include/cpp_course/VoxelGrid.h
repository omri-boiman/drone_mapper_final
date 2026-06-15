#pragma once

#include <TinyNPY.h>

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include <cpp_course/IMap3D.h>

namespace cpp_course {

// Occupancy map backed by a row-major 3D NumPy array with shape [X, Y, Z].
// Coordinates are interpreted in centimeters and converted with floor().
class VoxelGrid final : public IMap3D {
public:
    VoxelGrid() = default;

    explicit VoxelGrid(const std::string& path);

    void load(const std::string& path);
    
    [[nodiscard]] std::size_t x_size() const noexcept;

    [[nodiscard]] std::size_t y_size() const noexcept;

    [[nodiscard]] std::size_t z_size() const noexcept;


    [[nodiscard]] int get(const Position3D& pos) const override;
private:
    NpyArray array_{};
    std::vector<std::size_t> shape_{};
    std::size_t x_size_ = 0;
    std::size_t y_size_ = 0;
    // z=height
    std::size_t z_size_ = 0;

    [[nodiscard]] std::size_t flat_index(std::size_t x, std::size_t y, std::size_t z) const noexcept;

    [[nodiscard]] int value_at_flat_index(std::size_t index) const;

    void check_bounds(std::size_t x, std::size_t y, std::size_t z) const;

    [[nodiscard]] int at(std::size_t x, std::size_t y, std::size_t z) const;

    [[nodiscard]] static std::optional<std::size_t> coordinate_to_index(XLength coordinate);
    [[nodiscard]] static std::optional<std::size_t> coordinate_to_index(YLength coordinate);
    [[nodiscard]] static std::optional<std::size_t> coordinate_to_index(ZLength coordinate);

};

} // namespace cpp_course
