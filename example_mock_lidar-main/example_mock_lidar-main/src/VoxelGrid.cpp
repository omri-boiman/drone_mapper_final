#include <cpp_course/VoxelGrid.h>

#include <cstdint>
#include <typeinfo>
#include <cmath>
#include <stdexcept>
namespace cpp_course {

namespace {

// Assuming the map is given in cm
// Map coordinates are stored at 1 voxel per centimeter.
[[nodiscard]] std::optional<std::size_t> centimeters_to_index(double centimeters) {
    if (centimeters < 0.0) {
        return std::nullopt;
    }

    return static_cast<std::size_t>(std::floor(centimeters));
}

} // namespace

VoxelGrid::VoxelGrid(const std::string& path) {
    load(path);
}

void VoxelGrid::load(const std::string& path) {
    const char* err = array_.LoadNPY(path.c_str());
    if (err != nullptr) {
        throw std::runtime_error(std::string("Failed to load NPY file: ") + err);
    }

    if (array_.Shape().size() != 3) {
        throw std::runtime_error("Expected a 3D array with shape [X, Y, Z].");
    }

    if (array_.ColMajor()) {
        throw std::runtime_error("Expected a row-major NumPy array, got column-major.");
    }

    if (array_.ValueType() != typeid(int) && array_.ValueType() != typeid(std::uint8_t)) {
        throw std::runtime_error("Expected dtype int or uint8 in the NPY file.");
    }

    shape_ = array_.Shape();
    x_size_ = shape_[0];
    y_size_ = shape_[1];
    z_size_ = shape_[2];
}


std::size_t VoxelGrid::x_size() const noexcept {
    return x_size_;
}
std::size_t VoxelGrid::y_size() const noexcept {
    return y_size_;
}
std::size_t VoxelGrid::z_size() const noexcept {
    return z_size_;
}

int VoxelGrid::get(const Position3D& pos) const {
    const auto x = coordinate_to_index(pos.x);
    const auto y = coordinate_to_index(pos.y);
    const auto z = coordinate_to_index(pos.z);

    if (!x || !y || !z || *x >= x_size_ || *y >= y_size_ || *z >= z_size_) {
        return 0;
    }

    return at(*x, *y, *z);
}

std::size_t VoxelGrid::flat_index(std::size_t x, std::size_t y, std::size_t z) const noexcept {
    return x * z_size_ * y_size_ + y * z_size_ + z; // rowmaster representation
}

int VoxelGrid::value_at_flat_index(std::size_t index) const {
    if (array_.ValueType() == typeid(int)) {
        const int* values = array_.Data<int>();
        if (values == nullptr) {
            throw std::runtime_error("TinyNPY returned a null int data pointer.");
        }

        return values[index];
    }

    if (array_.ValueType() == typeid(std::uint8_t)) {
        const std::uint8_t* values = array_.Data<std::uint8_t>();
        if (values == nullptr) {
            throw std::runtime_error("TinyNPY returned a null uint8 data pointer.");
        }

        return static_cast<int>(values[index]);
    }

    throw std::runtime_error("VoxelGrid loaded an unsupported dtype.");
}

void VoxelGrid::check_bounds(std::size_t x, std::size_t y, std::size_t z) const {
    if (x >= x_size_ || y >= y_size_ || z >= z_size_) {
        throw std::out_of_range("VoxelGrid index out of bounds.");
    }
}

int VoxelGrid::at(std::size_t x, std::size_t y, std::size_t z) const {
    check_bounds(x, y, z);
    return value_at_flat_index(flat_index(x, y, z));
}

// Due to strong typing, we have 3 overloads - how would you simplify this?
std::optional<std::size_t> VoxelGrid::coordinate_to_index(XLength coordinate) {
    return centimeters_to_index(coordinate.force_numerical_value_in(cm));
}

std::optional<std::size_t> VoxelGrid::coordinate_to_index(YLength coordinate) {
    return centimeters_to_index(coordinate.force_numerical_value_in(cm));
}

std::optional<std::size_t> VoxelGrid::coordinate_to_index(ZLength coordinate) {
    return centimeters_to_index(coordinate.force_numerical_value_in(cm));
}

} // namespace cpp_course
