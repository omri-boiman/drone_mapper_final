#include <Simulator/Map3DImpl.h>

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <stdexcept>
#include <string>

namespace simulator {

std::size_t Map3DImpl::flatIndex(std::size_t xi, std::size_t yi, std::size_t zi) const noexcept {
    return xi * y_size_ * z_size_ + yi * z_size_ + zi;
}

bool Map3DImpl::toIndex(const Position3D& pos,
                        std::size_t& xi, std::size_t& yi, std::size_t& zi) const noexcept {
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

Map3DImpl::Map3DImpl(const std::filesystem::path& path, PhysicalLength resolution)
    : Map3DImpl(path, resolution, {}) {}

Map3DImpl::Map3DImpl(const std::filesystem::path& path, PhysicalLength resolution, Position3D offset) {
    map_ = std::make_shared<NpyArray>();
    const char* err = map_->LoadNPY(path.string());
    if (err != nullptr)
        throw std::runtime_error(std::string("Map3DImpl: failed to load '") +
                                 path.string() + "': " + err);

    if (map_->Shape().size() != 3)
        throw std::runtime_error("Map3DImpl: NPY must be 3-dimensional: " + path.string());
    if (map_->ColMajor())
        throw std::runtime_error("Map3DImpl: NPY must be row-major: " + path.string());
    if (map_->ValueType() != typeid(int) &&
        map_->ValueType() != typeid(uint8_t) &&
        map_->ValueType() != typeid(int8_t) &&
        map_->ValueType() != typeid(char))
        throw std::runtime_error("Map3DImpl: unsupported dtype in '" + path.string() + "'");

    x_size_ = map_->Shape()[0];
    y_size_ = map_->Shape()[1];
    z_size_ = map_->Shape()[2];

    const double res_cm = resolution.force_numerical_value_in(cm);
    config_.resolution = resolution;
    config_.offset = offset;
    config_.boundaries.min_x      = offset.x;
    config_.boundaries.max_x      = offset.x + static_cast<double>(x_size_) * res_cm * x_extent[cm];
    config_.boundaries.min_y      = offset.y;
    config_.boundaries.max_y      = offset.y + static_cast<double>(y_size_) * res_cm * y_extent[cm];
    config_.boundaries.min_height = offset.z;
    config_.boundaries.max_height = offset.z + static_cast<double>(z_size_) * res_cm * z_extent[cm];

    is_output_map_ = false;
}

Map3DImpl::Map3DImpl(const common::types::MappingBounds& bounds, PhysicalLength resolution, Position3D offset) {
    const double res = resolution.force_numerical_value_in(cm);
    config_.resolution = resolution;
    config_.offset = offset;
    config_.boundaries = bounds;

    const double range_x = bounds.max_x.force_numerical_value_in(cm) - bounds.min_x.force_numerical_value_in(cm);
    const double range_y = bounds.max_y.force_numerical_value_in(cm) - bounds.min_y.force_numerical_value_in(cm);
    const double range_z = bounds.max_height.force_numerical_value_in(cm) - bounds.min_height.force_numerical_value_in(cm);

    x_size_ = std::max(std::size_t{1}, static_cast<std::size_t>(std::ceil(range_x / res)));
    y_size_ = std::max(std::size_t{1}, static_cast<std::size_t>(std::ceil(range_y / res)));
    z_size_ = std::max(std::size_t{1}, static_cast<std::size_t>(std::ceil(range_z / res)));

    output_data_.assign(x_size_ * y_size_ * z_size_,
                        static_cast<int8_t>(common::types::VoxelOccupancy::Unmapped));
    is_output_map_ = true;
}

common::types::VoxelOccupancy Map3DImpl::atVoxel(const Position3D& pos) const {
    std::size_t xi = 0, yi = 0, zi = 0;
    if (!toIndex(pos, xi, yi, zi))
        return common::types::VoxelOccupancy::OutOfBounds;

    const std::size_t idx = flatIndex(xi, yi, zi);

    if (is_output_map_)
        return static_cast<common::types::VoxelOccupancy>(output_data_[idx]);

    const auto& vt = map_->ValueType();
    int raw = 0;
    if (vt == typeid(int))
        raw = map_->Data<int>()[idx];
    else if (vt == typeid(uint8_t))
        raw = static_cast<int>(map_->Data<uint8_t>()[idx]);
    else if (vt == typeid(int8_t))
        raw = static_cast<int>(map_->Data<int8_t>()[idx]);
    else if (vt == typeid(char))
        raw = static_cast<int>(map_->Data<char>()[idx]);

    return raw != 0 ? common::types::VoxelOccupancy::Occupied : common::types::VoxelOccupancy::Empty;
}

common::types::MapConfig Map3DImpl::getMapConfig() const {
    return config_;
}

bool Map3DImpl::isInBounds(const Position3D& pos) const {
    std::size_t xi = 0, yi = 0, zi = 0;
    return toIndex(pos, xi, yi, zi);
}

void Map3DImpl::set(const Position3D& pos, common::types::VoxelOccupancy value) {
    if (!is_output_map_) return;
    std::size_t xi = 0, yi = 0, zi = 0;
    if (!toIndex(pos, xi, yi, zi)) return;
    output_data_[flatIndex(xi, yi, zi)] = static_cast<int8_t>(value);
}

void Map3DImpl::save(const std::filesystem::path& path) const {
    std::filesystem::create_directories(path.parent_path());

    const char* err = nullptr;
    if (is_output_map_) {
        err = NpyArray::SaveNPY(path.string(), output_data_,
                                NpyArray::shape_t{x_size_, y_size_, z_size_});
    } else {
        err = map_->SaveNPY(path.string());
    }

    if (err != nullptr)
        throw std::runtime_error(std::string("Map3DImpl: failed to save '") +
                                 path.string() + "': " + err);
}

} // namespace simulator
