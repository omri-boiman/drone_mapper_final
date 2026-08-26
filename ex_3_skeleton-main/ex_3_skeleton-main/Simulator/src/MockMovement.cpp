#include <Simulator/MockMovement.h>

#include <cmath>
#include <numbers>

namespace simulator {

namespace {

constexpr double toCm(PhysicalLength l)  { return l.force_numerical_value_in(cm); }
constexpr double toDeg(HorizontalAngle a){ return a.force_numerical_value_in(deg); }
constexpr double toRad(HorizontalAngle a){ return toDeg(a) * std::numbers::pi / 180.0; }

HorizontalAngle clampAngle(HorizontalAngle value, HorizontalAngle limit) {
    if (value >  limit) return  limit;
    if (value < -limit) return -limit;
    return value;
}

PhysicalLength clampDist(PhysicalLength value, PhysicalLength limit) {
    if (value >  limit) return  limit;
    if (value < -limit) return -limit;
    return value;
}

HorizontalAngle wrapHeading(HorizontalAngle angle) {
    double raw = std::fmod(toDeg(angle), 360.0);
    if (raw < 0.0) raw += 360.0;
    return raw * horizontal_angle[deg];
}

} // namespace

MockMovement::MockMovement(MockGPS& gps,
                           const IMap3D& hidden_map,
                           const common::types::DroneConfigData& config)
    : gps_(gps), hidden_map_(hidden_map), config_(config) {}

common::types::MovementResult MockMovement::rotate(common::types::RotationDirection direction,
                                           HorizontalAngle angle) {
    const HorizontalAngle clamped = clampAngle(angle, config_.max_rotate);
    const HorizontalAngle signed_delta =
        (direction == common::types::RotationDirection::Left) ? -clamped : clamped;
    const Orientation current = gps_.heading();
    gps_.setHeading(Orientation{wrapHeading(current.horizontal + signed_delta),
                                current.altitude});
    return {true, {}};
}

common::types::MovementResult MockMovement::advance(PhysicalLength distance) {
    const PhysicalLength dist = clampDist(distance, config_.max_advance);
    const double distCm     = toCm(dist);
    const double headingRad = toRad(gps_.heading().horizontal);
    const double dx = distCm * std::cos(headingRad);
    const double dy = distCm * std::sin(headingRad);

    const Position3D start = gps_.position();
    const double startX = toCm(start.x);
    const double startY = toCm(start.y);
    const double startZ = toCm(start.z);

    const int steps = static_cast<int>(std::abs(distCm)) + 1;
    for (int i = 1; i <= steps; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(steps);
        const Position3D sample{
            (startX + t * dx) * x_extent[cm],
            (startY + t * dy) * y_extent[cm],
            startZ             * z_extent[cm],
        };
        if (hidden_map_.atVoxel(sample) == common::types::VoxelOccupancy::Occupied) {
            return {false, "DRONE_HITS_OBSTACLE"};
        }
    }

    gps_.setPosition(Position3D{
        (startX + dx) * x_extent[cm],
        (startY + dy) * y_extent[cm],
        startZ         * z_extent[cm],
    });
    return {true, {}};
}

common::types::MovementResult MockMovement::elevate(PhysicalLength distance) {
    const PhysicalLength dist = clampDist(distance, config_.max_elevate);
    const double distCm = toCm(dist);

    const Position3D start = gps_.position();
    const double startX = toCm(start.x);
    const double startY = toCm(start.y);
    const double startZ = toCm(start.z);

    const int steps = static_cast<int>(std::abs(distCm)) + 1;
    for (int i = 1; i <= steps; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(steps);
        const Position3D sample{
            startX               * x_extent[cm],
            startY               * y_extent[cm],
            (startZ + t * distCm) * z_extent[cm],
        };
        if (hidden_map_.atVoxel(sample) == common::types::VoxelOccupancy::Occupied) {
            return {false, "DRONE_HITS_OBSTACLE"};
        }
    }

    gps_.setPosition(Position3D{
        startX              * x_extent[cm],
        startY              * y_extent[cm],
        (startZ + distCm)   * z_extent[cm],
    });
    return {true, {}};
}

} // namespace simulator
