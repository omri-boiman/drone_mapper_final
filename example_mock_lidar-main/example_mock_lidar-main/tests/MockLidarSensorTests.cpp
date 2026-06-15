#include "TestHelpers.h"

#include <cpp_course/MockLidarSensor.h>
#include <cpp_course/PositionSensor.h>

#include <gtest/gtest.h>

#include <set>
#include <tuple>

using namespace cpp_course;

namespace {

class EmptyMap final : public IMap3D {
public:
    [[nodiscard]] int get(const Position3D&) const override {
        return 0;
    }
};

class AlwaysOccupiedMap final : public IMap3D {
public:
    [[nodiscard]] int get(const Position3D&) const override {
        return 1;
    }
};

class SingleVoxelMap final : public IMap3D {
public:
    SingleVoxelMap(int x, int y, int z) : x_(x), y_(y), z_(z) {}

    [[nodiscard]] int get(const Position3D& pos) const override {
        return test::voxel_coordinates(pos) == std::tuple{x_, y_, z_} ? 1 : 0;
    }

private:
    int x_;
    int y_;
    int z_;
};

class FixedPositionSensor final : public IPositionSensor {
public:
    FixedPositionSensor(Position3D position, Orientation heading)
        : position_(position), heading_(heading) {}

    [[nodiscard]] Position3D position() const override {
        return position_;
    }

    [[nodiscard]] Orientation heading() const override {
        return heading_;
    }

private:
    Position3D position_;
    Orientation heading_;
};

LidarConfig config_with(std::size_t fov_circles) {
    return {
        1.0 * cm,
        20.0 * cm,
        1.0 * cm,
        fov_circles,
    };
}

std::size_t expected_hit_count(std::size_t fov_circles) {
    if (fov_circles == 0) {
        return 0;
    }

    std::size_t total = 1;
    std::size_t ring = 1;
    for (std::size_t i = 1; i < fov_circles; ++i) {
        ring *= 4;
        total += ring;
    }
    return total;
}

} // namespace

TEST(MockLidarSensorTests, ReturnsConfiguredReference) {
    const EmptyMap map;
    const FixedPositionSensor sensor(test::make_position(0.0, 0.0, 0.0), test::make_orientation(0.0, 0.0));
    const LidarConfig config = config_with(3);

    const MockLidarSensor lidar(config, map, sensor);

    EXPECT_EQ(lidar.config().fov_circles, 3U);
    EXPECT_EQ(lidar.config().beam_length_min, 1.0 * cm);
    EXPECT_EQ(lidar.config().beam_length_max, 20.0 * cm);
    EXPECT_EQ(lidar.config().circle_spacing, 1.0 * cm);
}

TEST(MockLidarSensorTests, ReturnsEmptyScanWhenNoCirclesConfigured) {
    const EmptyMap map;
    const FixedPositionSensor sensor(test::make_position(0.0, 0.0, 0.0), test::make_orientation(0.0, 0.0));

    const MockLidarSensor lidar(config_with(0), map, sensor);

    EXPECT_TRUE(lidar.scan(test::make_orientation(0.0, 0.0)).empty());
}

TEST(MockLidarSensorTests, ReturnsExpectedNumberOfHitsWhenEveryBeamHits) {
    const AlwaysOccupiedMap map;
    const FixedPositionSensor sensor(test::make_position(0.0, 0.0, 0.0), test::make_orientation(0.0, 0.0));

    const MockLidarSensor lidar(config_with(4), map, sensor);
    const ScanResults hits = lidar.scan(test::make_orientation(0.0, 0.0));

    EXPECT_EQ(hits.size(), expected_hit_count(4));
}

TEST(MockLidarSensorTests, ReturnsEmptyScanWhenNoBeamHitsMap) {
    const EmptyMap map;
    const FixedPositionSensor sensor(test::make_position(0.0, 0.0, 0.0), test::make_orientation(0.0, 0.0));

    const MockLidarSensor lidar(config_with(3), map, sensor);

    EXPECT_TRUE(lidar.scan(test::make_orientation(0.0, 0.0)).empty());
}

TEST(MockLidarSensorTests, IgnoresOccupiedVoxelsPastMaximumBeamLength) {
    const SingleVoxelMap map(25, 0, 0);
    const FixedPositionSensor sensor(test::make_position(0.0, 0.0, 0.0), test::make_orientation(0.0, 0.0));

    const MockLidarSensor lidar(config_with(1), map, sensor);

    EXPECT_TRUE(lidar.scan(test::make_orientation(0.0, 0.0)).empty());
}

TEST(MockLidarSensorTests, HitCloserThanBeamLengthMinReturnsZeroDistance) {
    const SingleVoxelMap map(0, 0, 0);
    const FixedPositionSensor sensor(test::make_position(0.0, 0.0, 0.0), test::make_orientation(0.0, 0.0));

    const MockLidarSensor lidar(config_with(1), map, sensor);
    const ScanResults hits = lidar.scan(test::make_orientation(0.0, 0.0));

    ASSERT_EQ(hits.size(), 1U);
    const auto expected_distance = 0.0 * cm;
    const auto actual_distance = hits.front().distance;
    EXPECT_EQ(actual_distance, expected_distance)
        << "expected distance=" << expected_distance
        << ", actual distance=" << actual_distance;
}

TEST(MockLidarSensorTests, ReportsDistanceToFirstHitOnCenterBeam) {
    const SingleVoxelMap map(5, 0, 0);
    const FixedPositionSensor sensor(test::make_position(0.0, 0.0, 0.0), test::make_orientation(0.0, 0.0));

    const MockLidarSensor lidar(config_with(1), map, sensor);
    const ScanResults hits = lidar.scan(test::make_orientation(0.0, 0.0));

    ASSERT_EQ(hits.size(), 1U);
    EXPECT_NEAR(hits.front().distance.force_numerical_value_in(cm), 5.0, 0.11);
    EXPECT_NEAR(test::degrees(hits.front().angle.horizontal), 0.0, 1e-9);
    EXPECT_NEAR(test::degrees(hits.front().angle.altitude), 0.0, 1e-9);
}

TEST(MockLidarSensorTests, UsesRequestedScanOrientationForCenterBeam) {
    const SingleVoxelMap map(0, 5, 0);
    const FixedPositionSensor sensor(test::make_position(0.0, 0.0, 0.0), test::make_orientation(0.0, 0.0));

    const MockLidarSensor lidar(config_with(1), map, sensor);
    const ScanResults hits = lidar.scan(test::make_orientation(90.0, 0.0));

    ASSERT_EQ(hits.size(), 1U);
    EXPECT_NEAR(hits.front().distance.force_numerical_value_in(cm), 5.0, 0.11);
    EXPECT_NEAR(test::degrees(hits.front().angle.horizontal), 90.0, 1e-9);
    EXPECT_NEAR(test::degrees(hits.front().angle.altitude), 0.0, 1e-9);
}

TEST(MockLidarSensorTests, UsesSensorHeadingForTracingButReturnsRelativeBeamAngle) {
    const SingleVoxelMap map(0, 5, 0);
    const FixedPositionSensor sensor(test::make_position(0.0, 0.0, 0.0), test::make_orientation(90.0, 0.0));

    const MockLidarSensor lidar(config_with(1), map, sensor);
    const ScanResults hits = lidar.scan(test::make_orientation(0.0, 0.0));

    ASSERT_EQ(hits.size(), 1U);
    EXPECT_NEAR(hits.front().distance.force_numerical_value_in(cm), 5.0, 0.11);
    EXPECT_NEAR(test::degrees(hits.front().angle.horizontal), 0.0, 1e-9);
    EXPECT_NEAR(test::degrees(hits.front().angle.altitude), 0.0, 1e-9);

    const Position3D reconstructed = test::hit_to_position(
        sensor.position(),
        test::absolute_beam(sensor.heading(), hits.front().angle),
        hits.front().distance);
    EXPECT_EQ(test::voxel_coordinates(reconstructed), (std::tuple{0, 5, 0}));
}

TEST(MockLidarSensorTests, SupportsAltitudeInRequestedScanOrientation) {
    const SingleVoxelMap map(0, 0, 5);
    const FixedPositionSensor sensor(test::make_position(0.0, 0.0, 0.0), test::make_orientation(0.0, 0.0));

    const MockLidarSensor lidar(config_with(1), map, sensor);
    const ScanResults hits = lidar.scan(test::make_orientation(0.0, 90.0));

    ASSERT_EQ(hits.size(), 1U);
    EXPECT_NEAR(hits.front().distance.force_numerical_value_in(cm), 5.0, 0.11);
    EXPECT_NEAR(test::degrees(hits.front().angle.horizontal), 0.0, 1e-9);
    EXPECT_NEAR(test::degrees(hits.front().angle.altitude), 90.0, 1e-9);
}

TEST(MockLidarSensorTests, ReconstructsHitVoxelsCorrectlyWithZeroHeading) {
    const SingleVoxelMap map(5, 0, 0);
    const FixedPositionSensor sensor(test::make_position(0.0, 0.0, 0.0), test::make_orientation(0.0, 0.0));

    const MockLidarSensor lidar(config_with(5), map, sensor);
    const ScanResults hits = lidar.scan(test::make_orientation(0.0, 0.0));

    ASSERT_FALSE(hits.empty());

    std::set<std::tuple<int, int, int>> reconstructed_voxels;
    for (const auto& hit : hits) {
        const Position3D reconstructed = test::hit_to_position(
            sensor.position(),
            test::absolute_beam(sensor.heading(), hit.angle),
            hit.distance);
        reconstructed_voxels.insert(test::voxel_coordinates(reconstructed));
        EXPECT_EQ(map.get(reconstructed), 1);
    }

    EXPECT_EQ(reconstructed_voxels, (std::set<std::tuple<int, int, int>>{{5, 0, 0}}));
}
