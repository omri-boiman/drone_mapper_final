#include <gtest/gtest.h>

#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MockGPS.h>
#include <drone_mapper/MockLidar.h>

#include <limits>

namespace dm = drone_mapper;

namespace {

dm::Map3DImpl singleVoxelMap() {
    return dm::Map3DImpl("data_maps/single_voxel_x2_y4_z2.npy", 10.0*dm::cm);
}

dm::Map3DImpl emptyMap() {
    dm::types::MappingBounds b{
        0.0*dm::x_extent[dm::cm], 300.0*dm::x_extent[dm::cm],
        0.0*dm::y_extent[dm::cm], 300.0*dm::y_extent[dm::cm],
        0.0*dm::z_extent[dm::cm], 300.0*dm::z_extent[dm::cm],
    };
    return dm::Map3DImpl(b, 10.0*dm::cm, dm::Position3D{});
}

dm::types::LidarConfigData defaultLidar() {
    return {20.0*dm::cm, 120.0*dm::cm, 2.5*dm::cm, 3};
}

// Miss: distance == max double cm (new API sentinel)
bool isMiss(const dm::types::LidarHit& hit) {
    return hit.distance.force_numerical_value_in(dm::cm) >=
           std::numeric_limits<double>::max() / 2.0;
}

} // namespace

class MockLidar : public ::testing::Test {};

TEST_F(MockLidar, ScanOnEmptyMapReturnsAllMisses) {
    auto map = emptyMap();
    dm::MockGPS gps{
        {0.0*dm::x_extent[dm::cm], 0.0*dm::y_extent[dm::cm], 0.0*dm::z_extent[dm::cm]},
        {0.0*dm::horizontal_angle[dm::deg], 0.0*dm::altitude_angle[dm::deg]},
        10.0*dm::cm};
    dm::MockLidar lidar(defaultLidar(), map, gps);

    const auto result = lidar.scan({0.0*dm::horizontal_angle[dm::deg], 0.0*dm::altitude_angle[dm::deg]});
    ASSERT_FALSE(result.empty());
    for (const auto& hit : result)
        EXPECT_TRUE(isMiss(hit)) << "Expected miss on empty map";
}

TEST_F(MockLidar, ScanDetectsOccupiedVoxel) {
    auto map = singleVoxelMap();
    // Drone at (0, 40, 20) facing east (+X); voxel at (20, 40, 20)
    dm::MockGPS gps{
        {0.0*dm::x_extent[dm::cm], 40.0*dm::y_extent[dm::cm], 20.0*dm::z_extent[dm::cm]},
        {0.0*dm::horizontal_angle[dm::deg], 0.0*dm::altitude_angle[dm::deg]},
        10.0*dm::cm};
    dm::types::LidarConfigData lidar_cfg{5.0*dm::cm, 60.0*dm::cm, 2.5*dm::cm, 1};
    dm::MockLidar lidar(lidar_cfg, map, gps);

    const auto result = lidar.scan({0.0*dm::horizontal_angle[dm::deg], 0.0*dm::altitude_angle[dm::deg]});
    ASSERT_EQ(result.size(), 1u);
    EXPECT_FALSE(isMiss(result[0]));
    EXPECT_GT(result[0].distance.force_numerical_value_in(dm::cm), 0.0);
}

TEST_F(MockLidar, BeamCountMatchesFovCircles) {
    auto map = emptyMap();
    dm::MockGPS gps{
        {50.0*dm::x_extent[dm::cm], 50.0*dm::y_extent[dm::cm], 50.0*dm::z_extent[dm::cm]},
        {0.0*dm::horizontal_angle[dm::deg], 0.0*dm::altitude_angle[dm::deg]},
        10.0*dm::cm};
    // 3 circles: 1 + 4 + 16 = 21 beams
    dm::types::LidarConfigData lidar_cfg{5.0*dm::cm, 80.0*dm::cm, 2.5*dm::cm, 3};
    dm::MockLidar lidar(lidar_cfg, map, gps);

    const auto result = lidar.scan({0.0*dm::horizontal_angle[dm::deg], 0.0*dm::altitude_angle[dm::deg]});
    EXPECT_EQ(result.size(), 21u);
}

TEST_F(MockLidar, ZeroFovCirclesReturnsEmpty) {
    auto map = emptyMap();
    dm::MockGPS gps{
        {10.0*dm::x_extent[dm::cm], 10.0*dm::y_extent[dm::cm], 10.0*dm::z_extent[dm::cm]},
        {0.0*dm::horizontal_angle[dm::deg], 0.0*dm::altitude_angle[dm::deg]},
        10.0*dm::cm};
    dm::types::LidarConfigData lidar_cfg{5.0*dm::cm, 80.0*dm::cm, 2.5*dm::cm, 0};
    dm::MockLidar lidar(lidar_cfg, map, gps);

    const auto result = lidar.scan({0.0*dm::horizontal_angle[dm::deg], 0.0*dm::altitude_angle[dm::deg]});
    EXPECT_TRUE(result.empty());
}

TEST_F(MockLidar, MissDistanceIsMaxDouble) {
    auto map = emptyMap();
    dm::MockGPS gps{
        {50.0*dm::x_extent[dm::cm], 50.0*dm::y_extent[dm::cm], 50.0*dm::z_extent[dm::cm]},
        {0.0*dm::horizontal_angle[dm::deg], 0.0*dm::altitude_angle[dm::deg]},
        10.0*dm::cm};
    dm::types::LidarConfigData lidar_cfg{5.0*dm::cm, 80.0*dm::cm, 2.5*dm::cm, 1};
    dm::MockLidar lidar(lidar_cfg, map, gps);

    const auto result = lidar.scan({0.0*dm::horizontal_angle[dm::deg], 0.0*dm::altitude_angle[dm::deg]});
    ASSERT_EQ(result.size(), 1u);
    EXPECT_TRUE(isMiss(result[0]));
}

TEST_F(MockLidar, ObstacleBeyondMaxRangeIsMiss) {
    // Voxel at (20, 40, 20), drone at (0, 40, 20) facing east — distance is 20cm.
    // Set z_max=10cm so the voxel is beyond range.
    auto map = singleVoxelMap();
    dm::MockGPS gps{
        {0.0*dm::x_extent[dm::cm], 40.0*dm::y_extent[dm::cm], 20.0*dm::z_extent[dm::cm]},
        {0.0*dm::horizontal_angle[dm::deg], 0.0*dm::altitude_angle[dm::deg]},
        10.0*dm::cm};
    dm::types::LidarConfigData lidar_cfg{5.0*dm::cm, 10.0*dm::cm, 2.5*dm::cm, 1};
    dm::MockLidar lidar(lidar_cfg, map, gps);

    const auto result = lidar.scan({0.0*dm::horizontal_angle[dm::deg], 0.0*dm::altitude_angle[dm::deg]});
    ASSERT_EQ(result.size(), 1u);
    EXPECT_TRUE(isMiss(result[0])) << "Voxel beyond z_max should be a miss";
}

TEST_F(MockLidar, ScanAtOppositeHeadingDetectsVoxelBehind) {
    // Voxel at (20, 40, 20). Drone at (40, 40, 20) facing east — voxel is 20cm west.
    // Scanning at 180° (west relative to drone) should detect the voxel.
    // Scanning at 0° (east) should miss.
    auto map = singleVoxelMap();
    dm::MockGPS gps{
        {40.0*dm::x_extent[dm::cm], 40.0*dm::y_extent[dm::cm], 20.0*dm::z_extent[dm::cm]},
        {0.0*dm::horizontal_angle[dm::deg], 0.0*dm::altitude_angle[dm::deg]},
        10.0*dm::cm};
    dm::types::LidarConfigData lidar_cfg{5.0*dm::cm, 30.0*dm::cm, 2.5*dm::cm, 1};
    dm::MockLidar lidar(lidar_cfg, map, gps);

    const auto east_result = lidar.scan({0.0*dm::horizontal_angle[dm::deg], 0.0*dm::altitude_angle[dm::deg]});
    ASSERT_EQ(east_result.size(), 1u);
    EXPECT_TRUE(isMiss(east_result[0])) << "Scanning east should miss the voxel that's to the west";

    const auto west_result = lidar.scan({180.0*dm::horizontal_angle[dm::deg], 0.0*dm::altitude_angle[dm::deg]});
    ASSERT_EQ(west_result.size(), 1u);
    EXPECT_FALSE(isMiss(west_result[0])) << "Scanning west should detect the voxel at 20cm";
}

TEST_F(MockLidar, ObstacleExactlyAtMaxRangeIsDetected) {
    // Voxel at (20, 40, 20), drone at (0, 40, 20) facing east — distance is exactly 20cm.
    // z_max=20cm means the voxel sits exactly on the far edge of the beam's range and
    // must still be sampled and reported as a hit (not clipped short of z_max).
    auto map = singleVoxelMap();
    dm::MockGPS gps{
        {0.0*dm::x_extent[dm::cm], 40.0*dm::y_extent[dm::cm], 20.0*dm::z_extent[dm::cm]},
        {0.0*dm::horizontal_angle[dm::deg], 0.0*dm::altitude_angle[dm::deg]},
        10.0*dm::cm};
    dm::types::LidarConfigData lidar_cfg{5.0*dm::cm, 20.0*dm::cm, 2.5*dm::cm, 1};
    dm::MockLidar lidar(lidar_cfg, map, gps);

    const auto result = lidar.scan({0.0*dm::horizontal_angle[dm::deg], 0.0*dm::altitude_angle[dm::deg]});
    ASSERT_EQ(result.size(), 1u);
    EXPECT_FALSE(isMiss(result[0])) << "Voxel exactly at z_max should still be detected";
    EXPECT_NEAR(result[0].distance.force_numerical_value_in(dm::cm), 20.0, 0.5);
}

TEST_F(MockLidar, ObstacleExactlyAtMinRangeIsDetectedNotTooClose) {
    // Voxel at (20, 40, 20), drone at (0, 40, 20) facing east — distance is exactly 20cm,
    // and z_min is also 20cm. A hit exactly at z_min must be reported with its real distance,
    // not treated as "too close" (which would report distance 0).
    auto map = singleVoxelMap();
    dm::MockGPS gps{
        {0.0*dm::x_extent[dm::cm], 40.0*dm::y_extent[dm::cm], 20.0*dm::z_extent[dm::cm]},
        {0.0*dm::horizontal_angle[dm::deg], 0.0*dm::altitude_angle[dm::deg]},
        10.0*dm::cm};
    dm::types::LidarConfigData lidar_cfg{20.0*dm::cm, 60.0*dm::cm, 2.5*dm::cm, 1};
    dm::MockLidar lidar(lidar_cfg, map, gps);

    const auto result = lidar.scan({0.0*dm::horizontal_angle[dm::deg], 0.0*dm::altitude_angle[dm::deg]});
    ASSERT_EQ(result.size(), 1u);
    EXPECT_FALSE(isMiss(result[0]));
    EXPECT_NEAR(result[0].distance.force_numerical_value_in(dm::cm), 20.0, 0.5)
        << "Hit exactly at z_min should report its real distance, not be marked too-close (0)";
}

TEST_F(MockLidar, FovCircles4Has85Beams) {
    auto map = emptyMap();
    dm::MockGPS gps{
        {50.0*dm::x_extent[dm::cm], 50.0*dm::y_extent[dm::cm], 50.0*dm::z_extent[dm::cm]},
        {0.0*dm::horizontal_angle[dm::deg], 0.0*dm::altitude_angle[dm::deg]},
        10.0*dm::cm};
    // 4 circles: 1 + 4 + 16 + 64 = 85 beams
    dm::types::LidarConfigData lidar_cfg{5.0*dm::cm, 80.0*dm::cm, 2.5*dm::cm, 4};
    dm::MockLidar lidar(lidar_cfg, map, gps);

    const auto result = lidar.scan({0.0*dm::horizontal_angle[dm::deg], 0.0*dm::altitude_angle[dm::deg]});
    EXPECT_EQ(result.size(), 85u);
}
