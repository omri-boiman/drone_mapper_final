#include <gtest/gtest.h>

#include <Simulator/Map3DImpl.h>
#include <Simulator/MockGPS.h>
#include <Simulator/MockLidar.h>

#include <limits>

namespace sim = simulator;

namespace {

sim::Map3DImpl singleVoxelMap() {
    return sim::Map3DImpl("data_maps/single_voxel_x2_y4_z2.npy", 10.0*sim::cm);
}

sim::Map3DImpl emptyMap() {
    common::types::MappingBounds b{
        0.0*sim::x_extent[sim::cm], 300.0*sim::x_extent[sim::cm],
        0.0*sim::y_extent[sim::cm], 300.0*sim::y_extent[sim::cm],
        0.0*sim::z_extent[sim::cm], 300.0*sim::z_extent[sim::cm],
    };
    return sim::Map3DImpl(b, 10.0*sim::cm, sim::Position3D{});
}

common::types::LidarConfigData defaultLidar() {
    return {20.0*sim::cm, 120.0*sim::cm, 2.5*sim::cm, 3};
}

// Miss: distance == max double cm (new API sentinel)
bool isMiss(const common::types::LidarHit& hit) {
    return hit.distance.force_numerical_value_in(sim::cm) >=
           std::numeric_limits<double>::max() / 2.0;
}

} // namespace

class MockLidar : public ::testing::Test {};

TEST_F(MockLidar, ScanOnEmptyMapReturnsAllMisses) {
    auto map = emptyMap();
    sim::MockGPS gps{
        {0.0*sim::x_extent[sim::cm], 0.0*sim::y_extent[sim::cm], 0.0*sim::z_extent[sim::cm]},
        {0.0*sim::horizontal_angle[sim::deg], 0.0*sim::altitude_angle[sim::deg]},
        10.0*sim::cm};
    sim::MockLidar lidar(defaultLidar(), map, gps);

    const auto result = lidar.scan({0.0*sim::horizontal_angle[sim::deg], 0.0*sim::altitude_angle[sim::deg]});
    ASSERT_FALSE(result.empty());
    for (const auto& hit : result)
        EXPECT_TRUE(isMiss(hit)) << "Expected miss on empty map";
}

TEST_F(MockLidar, ScanDetectsOccupiedVoxel) {
    auto map = singleVoxelMap();
    // Drone at (0, 40, 20) facing east (+X); voxel at (20, 40, 20)
    sim::MockGPS gps{
        {0.0*sim::x_extent[sim::cm], 40.0*sim::y_extent[sim::cm], 20.0*sim::z_extent[sim::cm]},
        {0.0*sim::horizontal_angle[sim::deg], 0.0*sim::altitude_angle[sim::deg]},
        10.0*sim::cm};
    common::types::LidarConfigData lidar_cfg{5.0*sim::cm, 60.0*sim::cm, 2.5*sim::cm, 1};
    sim::MockLidar lidar(lidar_cfg, map, gps);

    const auto result = lidar.scan({0.0*sim::horizontal_angle[sim::deg], 0.0*sim::altitude_angle[sim::deg]});
    ASSERT_EQ(result.size(), 1u);
    EXPECT_FALSE(isMiss(result[0]));
    EXPECT_GT(result[0].distance.force_numerical_value_in(sim::cm), 0.0);
}

TEST_F(MockLidar, BeamCountMatchesFovCircles) {
    auto map = emptyMap();
    sim::MockGPS gps{
        {50.0*sim::x_extent[sim::cm], 50.0*sim::y_extent[sim::cm], 50.0*sim::z_extent[sim::cm]},
        {0.0*sim::horizontal_angle[sim::deg], 0.0*sim::altitude_angle[sim::deg]},
        10.0*sim::cm};
    // 3 circles: 1 + 4 + 16 = 21 beams
    common::types::LidarConfigData lidar_cfg{5.0*sim::cm, 80.0*sim::cm, 2.5*sim::cm, 3};
    sim::MockLidar lidar(lidar_cfg, map, gps);

    const auto result = lidar.scan({0.0*sim::horizontal_angle[sim::deg], 0.0*sim::altitude_angle[sim::deg]});
    EXPECT_EQ(result.size(), 21u);
}

TEST_F(MockLidar, ZeroFovCirclesReturnsEmpty) {
    auto map = emptyMap();
    sim::MockGPS gps{
        {10.0*sim::x_extent[sim::cm], 10.0*sim::y_extent[sim::cm], 10.0*sim::z_extent[sim::cm]},
        {0.0*sim::horizontal_angle[sim::deg], 0.0*sim::altitude_angle[sim::deg]},
        10.0*sim::cm};
    common::types::LidarConfigData lidar_cfg{5.0*sim::cm, 80.0*sim::cm, 2.5*sim::cm, 0};
    sim::MockLidar lidar(lidar_cfg, map, gps);

    const auto result = lidar.scan({0.0*sim::horizontal_angle[sim::deg], 0.0*sim::altitude_angle[sim::deg]});
    EXPECT_TRUE(result.empty());
}

TEST_F(MockLidar, MissDistanceIsMaxDouble) {
    auto map = emptyMap();
    sim::MockGPS gps{
        {50.0*sim::x_extent[sim::cm], 50.0*sim::y_extent[sim::cm], 50.0*sim::z_extent[sim::cm]},
        {0.0*sim::horizontal_angle[sim::deg], 0.0*sim::altitude_angle[sim::deg]},
        10.0*sim::cm};
    common::types::LidarConfigData lidar_cfg{5.0*sim::cm, 80.0*sim::cm, 2.5*sim::cm, 1};
    sim::MockLidar lidar(lidar_cfg, map, gps);

    const auto result = lidar.scan({0.0*sim::horizontal_angle[sim::deg], 0.0*sim::altitude_angle[sim::deg]});
    ASSERT_EQ(result.size(), 1u);
    EXPECT_TRUE(isMiss(result[0]));
}

TEST_F(MockLidar, ObstacleBeyondMaxRangeIsMiss) {
    // Voxel at (20, 40, 20), drone at (0, 40, 20) facing east — distance is 20cm.
    // Set z_max=10cm so the voxel is beyond range.
    auto map = singleVoxelMap();
    sim::MockGPS gps{
        {0.0*sim::x_extent[sim::cm], 40.0*sim::y_extent[sim::cm], 20.0*sim::z_extent[sim::cm]},
        {0.0*sim::horizontal_angle[sim::deg], 0.0*sim::altitude_angle[sim::deg]},
        10.0*sim::cm};
    common::types::LidarConfigData lidar_cfg{5.0*sim::cm, 10.0*sim::cm, 2.5*sim::cm, 1};
    sim::MockLidar lidar(lidar_cfg, map, gps);

    const auto result = lidar.scan({0.0*sim::horizontal_angle[sim::deg], 0.0*sim::altitude_angle[sim::deg]});
    ASSERT_EQ(result.size(), 1u);
    EXPECT_TRUE(isMiss(result[0])) << "Voxel beyond z_max should be a miss";
}

TEST_F(MockLidar, ScanAtOppositeHeadingDetectsVoxelBehind) {
    // Voxel at (20, 40, 20). Drone at (40, 40, 20) facing east — voxel is 20cm west.
    // Scanning at 180° (west relative to drone) should detect the voxel.
    // Scanning at 0° (east) should miss.
    auto map = singleVoxelMap();
    sim::MockGPS gps{
        {40.0*sim::x_extent[sim::cm], 40.0*sim::y_extent[sim::cm], 20.0*sim::z_extent[sim::cm]},
        {0.0*sim::horizontal_angle[sim::deg], 0.0*sim::altitude_angle[sim::deg]},
        10.0*sim::cm};
    common::types::LidarConfigData lidar_cfg{5.0*sim::cm, 30.0*sim::cm, 2.5*sim::cm, 1};
    sim::MockLidar lidar(lidar_cfg, map, gps);

    const auto east_result = lidar.scan({0.0*sim::horizontal_angle[sim::deg], 0.0*sim::altitude_angle[sim::deg]});
    ASSERT_EQ(east_result.size(), 1u);
    EXPECT_TRUE(isMiss(east_result[0])) << "Scanning east should miss the voxel that's to the west";

    const auto west_result = lidar.scan({180.0*sim::horizontal_angle[sim::deg], 0.0*sim::altitude_angle[sim::deg]});
    ASSERT_EQ(west_result.size(), 1u);
    EXPECT_FALSE(isMiss(west_result[0])) << "Scanning west should detect the voxel at 20cm";
}

TEST_F(MockLidar, ObstacleExactlyAtMaxRangeIsDetected) {
    auto map = singleVoxelMap();
    sim::MockGPS gps{
        {0.0*sim::x_extent[sim::cm], 40.0*sim::y_extent[sim::cm], 20.0*sim::z_extent[sim::cm]},
        {0.0*sim::horizontal_angle[sim::deg], 0.0*sim::altitude_angle[sim::deg]},
        10.0*sim::cm};
    common::types::LidarConfigData lidar_cfg{5.0*sim::cm, 20.0*sim::cm, 2.5*sim::cm, 1};
    sim::MockLidar lidar(lidar_cfg, map, gps);

    const auto result = lidar.scan({0.0*sim::horizontal_angle[sim::deg], 0.0*sim::altitude_angle[sim::deg]});
    ASSERT_EQ(result.size(), 1u);
    EXPECT_FALSE(isMiss(result[0])) << "Voxel exactly at z_max should still be detected";
    EXPECT_NEAR(result[0].distance.force_numerical_value_in(sim::cm), 20.0, 0.5);
}

TEST_F(MockLidar, ObstacleExactlyAtMinRangeIsDetectedNotTooClose) {
    auto map = singleVoxelMap();
    sim::MockGPS gps{
        {0.0*sim::x_extent[sim::cm], 40.0*sim::y_extent[sim::cm], 20.0*sim::z_extent[sim::cm]},
        {0.0*sim::horizontal_angle[sim::deg], 0.0*sim::altitude_angle[sim::deg]},
        10.0*sim::cm};
    common::types::LidarConfigData lidar_cfg{20.0*sim::cm, 60.0*sim::cm, 2.5*sim::cm, 1};
    sim::MockLidar lidar(lidar_cfg, map, gps);

    const auto result = lidar.scan({0.0*sim::horizontal_angle[sim::deg], 0.0*sim::altitude_angle[sim::deg]});
    ASSERT_EQ(result.size(), 1u);
    EXPECT_FALSE(isMiss(result[0]));
    EXPECT_NEAR(result[0].distance.force_numerical_value_in(sim::cm), 20.0, 0.5)
        << "Hit exactly at z_min should report its real distance, not be marked too-close (0)";
}

TEST_F(MockLidar, FovCircles4Has85Beams) {
    auto map = emptyMap();
    sim::MockGPS gps{
        {50.0*sim::x_extent[sim::cm], 50.0*sim::y_extent[sim::cm], 50.0*sim::z_extent[sim::cm]},
        {0.0*sim::horizontal_angle[sim::deg], 0.0*sim::altitude_angle[sim::deg]},
        10.0*sim::cm};
    // 4 circles: 1 + 4 + 16 + 64 = 85 beams
    common::types::LidarConfigData lidar_cfg{5.0*sim::cm, 80.0*sim::cm, 2.5*sim::cm, 4};
    sim::MockLidar lidar(lidar_cfg, map, gps);

    const auto result = lidar.scan({0.0*sim::horizontal_angle[sim::deg], 0.0*sim::altitude_angle[sim::deg]});
    EXPECT_EQ(result.size(), 85u);
}
