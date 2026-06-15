#include "TestHelpers.h"

#include <cpp_course/VoxelGrid.h>

#include <gtest/gtest.h>

#include <filesystem>

using namespace cpp_course;

namespace {

std::string map_path(const char* filename) {
    return (std::filesystem::path(CPP_COURSE_SOURCE_DIR) / "data_maps" / filename).string();
}

} // namespace

TEST(VoxelGridTests, LoadsOneBlockMapAndReportsShape) {
    const VoxelGrid grid(map_path("single_voxel_x2_y4_z2.npy"));

    EXPECT_EQ(grid.x_size(), 5U);
    EXPECT_EQ(grid.y_size(), 5U);
    EXPECT_EQ(grid.z_size(), 5U);
}

TEST(VoxelGridTests, ReturnsOccupiedVoxelFromOneBlockMap) {
    const VoxelGrid grid(map_path("single_voxel_x2_y4_z2.npy"));

    EXPECT_EQ(grid.get(test::make_position(2.0, 4.0, 2.0)), 1);
    EXPECT_EQ(grid.get(test::make_position(2.0, 4.0, 1.0)), 0);
    EXPECT_EQ(grid.get(test::make_position(1.0, 4.0, 2.0)), 0);
}

TEST(VoxelGridTests, ReturnsZeroForNegativeAndOutOfBoundsCoordinates) {
    const VoxelGrid grid(map_path("single_voxel_x2_y4_z2.npy"));

    EXPECT_EQ(grid.get(test::make_position(-1.0, 4.0, 2.0)), 0);
    EXPECT_EQ(grid.get(test::make_position(2.0, -1.0, 2.0)), 0);
    EXPECT_EQ(grid.get(test::make_position(2.0, 4.0, -1.0)), 0);
    EXPECT_EQ(grid.get(test::make_position(5.0, 4.0, 2.0)), 0);
    EXPECT_EQ(grid.get(test::make_position(2.0, 5.0, 2.0)), 0);
    EXPECT_EQ(grid.get(test::make_position(2.0, 4.0, 5.0)), 0);
}

TEST(VoxelGridTests, LoadsUint8CornerMap) {
    const VoxelGrid grid(map_path("single_voxel_x4_y4_z4.npy"));

    EXPECT_EQ(grid.get(test::make_position(4.0, 4.0, 4.0)), 1);
    EXPECT_EQ(grid.get(test::make_position(4.0, 4.0, 3.0)), 0);
}

TEST(VoxelGridTests, ReadsMultipleKnownOccupiedCells) {
    const VoxelGrid grid(map_path("five_voxels_y4_pattern.npy"));

    EXPECT_EQ(grid.get(test::make_position(2.0, 4.0, 1.0)), 1);
    EXPECT_EQ(grid.get(test::make_position(2.0, 4.0, 3.0)), 1);
    EXPECT_EQ(grid.get(test::make_position(3.0, 4.0, 2.0)), 1);
    EXPECT_EQ(grid.get(test::make_position(4.0, 4.0, 1.0)), 1);
    EXPECT_EQ(grid.get(test::make_position(4.0, 4.0, 3.0)), 1);
    EXPECT_EQ(grid.get(test::make_position(0.0, 0.0, 0.0)), 0);
}
