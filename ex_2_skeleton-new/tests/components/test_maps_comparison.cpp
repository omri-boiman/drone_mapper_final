#include <gtest/gtest.h>

#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MapsComparison.h>

namespace dm = drone_mapper;

namespace {

dm::Map3DImpl emptyMap(double res_cm = 10.0) {
    dm::types::MappingBounds bounds{
        0.0*dm::x_extent[dm::cm], 200.0*dm::x_extent[dm::cm],
        0.0*dm::y_extent[dm::cm], 200.0*dm::y_extent[dm::cm],
        0.0*dm::z_extent[dm::cm], 200.0*dm::z_extent[dm::cm],
    };
    return dm::Map3DImpl(bounds, res_cm*dm::cm, dm::Position3D{});
}

} // namespace

class MapsComparison : public ::testing::Test {};

TEST_F(MapsComparison, IdenticalFilesMapsReturn100) {
    dm::Map3DImpl m1("data_maps/single_voxel_x2_y4_z2.npy", 10.0*dm::cm);
    dm::Map3DImpl m2("data_maps/single_voxel_x2_y4_z2.npy", 10.0*dm::cm);
    const auto scores = dm::MapsComparison::compare(m1, {&m2});
    ASSERT_EQ(scores.size(), 1u);
    EXPECT_DOUBLE_EQ(scores[0], 100.0);
}

TEST_F(MapsComparison, TwoDifferentMapsReturnZero) {
    dm::Map3DImpl m1("data_maps/single_voxel_x2_y4_z2.npy", 10.0*dm::cm);
    dm::Map3DImpl m2("data_maps/single_voxel_x4_y4_z4.npy", 10.0*dm::cm);
    const auto scores = dm::MapsComparison::compare(m1, {&m2});
    ASSERT_EQ(scores.size(), 1u);
    EXPECT_EQ(scores[0], 0.0);
}

TEST_F(MapsComparison, BothEmptyOutputMapsReturn100) {
    auto m1 = emptyMap();
    auto m2 = emptyMap();
    const auto scores = dm::MapsComparison::compare(m1, {&m2});
    ASSERT_EQ(scores.size(), 1u);
    EXPECT_DOUBLE_EQ(scores[0], 100.0);
}

TEST_F(MapsComparison, PartialOverlapReturnsBetween0And100) {
    dm::types::MappingBounds b{
        0.0*dm::x_extent[dm::cm], 100.0*dm::x_extent[dm::cm],
        0.0*dm::y_extent[dm::cm], 100.0*dm::y_extent[dm::cm],
        0.0*dm::z_extent[dm::cm], 100.0*dm::z_extent[dm::cm],
    };
    auto m1 = dm::Map3DImpl(b, 10.0*dm::cm, dm::Position3D{});
    auto m2 = dm::Map3DImpl(b, 10.0*dm::cm, dm::Position3D{});

    auto setVoxel = [](dm::Map3DImpl& m, double x, double y, double z) {
        m.set({x*dm::x_extent[dm::cm], y*dm::y_extent[dm::cm], z*dm::z_extent[dm::cm]},
              dm::types::VoxelOccupancy::Occupied);
    };
    setVoxel(m1, 10, 10, 10); setVoxel(m1, 20, 20, 20); setVoxel(m1, 30, 30, 30);
    setVoxel(m2, 10, 10, 10); setVoxel(m2, 40, 40, 40);

    const auto scores = dm::MapsComparison::compare(m1, {&m2});
    ASSERT_EQ(scores.size(), 1u);
    EXPECT_GT(scores[0], 0.0) << "Expected partial overlap > 0";
    EXPECT_LT(scores[0], 100.0) << "Expected partial overlap < 100";
}

TEST_F(MapsComparison, OneEmptyOneNonEmptyReturnsZero) {
    dm::Map3DImpl m1("data_maps/single_voxel_x2_y4_z2.npy", 10.0*dm::cm);
    auto          m2 = emptyMap();
    const auto scores = dm::MapsComparison::compare(m1, {&m2});
    ASSERT_EQ(scores.size(), 1u);
    EXPECT_DOUBLE_EQ(scores[0], 0.0);
}

TEST_F(MapsComparison, MultipleTargetsReturnMultipleScores) {
    dm::Map3DImpl origin("data_maps/single_voxel_x2_y4_z2.npy", 10.0*dm::cm);
    dm::Map3DImpl identical("data_maps/single_voxel_x2_y4_z2.npy", 10.0*dm::cm);
    auto          blank = emptyMap();

    const auto scores = dm::MapsComparison::compare(origin, {&identical, &blank});
    ASSERT_EQ(scores.size(), 2u);
    EXPECT_DOUBLE_EQ(scores[0], 100.0);
    EXPECT_DOUBLE_EQ(scores[1], 0.0);
}

TEST_F(MapsComparison, HigherSimilarityScoresHigher) {
    dm::types::MappingBounds b{
        0.0*dm::x_extent[dm::cm], 100.0*dm::x_extent[dm::cm],
        0.0*dm::y_extent[dm::cm], 100.0*dm::y_extent[dm::cm],
        0.0*dm::z_extent[dm::cm], 100.0*dm::z_extent[dm::cm],
    };
    auto origin  = dm::Map3DImpl(b, 10.0*dm::cm, dm::Position3D{});
    auto close   = dm::Map3DImpl(b, 10.0*dm::cm, dm::Position3D{});
    auto faraway = dm::Map3DImpl(b, 10.0*dm::cm, dm::Position3D{});

    auto setVoxel = [](dm::Map3DImpl& m, double x, double y, double z) {
        m.set({x*dm::x_extent[dm::cm], y*dm::y_extent[dm::cm], z*dm::z_extent[dm::cm]},
              dm::types::VoxelOccupancy::Occupied);
    };

    // Origin has 4 occupied voxels
    setVoxel(origin, 10, 10, 10); setVoxel(origin, 20, 20, 20);
    setVoxel(origin, 30, 30, 30); setVoxel(origin, 40, 40, 40);

    // close shares 3 of 4
    setVoxel(close, 10, 10, 10); setVoxel(close, 20, 20, 20);
    setVoxel(close, 30, 30, 30);

    // faraway shares none
    setVoxel(faraway, 50, 50, 50); setVoxel(faraway, 60, 60, 60);

    const auto scores_close   = dm::MapsComparison::compare(origin, {&close});
    const auto scores_faraway = dm::MapsComparison::compare(origin, {&faraway});
    ASSERT_EQ(scores_close.size(), 1u);
    ASSERT_EQ(scores_faraway.size(), 1u);
    EXPECT_GT(scores_close[0], scores_faraway[0])
        << "More similar map should score higher";
}
