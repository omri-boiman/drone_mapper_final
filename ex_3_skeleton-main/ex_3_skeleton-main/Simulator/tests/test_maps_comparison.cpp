#include <gtest/gtest.h>

#include <Simulator/Map3DImpl.h>
#include <Simulator/MapsComparison.h>

namespace sim = simulator;

namespace {

sim::Map3DImpl emptyMap(double res_cm = 10.0) {
    common::types::MappingBounds bounds{
        0.0*sim::x_extent[sim::cm], 200.0*sim::x_extent[sim::cm],
        0.0*sim::y_extent[sim::cm], 200.0*sim::y_extent[sim::cm],
        0.0*sim::z_extent[sim::cm], 200.0*sim::z_extent[sim::cm],
    };
    return sim::Map3DImpl(bounds, res_cm*sim::cm, sim::Position3D{});
}

} // namespace

class MapsComparison : public ::testing::Test {};

TEST_F(MapsComparison, IdenticalFilesMapsReturn100) {
    sim::Map3DImpl m1("data_maps/single_voxel_x2_y4_z2.npy", 10.0*sim::cm);
    sim::Map3DImpl m2("data_maps/single_voxel_x2_y4_z2.npy", 10.0*sim::cm);
    const auto scores = sim::MapsComparison::compare(m1, {&m2});
    ASSERT_EQ(scores.size(), 1u);
    EXPECT_DOUBLE_EQ(scores[0], 100.0);
}

TEST_F(MapsComparison, TwoDifferentMapsReturnZero) {
    sim::Map3DImpl m1("data_maps/single_voxel_x2_y4_z2.npy", 10.0*sim::cm);
    sim::Map3DImpl m2("data_maps/single_voxel_x4_y4_z4.npy", 10.0*sim::cm);
    const auto scores = sim::MapsComparison::compare(m1, {&m2});
    ASSERT_EQ(scores.size(), 1u);
    EXPECT_EQ(scores[0], 0.0);
}

TEST_F(MapsComparison, BothEmptyOutputMapsReturn100) {
    auto m1 = emptyMap();
    auto m2 = emptyMap();
    const auto scores = sim::MapsComparison::compare(m1, {&m2});
    ASSERT_EQ(scores.size(), 1u);
    EXPECT_DOUBLE_EQ(scores[0], 100.0);
}

TEST_F(MapsComparison, PartialOverlapReturnsBetween0And100) {
    common::types::MappingBounds b{
        0.0*sim::x_extent[sim::cm], 100.0*sim::x_extent[sim::cm],
        0.0*sim::y_extent[sim::cm], 100.0*sim::y_extent[sim::cm],
        0.0*sim::z_extent[sim::cm], 100.0*sim::z_extent[sim::cm],
    };
    auto m1 = sim::Map3DImpl(b, 10.0*sim::cm, sim::Position3D{});
    auto m2 = sim::Map3DImpl(b, 10.0*sim::cm, sim::Position3D{});

    auto setVoxel = [](sim::Map3DImpl& m, double x, double y, double z) {
        m.set({x*sim::x_extent[sim::cm], y*sim::y_extent[sim::cm], z*sim::z_extent[sim::cm]},
              common::types::VoxelOccupancy::Occupied);
    };
    setVoxel(m1, 10, 10, 10); setVoxel(m1, 20, 20, 20); setVoxel(m1, 30, 30, 30);
    setVoxel(m2, 10, 10, 10); setVoxel(m2, 40, 40, 40);

    const auto scores = sim::MapsComparison::compare(m1, {&m2});
    ASSERT_EQ(scores.size(), 1u);
    EXPECT_GT(scores[0], 0.0) << "Expected partial overlap > 0";
    EXPECT_LT(scores[0], 100.0) << "Expected partial overlap < 100";
}

TEST_F(MapsComparison, OneEmptyOneNonEmptyReturnsZero) {
    sim::Map3DImpl m1("data_maps/single_voxel_x2_y4_z2.npy", 10.0*sim::cm);
    auto          m2 = emptyMap();
    const auto scores = sim::MapsComparison::compare(m1, {&m2});
    ASSERT_EQ(scores.size(), 1u);
    EXPECT_DOUBLE_EQ(scores[0], 0.0);
}

TEST_F(MapsComparison, MultipleTargetsReturnMultipleScores) {
    sim::Map3DImpl origin("data_maps/single_voxel_x2_y4_z2.npy", 10.0*sim::cm);
    sim::Map3DImpl identical("data_maps/single_voxel_x2_y4_z2.npy", 10.0*sim::cm);
    auto          blank = emptyMap();

    const auto scores = sim::MapsComparison::compare(origin, {&identical, &blank});
    ASSERT_EQ(scores.size(), 2u);
    EXPECT_DOUBLE_EQ(scores[0], 100.0);
    EXPECT_DOUBLE_EQ(scores[1], 0.0);
}

TEST_F(MapsComparison, HigherSimilarityScoresHigher) {
    common::types::MappingBounds b{
        0.0*sim::x_extent[sim::cm], 100.0*sim::x_extent[sim::cm],
        0.0*sim::y_extent[sim::cm], 100.0*sim::y_extent[sim::cm],
        0.0*sim::z_extent[sim::cm], 100.0*sim::z_extent[sim::cm],
    };
    auto origin  = sim::Map3DImpl(b, 10.0*sim::cm, sim::Position3D{});
    auto close   = sim::Map3DImpl(b, 10.0*sim::cm, sim::Position3D{});
    auto faraway = sim::Map3DImpl(b, 10.0*sim::cm, sim::Position3D{});

    auto setVoxel = [](sim::Map3DImpl& m, double x, double y, double z) {
        m.set({x*sim::x_extent[sim::cm], y*sim::y_extent[sim::cm], z*sim::z_extent[sim::cm]},
              common::types::VoxelOccupancy::Occupied);
    };

    // Origin has 4 occupied voxels
    setVoxel(origin, 10, 10, 10); setVoxel(origin, 20, 20, 20);
    setVoxel(origin, 30, 30, 30); setVoxel(origin, 40, 40, 40);

    // close shares 3 of 4
    setVoxel(close, 10, 10, 10); setVoxel(close, 20, 20, 20);
    setVoxel(close, 30, 30, 30);

    // faraway shares none
    setVoxel(faraway, 50, 50, 50); setVoxel(faraway, 60, 60, 60);

    const auto scores_close   = sim::MapsComparison::compare(origin, {&close});
    const auto scores_faraway = sim::MapsComparison::compare(origin, {&faraway});
    ASSERT_EQ(scores_close.size(), 1u);
    ASSERT_EQ(scores_faraway.size(), 1u);
    EXPECT_GT(scores_close[0], scores_faraway[0])
        << "More similar map should score higher";
}

TEST_F(MapsComparison, NullTargetScoresMinusOneWithoutCrashing) {
    sim::Map3DImpl origin("data_maps/single_voxel_x2_y4_z2.npy", 10.0*sim::cm);
    auto blank = emptyMap();

    const auto scores = sim::MapsComparison::compare(origin, {nullptr, &blank});
    ASSERT_EQ(scores.size(), 2u);
    EXPECT_DOUBLE_EQ(scores[0], -1.0) << "A null target must score -1.0, not dereference";
    EXPECT_DOUBLE_EQ(scores[1], 0.0);
}

TEST_F(MapsComparison, MoreThanTwoTargetsAreAllCompared) {
    sim::Map3DImpl origin("data_maps/single_voxel_x2_y4_z2.npy", 10.0*sim::cm);
    sim::Map3DImpl identical("data_maps/single_voxel_x2_y4_z2.npy", 10.0*sim::cm);
    auto blank1 = emptyMap();
    auto blank2 = emptyMap();

    const auto scores = sim::MapsComparison::compare(origin, {&identical, &blank1, &blank2});
    ASSERT_EQ(scores.size(), 3u) << "All targets must be compared, including the last one";
    EXPECT_DOUBLE_EQ(scores[0], 100.0);
    EXPECT_DOUBLE_EQ(scores[1], 0.0);
    EXPECT_DOUBLE_EQ(scores[2], 0.0);
}
