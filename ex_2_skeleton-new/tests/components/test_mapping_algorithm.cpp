#include <gtest/gtest.h>

#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MappingAlgorithmImpl.h>
#include <drone_mapper/types/MapTypes.h>

using namespace drone_mapper;

namespace {

types::MappingBounds defaultBounds() {
    return {
        0.0*x_extent[cm], 200.0*x_extent[cm],
        0.0*y_extent[cm], 200.0*y_extent[cm],
        0.0*z_extent[cm], 100.0*z_extent[cm],
    };
}

types::DroneConfigData defaultDrone() {
    // radius = 15cm (was dimensions=30cm / 2 in old design)
    return {15.0*cm, 45.0*horizontal_angle[deg], 50.0*cm, 40.0*cm};
}

types::DroneState stateAt(double x, double y, double z, double heading_deg = 0.0) {
    return {
        {x*x_extent[cm], y*y_extent[cm], z*z_extent[cm]},
        {heading_deg*horizontal_angle[deg], 0.0*altitude_angle[deg]},
        0,
    };
}

Map3DImpl makeOutputMap() {
    return Map3DImpl(defaultBounds(), 10.0*cm);
}

} // namespace

class MappingAlgorithm : public ::testing::Test {};

TEST_F(MappingAlgorithm, FirstCallReturnsWorkingStatus) {
    auto output_map = makeOutputMap();
    MappingAlgorithmImpl algo(defaultDrone(), output_map);
    const auto state = stateAt(50, 50, 50);
    const auto cmd = algo.nextStep(state, nullptr);
    EXPECT_EQ(cmd.status, types::AlgorithmStatus::Working);
    EXPECT_TRUE(cmd.movement.has_value());
}

TEST_F(MappingAlgorithm, FirstCallRequestsScan) {
    auto output_map = makeOutputMap();
    MappingAlgorithmImpl algo(defaultDrone(), output_map);
    const auto state = stateAt(50, 50, 50);
    const auto cmd = algo.nextStep(state, nullptr);
    EXPECT_TRUE(cmd.scan_orientation.has_value());
}

TEST_F(MappingAlgorithm, MultipleCallsReturnConsistentCommands) {
    auto output_map = makeOutputMap();
    MappingAlgorithmImpl algo(defaultDrone(), output_map);
    const auto state = stateAt(50, 50, 50);
    // First several calls should be rotation scan steps
    for (int i = 0; i < 8; ++i) {
        const auto cmd = algo.nextStep(state, nullptr);
        EXPECT_EQ(cmd.status, types::AlgorithmStatus::Working)
            << "Expected Working status at step " << i;
        ASSERT_TRUE(cmd.movement.has_value()) << "Expected movement at step " << i;
        EXPECT_EQ(cmd.movement->type, types::MovementCommandType::Rotate)
            << "Expected Rotate during 360° scan phase at step " << i;
    }
}

TEST_F(MappingAlgorithm, EventuallyFinishesOnSmallMap) {
    types::MappingBounds tiny_bounds{
        0.0*x_extent[cm], 50.0*x_extent[cm],
        0.0*y_extent[cm], 50.0*y_extent[cm],
        0.0*z_extent[cm], 50.0*z_extent[cm],
    };
    Map3DImpl output_map(tiny_bounds, 50.0*cm);
    MappingAlgorithmImpl algo(defaultDrone(), output_map);
    const auto state = stateAt(25, 25, 25);

    bool finished = false;
    for (int i = 0; i < 1000; ++i) {
        const auto cmd = algo.nextStep(state, nullptr);
        if (cmd.status == types::AlgorithmStatus::Finished ||
            cmd.status == types::AlgorithmStatus::FinishedWithUnmappableVoxels) {
            finished = true;
            break;
        }
    }
    EXPECT_TRUE(finished) << "Algorithm did not finish within 1000 steps on tiny map";
}
