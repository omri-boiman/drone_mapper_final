#include <gtest/gtest.h>

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

types::MissionConfigData defaultMission() {
    return {100, 10.0*cm, 1};
}

types::DroneConfigData defaultDrone() {
    return {30.0*cm, 45.0*horizontal_angle[deg], 50.0*cm, 40.0*cm};
}

types::DroneState stateAt(double x, double y, double z, double heading_deg = 0.0) {
    return {
        {x*x_extent[cm], y*y_extent[cm], z*z_extent[cm]},
        {heading_deg*horizontal_angle[deg], 0.0*altitude_angle[deg]},
        0,
    };
}

} // namespace

class MappingAlgorithm : public ::testing::Test {};

TEST_F(MappingAlgorithm, FirstCallReturnsNonHoverCommand) {
    MappingAlgorithmImpl algo(defaultMission(), defaultDrone(), defaultBounds());
    const auto state = stateAt(50, 50, 50);
    const auto cmd = algo.nextMove(state, {});
    EXPECT_NE(cmd.type, types::MovementCommandType::Hover);
}

TEST_F(MappingAlgorithm, ApplyVoxelUpdatesDoesNotCrash) {
    MappingAlgorithmImpl algo(defaultMission(), defaultDrone(), defaultBounds());
    std::vector<types::MappedVoxel> voxels{
        {{10.0*x_extent[cm], 10.0*y_extent[cm], 10.0*z_extent[cm]}, types::VoxelOccupancy::Occupied},
        {{20.0*x_extent[cm], 20.0*y_extent[cm], 20.0*z_extent[cm]}, types::VoxelOccupancy::Empty},
    };
    EXPECT_NO_THROW(algo.applyVoxelUpdates(voxels));
}

TEST_F(MappingAlgorithm, MultipleCallsReturnConsistentCommands) {
    MappingAlgorithmImpl algo(defaultMission(), defaultDrone(), defaultBounds());
    const auto state = stateAt(50, 50, 50);
    for (int i = 0; i < 8; ++i) {
        const auto cmd = algo.nextMove(state, {});
        EXPECT_EQ(cmd.type, types::MovementCommandType::Rotate)
            << "Expected Rotate during 360° scan phase at step " << i;
    }
}

TEST_F(MappingAlgorithm, EventuallyReturnsHoverWhenObstaclesBlockAll) {
    types::MappingBounds tiny_bounds{
        0.0*x_extent[cm], 50.0*x_extent[cm],
        0.0*y_extent[cm], 50.0*y_extent[cm],
        0.0*z_extent[cm], 50.0*z_extent[cm],
    };
    types::MissionConfigData tiny_mission{200, 50.0*cm, 1};
    MappingAlgorithmImpl algo(tiny_mission, defaultDrone(), tiny_bounds);
    const auto state = stateAt(25, 25, 25);

    std::vector<types::MappedVoxel> obstacles;
    for (double x : {0.0, 50.0}) {
        for (double y : {0.0, 50.0}) {
            obstacles.push_back({{x*x_extent[cm], y*y_extent[cm], 25.0*z_extent[cm]},
                                  types::VoxelOccupancy::Occupied});
        }
    }
    algo.applyVoxelUpdates(obstacles);

    bool got_hover = false;
    for (int i = 0; i < 1000; ++i) {
        const auto cmd = algo.nextMove(state, {});
        if (cmd.type == types::MovementCommandType::Hover) { got_hover = true; break; }
    }
    EXPECT_TRUE(got_hover) << "Algorithm did not signal Hover within 1000 steps";
}
