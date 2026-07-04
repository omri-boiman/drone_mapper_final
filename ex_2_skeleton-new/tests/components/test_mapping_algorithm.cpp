#include <gtest/gtest.h>

#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MappingAlgorithmImpl.h>
#include <drone_mapper/types/MapTypes.h>

#include <limits>

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
    MappingAlgorithmImpl algo({}, {}, defaultDrone(), output_map);
    const auto state = stateAt(50, 50, 50);
    const auto cmd = algo.nextStep(state, nullptr);
    EXPECT_EQ(cmd.status, types::AlgorithmStatus::Working);
    EXPECT_TRUE(cmd.movement.has_value());
}

TEST_F(MappingAlgorithm, FirstCallRequestsScan) {
    auto output_map = makeOutputMap();
    MappingAlgorithmImpl algo({}, {}, defaultDrone(), output_map);
    const auto state = stateAt(50, 50, 50);
    const auto cmd = algo.nextStep(state, nullptr);
    EXPECT_TRUE(cmd.scan_orientation.has_value());
}

TEST_F(MappingAlgorithm, MultipleCallsReturnConsistentCommands) {
    auto output_map = makeOutputMap();
    MappingAlgorithmImpl algo({}, {}, defaultDrone(), output_map);
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
    MappingAlgorithmImpl algo({}, {}, defaultDrone(), output_map);
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

TEST_F(MappingAlgorithm, ScanResultWithHitsDoesNotCrash) {
    auto output_map = makeOutputMap();
    MappingAlgorithmImpl algo({}, {}, defaultDrone(), output_map);
    const auto state = stateAt(50, 50, 50);

    // Provide a fake scan result and verify the algorithm handles it without crash
    types::LidarScanResult scan = {
        types::LidarHit{30.0*cm, {0.0*horizontal_angle[deg], 0.0*altitude_angle[deg]}},
        types::LidarHit{std::numeric_limits<double>::max()*cm, {5.0*horizontal_angle[deg], 0.0*altitude_angle[deg]}},
    };
    ASSERT_NO_THROW({
        for (int i = 0; i < 5; ++i) {
            const auto* scan_ptr = (i % 2 == 0) ? &scan : nullptr;
            std::ignore = algo.nextStep(state, scan_ptr);
        }
    });
}

TEST_F(MappingAlgorithm, AlgorithmDoesNotCrashWhenDroneAtBoundaryEdge) {
    types::MappingBounds bounds{
        0.0*x_extent[cm], 100.0*x_extent[cm],
        0.0*y_extent[cm], 100.0*y_extent[cm],
        0.0*z_extent[cm], 100.0*z_extent[cm],
    };
    Map3DImpl output_map(bounds, 10.0*cm);
    MappingAlgorithmImpl algo({}, {}, defaultDrone(), output_map);

    // Drone positioned at the corner of the map boundary
    const auto state = stateAt(0, 0, 0);
    ASSERT_NO_THROW({
        for (int i = 0; i < 20; ++i)
            std::ignore = algo.nextStep(state, nullptr);
    });
}

// --- Behavioral contract tests (targeted for bug detection) ---

// Every Working step must include a scan request so DroneControl fires the lidar.
TEST_F(MappingAlgorithm, ScanOrientationPresentOnEveryWorkingStep) {
    auto output_map = makeOutputMap();
    MappingAlgorithmImpl algo({}, {}, defaultDrone(), output_map);
    const auto state = stateAt(50, 50, 50);
    for (int i = 0; i < 70; ++i) {
        const auto cmd = algo.nextStep(state, nullptr);
        if (cmd.status != types::AlgorithmStatus::Working) break;
        EXPECT_TRUE(cmd.scan_orientation.has_value())
            << "Step " << i << ": scan_orientation missing while Working";
    }
}

// The initial 360° sweep must issue only Right-direction Rotate commands.
// defaultBounds = 20×20×10 voxels → rot_step=5.625° → 64 steps.
TEST_F(MappingAlgorithm, InitialSweepCommandsAreRotateRight) {
    auto output_map = makeOutputMap();
    MappingAlgorithmImpl algo({}, {}, defaultDrone(), output_map);
    const auto state = stateAt(50, 50, 50);

    for (int i = 0; i < 64; ++i) {
        const auto cmd = algo.nextStep(state, nullptr);
        ASSERT_EQ(cmd.status, types::AlgorithmStatus::Working) << "step " << i;
        ASSERT_TRUE(cmd.movement.has_value()) << "step " << i;
        EXPECT_EQ(cmd.movement->type, types::MovementCommandType::Rotate) << "step " << i;
        EXPECT_EQ(cmd.movement->rotation, types::RotationDirection::Right) << "step " << i;
    }
}

// The total rotation issued in the sweep must cover a full 360°.
TEST_F(MappingAlgorithm, SweepCoversFullRotation) {
    auto output_map = makeOutputMap();
    MappingAlgorithmImpl algo({}, {}, defaultDrone(), output_map);
    const auto state = stateAt(50, 50, 50);

    double total_deg = 0.0;
    for (int i = 0; i < 100; ++i) {
        const auto cmd = algo.nextStep(state, nullptr);
        if (!cmd.movement.has_value() ||
            cmd.movement->type != types::MovementCommandType::Rotate) break;
        total_deg += cmd.movement->angle.force_numerical_value_in(deg);
    }
    EXPECT_GE(total_deg, 360.0);
}

// No single rotation command may exceed the drone's max_rotate limit.
TEST_F(MappingAlgorithm, SweepRotationStepsRespectMaxRotate) {
    auto output_map = makeOutputMap();
    MappingAlgorithmImpl algo({}, {}, defaultDrone(), output_map);
    const auto state = stateAt(50, 50, 50);
    constexpr double max_rot_deg = 45.0;  // defaultDrone max_rotate

    for (int i = 0; i < 100; ++i) {
        const auto cmd = algo.nextStep(state, nullptr);
        if (!cmd.movement.has_value() ||
            cmd.movement->type != types::MovementCommandType::Rotate) break;
        EXPECT_LE(cmd.movement->angle.force_numerical_value_in(deg), max_rot_deg + 1e-9)
            << "step " << i;
    }
}

// Maps with ≤20 voxels in every dimension use the fine 5.625° step = 64 rotations.
TEST_F(MappingAlgorithm, SmallMapUses64SweepSteps) {
    types::MappingBounds small{
        0.0*x_extent[cm], 100.0*x_extent[cm],
        0.0*y_extent[cm], 100.0*y_extent[cm],
        0.0*z_extent[cm], 100.0*z_extent[cm],
    };
    Map3DImpl output_map(small, 10.0*cm);  // 10×10×10 voxels → 5.625° → 64 steps
    MappingAlgorithmImpl algo({}, {}, defaultDrone(), output_map);
    const auto state = stateAt(50, 50, 50);

    int count = 0;
    for (int i = 0; i < 100; ++i) {
        const auto cmd = algo.nextStep(state, nullptr);
        if (!cmd.movement.has_value() ||
            cmd.movement->type != types::MovementCommandType::Rotate) break;
        ++count;
    }
    EXPECT_EQ(count, 64);
}

// Maps with >20 voxels in any dimension use the coarse 11.25° step = 32 rotations.
TEST_F(MappingAlgorithm, LargeMapUses32SweepSteps) {
    types::MappingBounds large{
        0.0*x_extent[cm], 250.0*x_extent[cm],
        0.0*y_extent[cm], 250.0*y_extent[cm],
        0.0*z_extent[cm], 100.0*z_extent[cm],
    };
    Map3DImpl output_map(large, 10.0*cm);  // 25×25×10 voxels → 11.25° → 32 steps
    MappingAlgorithmImpl algo({}, {}, defaultDrone(), output_map);
    const auto state = stateAt(50, 50, 50);

    int count = 0;
    for (int i = 0; i < 100; ++i) {
        const auto cmd = algo.nextStep(state, nullptr);
        if (!cmd.movement.has_value() ||
            cmd.movement->type != types::MovementCommandType::Rotate) break;
        ++count;
    }
    EXPECT_EQ(count, 32);
}

// Once the algorithm signals done, every subsequent call must return Finished
// with no movement and no scan (the done_ flag must be sticky).
TEST_F(MappingAlgorithm, SubsequentCallsAfterDoneReturnFinished) {
    types::MappingBounds tiny{
        0.0*x_extent[cm], 50.0*x_extent[cm],
        0.0*y_extent[cm], 50.0*y_extent[cm],
        0.0*z_extent[cm], 50.0*z_extent[cm],
    };
    Map3DImpl output_map(tiny, 50.0*cm);
    MappingAlgorithmImpl algo({}, {}, defaultDrone(), output_map);
    const auto state = stateAt(25, 25, 25);

    for (int i = 0; i < 500; ++i) {
        if (algo.nextStep(state, nullptr).status != types::AlgorithmStatus::Working) break;
    }

    for (int i = 0; i < 3; ++i) {
        const auto cmd = algo.nextStep(state, nullptr);
        EXPECT_EQ(cmd.status, types::AlgorithmStatus::Finished) << "call " << i;
        EXPECT_FALSE(cmd.movement.has_value()) << "call " << i;
        EXPECT_FALSE(cmd.scan_orientation.has_value()) << "call " << i;
    }
}
