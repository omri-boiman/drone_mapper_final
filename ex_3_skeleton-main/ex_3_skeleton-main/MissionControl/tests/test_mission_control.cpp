#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <Common/IDroneMovement.h>
#include <Common/IGPS.h>
#include <Common/ILidar.h>
#include <Common/IMappingAlgorithm.h>
#include <Common/IMutableMap3D.h>
#include <MissionControl/MissionControlImpl.h>

#include "TestMap3D.h"

#include <filesystem>
#include <fstream>
#include <sstream>

// MissionControlImpl_211781141_325049575 builds its own internal DroneControlImpl
// from raw sensor/actuator/algorithm dependencies now (IDroneControl is no
// longer injectable from outside -- see project notes), so these tests mock
// at that lower level instead of mocking IDroneControl directly like ex2 did.
// The algorithm mock is what drives Completed/MaxSteps/Error scenarios: a
// Finished status flows through the real DroneControlImpl into
// DroneStepStatus::Completed, and a failing movement call flows into
// DroneStepStatus::Error, exactly like ex2's DroneControlImpl-level tests.

using namespace common;
using namespace mission_control_211781141_325049575;
using mission_control_211781141_325049575::test::TestMap3D;
using ::testing::Return;
using ::testing::_;

class MockGPS_MC : public IGPS {
public:
    MOCK_METHOD(Position3D,  position, (), (const, override));
    MOCK_METHOD(Orientation, heading,  (), (const, override));
};

class MockLidar_MC : public ILidar {
public:
    MOCK_METHOD(types::LidarScanResult, scan, (Orientation), (const, override));
    MOCK_METHOD(types::LidarConfigData, config, (), (const, override));
};

class MockMovement_MC : public IDroneMovement {
public:
    MOCK_METHOD(types::MovementResult, rotate,
                (types::RotationDirection, HorizontalAngle), (override));
    MOCK_METHOD(types::MovementResult, advance, (PhysicalLength), (override));
    MOCK_METHOD(types::MovementResult, elevate, (PhysicalLength), (override));
};

class MockAlgorithm_MC : public IMappingAlgorithm {
public:
    struct NullMap : IMap3D {
        types::VoxelOccupancy atVoxel(const Position3D&) const override {
            return types::VoxelOccupancy::Empty;
        }
        types::MapConfig getMapConfig() const override { return {}; }
        bool isInBounds(const Position3D&) const override { return false; }
    };
    MockAlgorithm_MC()
        : IMappingAlgorithm(MappingAlgorithmDependencies{
              types::MissionConfigData{}, types::LidarConfigData{}, types::DroneConfigData{}, null_map_}) {}
    MOCK_METHOD(types::MappingStepCommand, nextStep,
                (const types::DroneState&, const types::LidarScanResult*), (override));
private:
    NullMap null_map_;
};

namespace {

types::MissionConfigData mission(std::size_t max_steps = 100) {
    return {max_steps, 10.0 * cm, 1};
}

types::DroneConfigData drone() {
    return {30.0*cm, 45.0*horizontal_angle[deg], 50.0*cm, 40.0*cm};
}

TestMap3D makeOutputMap() {
    types::MappingBounds b{
        0.0*x_extent[cm], 200.0*x_extent[cm],
        0.0*y_extent[cm], 200.0*y_extent[cm],
        0.0*z_extent[cm], 200.0*z_extent[cm],
    };
    return TestMap3D(b, 10.0*cm, Position3D{});
}

types::MappingStepCommand workingNoOp() {
    return {std::nullopt, std::nullopt, types::AlgorithmStatus::Working};
}

types::MappingStepCommand finished() {
    return {std::nullopt, std::nullopt, types::AlgorithmStatus::Finished};
}

} // namespace

class MissionControl : public ::testing::Test {
protected:
    void SetUp() override {
        ON_CALL(gps, position()).WillByDefault(Return(Position3D{}));
        ON_CALL(gps, heading()).WillByDefault(Return(Orientation{}));
        ON_CALL(lidar, config()).WillByDefault(Return(types::LidarConfigData{}));
        ON_CALL(movement, rotate(_, _)).WillByDefault(Return(types::MovementResult{true}));
        ON_CALL(movement, advance(_)).WillByDefault(Return(types::MovementResult{true}));
        ON_CALL(movement, elevate(_)).WillByDefault(Return(types::MovementResult{true}));
    }

    MissionControlDependencies depsFor(TestMap3D& output_map,
                                       std::filesystem::path out_file,
                                       std::size_t max_steps = 100) {
        mission_ = mission(max_steps);
        drone_ = drone();
        return MissionControlDependencies{
            mission_, drone_, lidar, gps, movement, output_map, algo,
            std::move(out_file), false};
    }

    MockGPS_MC       gps;
    MockLidar_MC     lidar;
    MockMovement_MC  movement;
    MockAlgorithm_MC algo;
    types::MissionConfigData mission_;
    types::DroneConfigData drone_;
};

TEST_F(MissionControl, CompletesWhenAlgorithmSignalsFinished) {
    auto output = makeOutputMap();
    int call_count = 0;
    ON_CALL(algo, nextStep(_, _)).WillByDefault([&](const auto&, const auto*) {
        ++call_count;
        return call_count >= 5 ? finished() : workingNoOp();
    });

    MissionControlImpl_211781141_325049575 mc(
        depsFor(output, "/tmp/test_mission_completed.npy"));
    const auto result = mc.runMission();

    EXPECT_EQ(result.status, types::MissionRunStatus::Completed);
    EXPECT_EQ(result.steps, 4u);
}

TEST_F(MissionControl, StopsAtMaxSteps) {
    auto output = makeOutputMap();
    ON_CALL(algo, nextStep(_, _)).WillByDefault(Return(workingNoOp()));

    MissionControlImpl_211781141_325049575 mc(
        depsFor(output, "/tmp/test_mission_maxsteps.npy", 10));
    const auto result = mc.runMission();

    EXPECT_EQ(result.status, types::MissionRunStatus::MaxSteps);
    EXPECT_EQ(result.steps, 10u);
}

TEST_F(MissionControl, ReturnsErrorStatusAndCodeOnError) {
    auto output = makeOutputMap();
    ON_CALL(algo, nextStep(_, _)).WillByDefault(Return(types::MappingStepCommand{
        types::MovementCommand{types::MovementCommandType::Advance,
                               types::RotationDirection::Left, 0.0*horizontal_angle[deg], 30.0*cm},
        std::nullopt, types::AlgorithmStatus::Working}));
    ON_CALL(movement, advance(_)).WillByDefault(
        Return(types::MovementResult{false, "DRONE_HITS_OBSTACLE"}));

    MissionControlImpl_211781141_325049575 mc(
        depsFor(output, "/tmp/test_mission_error.npy"));
    const auto result = mc.runMission();

    EXPECT_EQ(result.status, types::MissionRunStatus::Error);
    ASSERT_FALSE(result.errors.empty());
    EXPECT_EQ(result.errors[0].code, "DRONE_HITS_OBSTACLE");
}

TEST_F(MissionControl, SavesOutputMapFile) {
    auto output = makeOutputMap();
    ON_CALL(algo, nextStep(_, _)).WillByDefault(Return(finished()));

    const std::filesystem::path out = "/tmp/test_mission_saves.npy";
    std::filesystem::remove(out);
    MissionControlImpl_211781141_325049575 mc(depsFor(output, out));
    std::ignore = mc.runMission();

    EXPECT_TRUE(std::filesystem::exists(out));
}

TEST_F(MissionControl, SavesOutputMapEvenOnError) {
    auto output = makeOutputMap();
    ON_CALL(algo, nextStep(_, _)).WillByDefault(Return(types::MappingStepCommand{
        types::MovementCommand{types::MovementCommandType::Advance,
                               types::RotationDirection::Left, 0.0*horizontal_angle[deg], 30.0*cm},
        std::nullopt, types::AlgorithmStatus::Working}));
    ON_CALL(movement, advance(_)).WillByDefault(
        Return(types::MovementResult{false, "DRONE_HITS_OBSTACLE"}));

    const std::filesystem::path out = "/tmp/test_mission_error_saves.npy";
    std::filesystem::remove(out);
    MissionControlImpl_211781141_325049575 mc(depsFor(output, out));
    std::ignore = mc.runMission();

    EXPECT_TRUE(std::filesystem::exists(out)) << "Output map must be saved even on error";
}

TEST_F(MissionControl, ImmediateCompleteGivesZeroSteps) {
    auto output = makeOutputMap();
    ON_CALL(algo, nextStep(_, _)).WillByDefault(Return(finished()));

    MissionControlImpl_211781141_325049575 mc(
        depsFor(output, "/tmp/mc_zero_steps.npy"));
    const auto result = mc.runMission();

    EXPECT_EQ(result.status, types::MissionRunStatus::Completed);
    EXPECT_EQ(result.steps, 0u);
}

// ---- -verbose output: previously only checked by hand (see
// memory-ex3-status.md's "-verbose output enriched" session note). These
// lock down the log's structure and, more importantly, that turning
// -verbose on never changes the mission's actual result -- the property the
// manual check cared about most.

TEST_F(MissionControl, VerboseFlag_ProducesHeaderStepsAndFooter) {
    auto output = makeOutputMap();
    int call_count = 0;
    ON_CALL(algo, nextStep(_, _)).WillByDefault([&](const auto&, const auto*) {
        ++call_count;
        return call_count >= 3 ? finished() : workingNoOp();
    });

    const std::filesystem::path out = "/tmp/mc_verbose_test.npy";
    const std::filesystem::path log = "/tmp/mc_verbose_test_verbose.log";
    std::filesystem::remove(log);

    mission_ = mission(100);
    drone_ = drone();
    MissionControlDependencies deps{
        mission_, drone_, lidar, gps, movement, output, algo, out, /*verbose=*/true};
    MissionControlImpl_211781141_325049575 mc(std::move(deps));
    const auto result = mc.runMission();

    ASSERT_TRUE(std::filesystem::exists(log)) << "no verbose log created at " << log;
    std::ifstream in(log);
    std::ostringstream contents;
    contents << in.rdbuf();
    const std::string text = contents.str();

    EXPECT_NE(text.find("=== mission start"), std::string::npos) << text;
    EXPECT_NE(text.find("max_steps="), std::string::npos) << text;
    EXPECT_NE(text.find("step="), std::string::npos) << text;
    EXPECT_NE(text.find("moved_cm="), std::string::npos) << text;
    EXPECT_NE(text.find("elapsed_ms="), std::string::npos) << text;
    EXPECT_NE(text.find("=== mission end"), std::string::npos) << text;
    EXPECT_NE(text.find("total_steps="), std::string::npos) << text;
    EXPECT_EQ(result.status, types::MissionRunStatus::Completed);
    EXPECT_EQ(result.steps, 2u);
}

TEST_F(MissionControl, VerboseFlag_DoesNotAffectMissionResult) {
    // Same scenario run twice, once with -verbose off and once on: the
    // returned MissionRunResult (status + steps) must be bit-for-bit
    // identical -- instrumentation must be side-effect-free on behavior.
    auto runScenario = [&](bool verbose) {
        MockGPS_MC local_gps;
        MockLidar_MC local_lidar;
        MockMovement_MC local_movement;
        MockAlgorithm_MC local_algo;
        ON_CALL(local_gps, position()).WillByDefault(Return(Position3D{}));
        ON_CALL(local_gps, heading()).WillByDefault(Return(Orientation{}));
        ON_CALL(local_lidar, config()).WillByDefault(Return(types::LidarConfigData{}));
        ON_CALL(local_movement, rotate(_, _)).WillByDefault(Return(types::MovementResult{true}));
        ON_CALL(local_movement, advance(_)).WillByDefault(Return(types::MovementResult{true}));
        ON_CALL(local_movement, elevate(_)).WillByDefault(Return(types::MovementResult{true}));
        int call_count = 0;
        ON_CALL(local_algo, nextStep(_, _)).WillByDefault([&](const auto&, const auto*) {
            ++call_count;
            return call_count >= 4 ? finished() : workingNoOp();
        });

        auto output = makeOutputMap();
        auto mission_cfg = mission(100);
        auto drone_cfg = drone();
        const std::string suffix = verbose ? "_verbose" : "_plain";
        MissionControlDependencies deps{
            mission_cfg, drone_cfg, local_lidar, local_gps, local_movement, output, local_algo,
            std::filesystem::path("/tmp/mc_verbose_parity" + suffix + ".npy"), verbose};
        MissionControlImpl_211781141_325049575 mc(std::move(deps));
        return mc.runMission();
    };

    const auto plain_result = runScenario(false);
    const auto verbose_result = runScenario(true);

    EXPECT_EQ(plain_result.status, verbose_result.status);
    EXPECT_EQ(plain_result.steps, verbose_result.steps);
}
