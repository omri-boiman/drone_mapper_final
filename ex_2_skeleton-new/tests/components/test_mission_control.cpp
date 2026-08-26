#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <drone_mapper/IDroneControl.h>
#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MissionControlImpl.h>

#include <filesystem>
#include <fstream>

using namespace drone_mapper;
using ::testing::Return;
using ::testing::_;

class MockDroneCtrl : public IDroneControl {
public:
    MOCK_METHOD(types::DroneStepResult, step, (), (override));
    MOCK_METHOD(types::DroneState, state, (), (const, override));
};

namespace {

types::MissionConfigData mission(std::size_t max_steps = 100) {
    return {max_steps, 10.0 * cm, 1};
}

types::DroneConfigData drone() {
    return {30.0*cm, 45.0*horizontal_angle[deg], 50.0*cm, 40.0*cm};
}

Map3DImpl makeHiddenMap() {
    return Map3DImpl("data_maps/single_voxel_x2_y4_z2.npy", 10.0*cm);
}

Map3DImpl makeOutputMap() {
    types::MappingBounds b{
        0.0*x_extent[cm], 200.0*x_extent[cm],
        0.0*y_extent[cm], 200.0*y_extent[cm],
        0.0*z_extent[cm], 200.0*z_extent[cm],
    };
    return Map3DImpl(b, 10.0*cm, Position3D{});
}

} // namespace

class MissionControl : public ::testing::Test {};

TEST_F(MissionControl, CompletesWhenDroneSignalsCompleted) {
    auto hidden = makeHiddenMap();
    auto output = makeOutputMap();
    MockDroneCtrl ctrl;

    int call_count = 0;
    ON_CALL(ctrl, step()).WillByDefault([&]() -> types::DroneStepResult {
        ++call_count;
        if (call_count >= 5) return {types::DroneStepStatus::Completed};
        return {types::DroneStepStatus::Continue};
    });
    ON_CALL(ctrl, state()).WillByDefault(Return(types::DroneState{}));

    const std::filesystem::path out = "/tmp/test_mission_completed.npy";
    MissionControlImpl mc(mission(), drone(), hidden, output, ctrl, out);
    const auto result = mc.runMission();

    EXPECT_EQ(result.status, types::MissionRunStatus::Completed);
    EXPECT_EQ(result.steps, 4u);
}

TEST_F(MissionControl, StopsAtMaxSteps) {
    auto hidden = makeHiddenMap();
    auto output = makeOutputMap();
    MockDroneCtrl ctrl;

    ON_CALL(ctrl, step()).WillByDefault(Return(types::DroneStepResult{types::DroneStepStatus::Continue}));
    ON_CALL(ctrl, state()).WillByDefault(Return(types::DroneState{}));

    const std::filesystem::path out = "/tmp/test_mission_maxsteps.npy";
    MissionControlImpl mc(mission(10), drone(), hidden, output, ctrl, out);
    const auto result = mc.runMission();

    EXPECT_EQ(result.status, types::MissionRunStatus::MaxSteps);
    EXPECT_EQ(result.steps, 10u);
}

TEST_F(MissionControl, ReturnsErrorStatusAndCodeOnError) {
    auto hidden = makeHiddenMap();
    auto output = makeOutputMap();
    MockDroneCtrl ctrl;

    ON_CALL(ctrl, step()).WillByDefault(Return(
        types::DroneStepResult{types::DroneStepStatus::Error, "DRONE_HITS_OBSTACLE"}));
    ON_CALL(ctrl, state()).WillByDefault(Return(types::DroneState{}));

    const std::filesystem::path out = "/tmp/test_mission_error.npy";
    MissionControlImpl mc(mission(), drone(), hidden, output, ctrl, out);
    const auto result = mc.runMission();

    EXPECT_EQ(result.status, types::MissionRunStatus::Error);
    ASSERT_FALSE(result.errors.empty());
    EXPECT_EQ(result.errors[0].code, "DRONE_HITS_OBSTACLE");
}

TEST_F(MissionControl, SavesOutputMapFile) {
    auto hidden = makeHiddenMap();
    auto output = makeOutputMap();
    MockDroneCtrl ctrl;

    ON_CALL(ctrl, step()).WillByDefault(Return(types::DroneStepResult{types::DroneStepStatus::Completed}));
    ON_CALL(ctrl, state()).WillByDefault(Return(types::DroneState{}));

    const std::filesystem::path out = "/tmp/test_mission_saves.npy";
    std::filesystem::remove(out);
    MissionControlImpl mc(mission(), drone(), hidden, output, ctrl, out);
    std::ignore = mc.runMission();

    EXPECT_TRUE(std::filesystem::exists(out));
}

TEST_F(MissionControl, SavesOutputMapEvenOnError) {
    auto hidden = makeHiddenMap();
    auto output = makeOutputMap();
    MockDroneCtrl ctrl;

    ON_CALL(ctrl, step()).WillByDefault(Return(
        types::DroneStepResult{types::DroneStepStatus::Error, "DRONE_HITS_OBSTACLE"}));
    ON_CALL(ctrl, state()).WillByDefault(Return(types::DroneState{}));

    const std::filesystem::path out = "/tmp/test_mission_error_saves.npy";
    std::filesystem::remove(out);
    MissionControlImpl mc(mission(), drone(), hidden, output, ctrl, out);
    std::ignore = mc.runMission();

    EXPECT_TRUE(std::filesystem::exists(out)) << "Output map must be saved even on error";
}

TEST_F(MissionControl, ImmediateCompleteGivesZeroSteps) {
    auto hidden = makeHiddenMap();
    auto output = makeOutputMap();
    MockDroneCtrl ctrl;

    ON_CALL(ctrl, step()).WillByDefault(Return(types::DroneStepResult{types::DroneStepStatus::Completed}));
    ON_CALL(ctrl, state()).WillByDefault(Return(types::DroneState{}));

    MissionControlImpl mc(mission(), drone(), hidden, output, ctrl, "/tmp/mc_zero_steps.npy");
    const auto result = mc.runMission();

    EXPECT_EQ(result.status, types::MissionRunStatus::Completed);
    EXPECT_EQ(result.steps, 0u);
}
