#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <drone_mapper/IDroneControl.h>
#include <drone_mapper/ILidar.h>
#include <drone_mapper/IMappingAlgorithm.h>
#include <drone_mapper/IMissionControl.h>
#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MockGPS.h>
#include <drone_mapper/MockMovement.h>
#include <drone_mapper/SimulationRunImpl.h>

using namespace drone_mapper;
using ::testing::Return;
using ::testing::_;

class MockMissionCtrl : public IMissionControl {
public:
    MOCK_METHOD(types::MissionRunResult, runMission, (), (override));
};

class StubLidar : public ILidar {
public:
    types::LidarScanResult scan(Orientation) const override { return {}; }
};

class StubAlgorithm : public IMappingAlgorithm {
public:
    struct NullMap : IMap3D {
        types::VoxelOccupancy atVoxel(const Position3D&) const override {
            return types::VoxelOccupancy::Empty;
        }
        types::MapConfig getMapConfig() const override { return {}; }
        bool isInBounds(const Position3D&) const override { return false; }
    };
    StubAlgorithm() : IMappingAlgorithm(types::MissionConfigData{}, types::LidarConfigData{}, types::DroneConfigData{}, null_map_) {}
    types::MappingStepCommand nextStep(const types::DroneState&,
                                       const types::LidarScanResult*) override {
        return {std::nullopt, std::nullopt, types::AlgorithmStatus::Finished};
    }
private:
    NullMap null_map_;
};

class StubDroneCtrl : public IDroneControl {
public:
    types::DroneStepResult step() override { return {types::DroneStepStatus::Completed}; }
    types::DroneState      state() const override { return {}; }
};

namespace {

types::SimulationConfigData defaultSimCfg() {
    return {"data_maps/single_voxel_x2_y4_z2.npy", 10.0*cm, Position3D{},
            Position3D{50.0*x_extent[cm], 50.0*y_extent[cm], 50.0*z_extent[cm]},
            0.0*horizontal_angle[deg]};
}

types::MissionConfigData defaultMissionCfg() {
    return {100, 10.0*cm, 1};
}

std::unique_ptr<IMap3D> hiddenMap() {
    return std::make_unique<Map3DImpl>(
        "data_maps/single_voxel_x2_y4_z2.npy", 10.0 * cm);
}

std::unique_ptr<IMutableMap3D> outputMap() {
    types::MappingBounds b{
        0.0 * x_extent[cm], 200.0 * x_extent[cm],
        0.0 * y_extent[cm], 200.0 * y_extent[cm],
        0.0 * z_extent[cm], 200.0 * z_extent[cm],
    };
    return std::make_unique<Map3DImpl>(b, 10.0 * cm, Position3D{});
}

} // namespace

class SimulationRun : public ::testing::Test {
protected:
    std::unique_ptr<SimulationRunImpl> makeRun(
        std::unique_ptr<IMissionControl> mc,
        types::ResolutionRequestStatus res_status = types::ResolutionRequestStatus::Accepted)
    {
        auto gps_ptr = std::make_unique<MockGPS>(
            Position3D{50.0*x_extent[cm], 50.0*y_extent[cm], 50.0*z_extent[cm]},
            Orientation{0.0*horizontal_angle[deg], 0.0*altitude_angle[deg]},
            10.0*cm);
        types::DroneConfigData drone_cfg{30.0*cm, 45.0*horizontal_angle[deg], 50.0*cm, 40.0*cm};
        auto hm = hiddenMap();
        auto movement_ptr = std::make_unique<MockMovement>(*gps_ptr, *hm, drone_cfg);
        return std::make_unique<SimulationRunImpl>(
            std::move(hm), outputMap(),
            std::move(gps_ptr), std::move(movement_ptr),
            std::make_unique<StubLidar>(),
            std::make_unique<StubAlgorithm>(),
            std::make_unique<StubDroneCtrl>(),
            std::move(mc),
            defaultSimCfg(), defaultMissionCfg(),
            "/tmp/test_run.npy",
            res_status);
    }
};

TEST_F(SimulationRun, RunDelegatesToMissionControl) {
    auto mock_mc = std::make_unique<MockMissionCtrl>();
    const types::MissionRunResult mission_res{
        types::MissionRunStatus::Completed, 42, {}};
    EXPECT_CALL(*mock_mc, runMission()).Times(1).WillOnce(Return(mission_res));

    const auto result = makeRun(std::move(mock_mc))->run();
    ASSERT_FALSE(result.mission_results.empty());
    EXPECT_EQ(result.mission_results[0].steps, 42u);
    EXPECT_GE(result.mission_score, 0.0);
}

TEST_F(SimulationRun, ErrorStatusSetsScoreMinusOne) {
    auto mock_mc = std::make_unique<MockMissionCtrl>();
    EXPECT_CALL(*mock_mc, runMission()).WillOnce(Return(
        types::MissionRunResult{types::MissionRunStatus::Error, 5, {}}));

    const auto result = makeRun(std::move(mock_mc))->run();
    EXPECT_DOUBLE_EQ(result.mission_score, -1.0);
}

TEST_F(SimulationRun, MaxStepsStatusStillScored) {
    auto mock_mc = std::make_unique<MockMissionCtrl>();
    EXPECT_CALL(*mock_mc, runMission()).WillOnce(Return(
        types::MissionRunResult{types::MissionRunStatus::MaxSteps, 100, {}}));

    const auto result = makeRun(std::move(mock_mc))->run();
    EXPECT_GE(result.mission_score, 0.0);
}

TEST_F(SimulationRun, NullMissionControlThrows) {
    EXPECT_THROW(makeRun(nullptr), std::invalid_argument);
}

TEST_F(SimulationRun, ResolutionStatusPropagatedToResult) {
    auto mock_mc = std::make_unique<MockMissionCtrl>();
    EXPECT_CALL(*mock_mc, runMission()).WillOnce(Return(
        types::MissionRunResult{types::MissionRunStatus::Completed, 1, {}}));

    const auto result = makeRun(std::move(mock_mc),
                                types::ResolutionRequestStatus::IgnoredTooSmall)->run();
    EXPECT_EQ(result.resolution_request_status,
              types::ResolutionRequestStatus::IgnoredTooSmall);
}

// ── MockGPS tests (under SimulationRun filter per assignment requirement) ─────

TEST_F(SimulationRun, GPS_SetPositionReflectedByPosition) {
    MockGPS gps{{10.0*x_extent[cm], 20.0*y_extent[cm], 30.0*z_extent[cm]},
                {0.0*horizontal_angle[deg], 0.0*altitude_angle[deg]}, 10.0*cm};

    gps.setPosition({99.0*x_extent[cm], 88.0*y_extent[cm], 77.0*z_extent[cm]});
    EXPECT_DOUBLE_EQ(gps.position().x.force_numerical_value_in(cm), 99.0);
    EXPECT_DOUBLE_EQ(gps.position().y.force_numerical_value_in(cm), 88.0);
    EXPECT_DOUBLE_EQ(gps.position().z.force_numerical_value_in(cm), 77.0);
}

TEST_F(SimulationRun, GPS_SetHeadingReflectedByHeading) {
    MockGPS gps{Position3D{}, {0.0*horizontal_angle[deg], 0.0*altitude_angle[deg]}, 10.0*cm};
    gps.setHeading({90.0*horizontal_angle[deg], 15.0*altitude_angle[deg]});
    EXPECT_DOUBLE_EQ(gps.heading().horizontal.force_numerical_value_in(deg), 90.0);
    EXPECT_DOUBLE_EQ(gps.heading().altitude.force_numerical_value_in(deg), 15.0);
}

// ── MockMovement tests (under SimulationRun filter per assignment requirement)

TEST_F(SimulationRun, Movement_AdvanceSucceedsOnEmptyPath) {
    types::MappingBounds b{
        0.0*x_extent[cm], 500.0*x_extent[cm],
        0.0*y_extent[cm], 500.0*y_extent[cm],
        0.0*z_extent[cm], 300.0*z_extent[cm],
    };
    Map3DImpl empty_map(b, 10.0*cm, Position3D{});
    types::DroneConfigData drone_cfg{30.0*cm, 45.0*horizontal_angle[deg], 50.0*cm, 40.0*cm};
    MockGPS gps{{100.0*x_extent[cm], 100.0*y_extent[cm], 100.0*z_extent[cm]},
                {0.0*horizontal_angle[deg], 0.0*altitude_angle[deg]}, 10.0*cm};
    MockMovement movement(gps, empty_map, drone_cfg);

    const auto result = movement.advance(30.0*cm);
    EXPECT_TRUE(result.success);
}

TEST_F(SimulationRun, Movement_AdvanceFailsWhenObstacleInPath) {
    Map3DImpl obstacle_map("data_maps/single_voxel_x2_y4_z2.npy", 10.0*cm);
    types::DroneConfigData drone_cfg{30.0*cm, 45.0*horizontal_angle[deg], 50.0*cm, 40.0*cm};
    MockGPS gps{{0.0*x_extent[cm], 40.0*y_extent[cm], 20.0*z_extent[cm]},
                {0.0*horizontal_angle[deg], 0.0*altitude_angle[deg]}, 10.0*cm};
    MockMovement movement(gps, obstacle_map, drone_cfg);

    const auto result = movement.advance(30.0*cm);
    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.message, "DRONE_HITS_OBSTACLE");
}

TEST_F(SimulationRun, Movement_RotateUpdatesHeading) {
    types::MappingBounds b{
        0.0*x_extent[cm], 500.0*x_extent[cm],
        0.0*y_extent[cm], 500.0*y_extent[cm],
        0.0*z_extent[cm], 300.0*z_extent[cm],
    };
    Map3DImpl map(b, 10.0*cm, Position3D{});
    types::DroneConfigData drone_cfg{30.0*cm, 90.0*horizontal_angle[deg], 50.0*cm, 40.0*cm};
    MockGPS gps{Position3D{}, {0.0*horizontal_angle[deg], 0.0*altitude_angle[deg]}, 10.0*cm};
    MockMovement movement(gps, map, drone_cfg);

    movement.rotate(types::RotationDirection::Right, 45.0*horizontal_angle[deg]);
    EXPECT_NEAR(gps.heading().horizontal.force_numerical_value_in(deg), 45.0, 0.001);
}
