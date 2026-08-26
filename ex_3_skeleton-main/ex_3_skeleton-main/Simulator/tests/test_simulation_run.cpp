#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <Common/ILidar.h>
#include <Common/IMappingAlgorithm.h>
#include <Common/IMissionControl.h>
#include <Simulator/Map3DImpl.h>
#include <Simulator/MockGPS.h>
#include <Simulator/MockMovement.h>
#include <Simulator/SimulationRunImpl.h>

namespace sim = simulator;
using ::testing::Return;
using ::testing::_;

class MockMissionCtrl : public common::IMissionControl {
public:
    MOCK_METHOD(common::types::MissionRunResult, runMission, (), (override));
};

class StubLidar : public common::ILidar {
public:
    common::types::LidarScanResult scan(common::Orientation) const override { return {}; }
    common::types::LidarConfigData config() const override { return {}; }
};

class StubAlgorithm : public common::IMappingAlgorithm {
public:
    struct NullMap : common::IMap3D {
        common::types::VoxelOccupancy atVoxel(const common::Position3D&) const override {
            return common::types::VoxelOccupancy::Empty;
        }
        common::types::MapConfig getMapConfig() const override { return {}; }
        bool isInBounds(const common::Position3D&) const override { return false; }
    };
    StubAlgorithm()
        : IMappingAlgorithm(common::MappingAlgorithmDependencies{
              common::types::MissionConfigData{}, common::types::LidarConfigData{},
              common::types::DroneConfigData{}, null_map_}) {}
    common::types::MappingStepCommand nextStep(const common::types::DroneState&,
                                               const common::types::LidarScanResult*) override {
        return {std::nullopt, std::nullopt, common::types::AlgorithmStatus::Finished};
    }
private:
    NullMap null_map_;
};

namespace {

sim::types::SimulationConfigData defaultSimCfg() {
    return {"data_maps/single_voxel_x2_y4_z2.npy", 10.0*sim::cm, sim::Position3D{},
            sim::Position3D{50.0*sim::x_extent[sim::cm], 50.0*sim::y_extent[sim::cm], 50.0*sim::z_extent[sim::cm]},
            0.0*sim::horizontal_angle[sim::deg]};
}

common::types::MissionConfigData defaultMissionCfg() {
    return {100, 10.0*sim::cm, 1};
}

std::unique_ptr<common::IMap3D> hiddenMap() {
    return std::make_unique<sim::Map3DImpl>(
        "data_maps/single_voxel_x2_y4_z2.npy", 10.0 * sim::cm);
}

std::unique_ptr<common::IMutableMap3D> outputMap() {
    common::types::MappingBounds b{
        0.0 * sim::x_extent[sim::cm], 200.0 * sim::x_extent[sim::cm],
        0.0 * sim::y_extent[sim::cm], 200.0 * sim::y_extent[sim::cm],
        0.0 * sim::z_extent[sim::cm], 200.0 * sim::z_extent[sim::cm],
    };
    return std::make_unique<sim::Map3DImpl>(b, 10.0 * sim::cm, sim::Position3D{});
}

} // namespace

class SimulationRun : public ::testing::Test {
protected:
    std::unique_ptr<sim::SimulationRunImpl> makeRun(
        std::unique_ptr<common::IMissionControl> mc,
        sim::types::ResolutionRequestStatus res_status = sim::types::ResolutionRequestStatus::Accepted)
    {
        auto gps_ptr = std::make_unique<sim::MockGPS>(
            sim::Position3D{50.0*sim::x_extent[sim::cm], 50.0*sim::y_extent[sim::cm], 50.0*sim::z_extent[sim::cm]},
            sim::Orientation{0.0*sim::horizontal_angle[sim::deg], 0.0*sim::altitude_angle[sim::deg]},
            10.0*sim::cm);
        common::types::DroneConfigData drone_cfg{30.0*sim::cm, 45.0*sim::horizontal_angle[sim::deg], 50.0*sim::cm, 40.0*sim::cm};
        auto hm = hiddenMap();
        auto movement_ptr = std::make_unique<sim::MockMovement>(*gps_ptr, *hm, drone_cfg);
        return std::make_unique<sim::SimulationRunImpl>(
            std::move(hm), outputMap(),
            std::move(gps_ptr), std::move(movement_ptr),
            std::make_unique<StubLidar>(),
            std::make_unique<StubAlgorithm>(),
            std::move(mc),
            defaultSimCfg(), defaultMissionCfg(),
            "/tmp/test_run.npy",
            res_status);
    }
};

TEST_F(SimulationRun, RunDelegatesToMissionControl) {
    auto mock_mc = std::make_unique<MockMissionCtrl>();
    const common::types::MissionRunResult mission_res{
        common::types::MissionRunStatus::Completed, 42, {}};
    EXPECT_CALL(*mock_mc, runMission()).Times(1).WillOnce(Return(mission_res));

    const auto result = makeRun(std::move(mock_mc))->run();
    ASSERT_FALSE(result.mission_results.empty());
    EXPECT_EQ(result.mission_results[0].steps, 42u);
    EXPECT_GE(result.mission_score, 0.0);
}

TEST_F(SimulationRun, ErrorStatusSetsScoreMinusOne) {
    auto mock_mc = std::make_unique<MockMissionCtrl>();
    EXPECT_CALL(*mock_mc, runMission()).WillOnce(Return(
        common::types::MissionRunResult{common::types::MissionRunStatus::Error, 5, {}}));

    const auto result = makeRun(std::move(mock_mc))->run();
    EXPECT_DOUBLE_EQ(result.mission_score, -1.0);
}

TEST_F(SimulationRun, ErrorMissionResultIsStillAddedToMissionResults) {
    auto mock_mc = std::make_unique<MockMissionCtrl>();
    EXPECT_CALL(*mock_mc, runMission()).WillOnce(Return(
        common::types::MissionRunResult{common::types::MissionRunStatus::Error, 5, {}}));

    const auto result = makeRun(std::move(mock_mc))->run();
    ASSERT_FALSE(result.mission_results.empty())
        << "An errored mission result must still be recorded in mission_results";
    EXPECT_EQ(result.mission_results[0].status, common::types::MissionRunStatus::Error);
}

TEST_F(SimulationRun, MaxStepsStatusStillScored) {
    auto mock_mc = std::make_unique<MockMissionCtrl>();
    EXPECT_CALL(*mock_mc, runMission()).WillOnce(Return(
        common::types::MissionRunResult{common::types::MissionRunStatus::MaxSteps, 100, {}}));

    const auto result = makeRun(std::move(mock_mc))->run();
    EXPECT_GE(result.mission_score, 0.0);
}

TEST_F(SimulationRun, NullMissionControlThrows) {
    EXPECT_THROW(makeRun(nullptr), std::invalid_argument);
}

TEST_F(SimulationRun, ResolutionStatusPropagatedToResult) {
    auto mock_mc = std::make_unique<MockMissionCtrl>();
    EXPECT_CALL(*mock_mc, runMission()).WillOnce(Return(
        common::types::MissionRunResult{common::types::MissionRunStatus::Completed, 1, {}}));

    const auto result = makeRun(std::move(mock_mc),
                                sim::types::ResolutionRequestStatus::IgnoredTooSmall)->run();
    EXPECT_EQ(result.resolution_request_status,
              sim::types::ResolutionRequestStatus::IgnoredTooSmall);
}

// ── MockGPS tests (under SimulationRun filter per assignment requirement) ─────

TEST_F(SimulationRun, GPS_SetPositionReflectedByPosition) {
    sim::MockGPS gps{{10.0*sim::x_extent[sim::cm], 20.0*sim::y_extent[sim::cm], 30.0*sim::z_extent[sim::cm]},
                {0.0*sim::horizontal_angle[sim::deg], 0.0*sim::altitude_angle[sim::deg]}, 10.0*sim::cm};

    gps.setPosition({99.0*sim::x_extent[sim::cm], 88.0*sim::y_extent[sim::cm], 77.0*sim::z_extent[sim::cm]});
    EXPECT_DOUBLE_EQ(gps.position().x.force_numerical_value_in(sim::cm), 99.0);
    EXPECT_DOUBLE_EQ(gps.position().y.force_numerical_value_in(sim::cm), 88.0);
    EXPECT_DOUBLE_EQ(gps.position().z.force_numerical_value_in(sim::cm), 77.0);
}

TEST_F(SimulationRun, GPS_SetHeadingReflectedByHeading) {
    sim::MockGPS gps{sim::Position3D{}, {0.0*sim::horizontal_angle[sim::deg], 0.0*sim::altitude_angle[sim::deg]}, 10.0*sim::cm};
    gps.setHeading({90.0*sim::horizontal_angle[sim::deg], 15.0*sim::altitude_angle[sim::deg]});
    EXPECT_DOUBLE_EQ(gps.heading().horizontal.force_numerical_value_in(sim::deg), 90.0);
    EXPECT_DOUBLE_EQ(gps.heading().altitude.force_numerical_value_in(sim::deg), 15.0);
}

// ── MockMovement tests (under SimulationRun filter per assignment requirement)

TEST_F(SimulationRun, Movement_AdvanceSucceedsOnEmptyPath) {
    common::types::MappingBounds b{
        0.0*sim::x_extent[sim::cm], 500.0*sim::x_extent[sim::cm],
        0.0*sim::y_extent[sim::cm], 500.0*sim::y_extent[sim::cm],
        0.0*sim::z_extent[sim::cm], 300.0*sim::z_extent[sim::cm],
    };
    sim::Map3DImpl empty_map(b, 10.0*sim::cm, sim::Position3D{});
    common::types::DroneConfigData drone_cfg{30.0*sim::cm, 45.0*sim::horizontal_angle[sim::deg], 50.0*sim::cm, 40.0*sim::cm};
    sim::MockGPS gps{{100.0*sim::x_extent[sim::cm], 100.0*sim::y_extent[sim::cm], 100.0*sim::z_extent[sim::cm]},
                {0.0*sim::horizontal_angle[sim::deg], 0.0*sim::altitude_angle[sim::deg]}, 10.0*sim::cm};
    sim::MockMovement movement(gps, empty_map, drone_cfg);

    const auto result = movement.advance(30.0*sim::cm);
    EXPECT_TRUE(result.success);
}

TEST_F(SimulationRun, Movement_AdvanceFailsWhenObstacleInPath) {
    sim::Map3DImpl obstacle_map("data_maps/single_voxel_x2_y4_z2.npy", 10.0*sim::cm);
    common::types::DroneConfigData drone_cfg{30.0*sim::cm, 45.0*sim::horizontal_angle[sim::deg], 50.0*sim::cm, 40.0*sim::cm};
    sim::MockGPS gps{{0.0*sim::x_extent[sim::cm], 40.0*sim::y_extent[sim::cm], 20.0*sim::z_extent[sim::cm]},
                {0.0*sim::horizontal_angle[sim::deg], 0.0*sim::altitude_angle[sim::deg]}, 10.0*sim::cm};
    sim::MockMovement movement(gps, obstacle_map, drone_cfg);

    const auto result = movement.advance(30.0*sim::cm);
    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.message, "DRONE_HITS_OBSTACLE");
}

TEST_F(SimulationRun, Movement_RotateUpdatesHeading) {
    common::types::MappingBounds b{
        0.0*sim::x_extent[sim::cm], 500.0*sim::x_extent[sim::cm],
        0.0*sim::y_extent[sim::cm], 500.0*sim::y_extent[sim::cm],
        0.0*sim::z_extent[sim::cm], 300.0*sim::z_extent[sim::cm],
    };
    sim::Map3DImpl map(b, 10.0*sim::cm, sim::Position3D{});
    common::types::DroneConfigData drone_cfg{30.0*sim::cm, 90.0*sim::horizontal_angle[sim::deg], 50.0*sim::cm, 40.0*sim::cm};
    sim::MockGPS gps{sim::Position3D{}, {0.0*sim::horizontal_angle[sim::deg], 0.0*sim::altitude_angle[sim::deg]}, 10.0*sim::cm};
    sim::MockMovement movement(gps, map, drone_cfg);

    movement.rotate(common::types::RotationDirection::Right, 45.0*sim::horizontal_angle[sim::deg]);
    EXPECT_NEAR(gps.heading().horizontal.force_numerical_value_in(sim::deg), 45.0, 0.001);
}
