#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <drone_mapper/ISimulationRun.h>
#include <drone_mapper/ISimulationRunFactory.h>
#include <drone_mapper/SimulationManager.h>

namespace dm = drone_mapper;
using ::testing::Return;
using ::testing::_;

class MockRunFactory : public dm::ISimulationRunFactory {
public:
    MOCK_METHOD(std::unique_ptr<dm::ISimulationRun>, create,
                (const dm::types::SimulationConfigData&,
                 const dm::types::MissionConfigData&,
                 const dm::types::DroneConfigData&,
                 const dm::types::LidarConfigData&,
                 const std::filesystem::path&),
                (override));
};

class MockSimRun : public dm::ISimulationRun {
public:
    explicit MockSimRun(dm::types::SimulationResult r) : result_(std::move(r)) {}
    dm::types::SimulationResult run() override { return result_; }
private:
    dm::types::SimulationResult result_;
};

namespace {

dm::types::SimulationCompositionData makeComposition(
    int n_missions, int n_drones, int n_lidars) {
    std::vector<dm::types::SimulationConfigData> sims{{
        "data_maps/single_voxel_x2_y4_z2.npy",
        10.0 * dm::cm,
        dm::Position3D{},
        dm::Position3D{50.0*dm::x_extent[dm::cm], 50.0*dm::y_extent[dm::cm], 50.0*dm::z_extent[dm::cm]},
        0.0 * dm::horizontal_angle[dm::deg],
    }};
    std::vector<dm::types::MissionConfigData> missions;
    for (int i = 0; i < n_missions; ++i)
        missions.push_back({100, 10.0 * dm::cm, 1});

    std::vector<dm::types::DroneConfigData> drones;
    for (int i = 0; i < n_drones; ++i)
        drones.push_back({30.0*dm::cm, 45.0*dm::horizontal_angle[dm::deg], 50.0*dm::cm, 40.0*dm::cm});

    std::vector<dm::types::LidarConfigData> lidars;
    for (int i = 0; i < n_lidars; ++i)
        lidars.push_back({20.0*dm::cm, 120.0*dm::cm, 2.5*dm::cm, 3});

    return {"composition.yaml", sims, missions, drones, lidars};
}

dm::types::SimulationResult okResult(double score = 75.0) {
    dm::types::SimulationResult r;
    r.mission_results = {{dm::types::MissionRunStatus::Completed, 10, {}}};
    r.mission_score   = score;
    return r;
}

dm::types::SimulationResult errResult() {
    dm::types::SimulationResult r;
    r.mission_results = {{dm::types::MissionRunStatus::Error, 0, {{"ERR","ERR"}}}};
    r.mission_score   = -1.0;
    return r;
}

} // namespace

class SimulationManager : public ::testing::Test {};

TEST_F(SimulationManager, ExpandsCartesianProduct_2Missions2Drones2Lidars) {
    auto factory = std::make_unique<MockRunFactory>();
    // 1 sim × 2 missions × 2 drones × 2 lidars = 8 runs
    EXPECT_CALL(*factory, create(_, _, _, _, _))
        .Times(8)
        .WillRepeatedly([](const auto&, const auto&, const auto&, const auto&, const auto&) {
            return std::make_unique<MockSimRun>(okResult());
        });

    dm::SimulationManager manager(std::move(factory));
    const auto report = manager.run(makeComposition(2, 2, 2), "/tmp/test_mgr");
    EXPECT_EQ(report.runs.size(), 8u);
}

TEST_F(SimulationManager, AggregatesScores) {
    auto factory = std::make_unique<MockRunFactory>();
    EXPECT_CALL(*factory, create(_, _, _, _, _))
        .Times(1)
        .WillOnce([](const auto&, const auto&, const auto&, const auto&, const auto&) {
            return std::make_unique<MockSimRun>(okResult(90.0));
        });

    dm::SimulationManager manager(std::move(factory));
    const auto report = manager.run(makeComposition(1, 1, 1), "/tmp/test_mgr_scores");
    ASSERT_EQ(report.runs.size(), 1u);
    EXPECT_DOUBLE_EQ(report.runs[0].mission_score, 90.0);
}

TEST_F(SimulationManager, CapturesExceptionsAsErrorRuns) {
    auto factory = std::make_unique<MockRunFactory>();
    EXPECT_CALL(*factory, create(_, _, _, _, _))
        .Times(1)
        .WillOnce([](const auto&, const auto&, const auto&, const auto&, const auto&)
                   -> std::unique_ptr<dm::ISimulationRun> {
            throw std::runtime_error("map file not found");
        });

    dm::SimulationManager manager(std::move(factory));
    dm::types::SimulationManagerReport report;
    ASSERT_NO_THROW(report = manager.run(makeComposition(1, 1, 1), "/tmp/test_mgr_err"));
    ASSERT_EQ(report.runs.size(), 1u);
    ASSERT_FALSE(report.runs[0].mission_results.empty());
    EXPECT_EQ(report.runs[0].mission_results[0].status, dm::types::MissionRunStatus::Error);
}
