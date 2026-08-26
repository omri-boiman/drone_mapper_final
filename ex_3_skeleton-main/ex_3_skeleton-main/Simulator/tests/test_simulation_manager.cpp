#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <Simulator/ISimulationRun.h>
#include <Simulator/ISimulationRunFactory.h>
#include <Simulator/SimulationManager.h>

#include <filesystem>
#include <fstream>
#include <sstream>

namespace sim = simulator;
using ::testing::Return;
using ::testing::_;

class MockRunFactory : public sim::ISimulationRunFactory {
public:
    MOCK_METHOD(std::unique_ptr<sim::ISimulationRun>, create,
                (const sim::types::SimulationConfigData&,
                 const common::types::MissionConfigData&,
                 const common::types::DroneConfigData&,
                 const common::types::LidarConfigData&,
                 const std::filesystem::path&),
                (override));
};

class MockSimRun : public sim::ISimulationRun {
public:
    explicit MockSimRun(sim::types::SimulationResult r) : result_(std::move(r)) {}
    sim::types::SimulationResult run() override { return result_; }
private:
    sim::types::SimulationResult result_;
};

namespace {

sim::types::SimulationCompositionData makeComposition(
    int n_missions, int n_drones, int n_lidars) {
    sim::types::SimulationConfigData sim_cfg{
        "data_maps/single_voxel_x2_y4_z2.npy",
        10.0 * common::cm,
        common::Position3D{},
        common::Position3D{50.0*common::x_extent[common::cm], 50.0*common::y_extent[common::cm], 50.0*common::z_extent[common::cm]},
        0.0 * common::horizontal_angle[common::deg],
    };
    std::vector<common::types::MissionConfigData> missions;
    for (int i = 0; i < n_missions; ++i)
        missions.push_back({100, 10.0 * common::cm, 1});

    std::vector<common::types::DroneConfigData> drones;
    for (int i = 0; i < n_drones; ++i)
        drones.push_back({30.0*common::cm, 45.0*common::horizontal_angle[common::deg], 50.0*common::cm, 40.0*common::cm});

    std::vector<common::types::LidarConfigData> lidars;
    for (int i = 0; i < n_lidars; ++i)
        lidars.push_back({20.0*common::cm, 120.0*common::cm, 2.5*common::cm, 3});

    sim::types::SimulationCompositionData comp;
    comp.composition_file = "composition.yaml";
    comp.simulation_mission_groups.emplace_back(std::move(sim_cfg), std::move(missions));
    comp.drone_configs = std::move(drones);
    comp.lidar_configs = std::move(lidars);
    return comp;
}

sim::types::SimulationResult okResult(double score = 75.0) {
    sim::types::SimulationResult r;
    r.mission_results = {{common::types::MissionRunStatus::Completed, 10, {}}};
    r.mission_score   = score;
    return r;
}

sim::types::SimulationResult errResult() {
    sim::types::SimulationResult r;
    r.mission_results = {{common::types::MissionRunStatus::Error, 0, {{"ERR","ERR"}}}};
    r.mission_score   = -1.0;
    return r;
}

} // namespace

class ThrowingSimRun : public sim::ISimulationRun {
public:
    sim::types::SimulationResult run() override {
        throw std::runtime_error("run failed");
    }
};

class SimulationManager : public ::testing::Test {};

TEST_F(SimulationManager, ExpandsCartesianProduct_2Missions2Drones2Lidars) {
    auto factory = std::make_unique<MockRunFactory>();
    // 1 sim × 2 missions × 2 drones × 2 lidars = 8 runs
    EXPECT_CALL(*factory, create(_, _, _, _, _))
        .Times(8)
        .WillRepeatedly([](const auto&, const auto&, const auto&, const auto&, const auto&) {
            return std::make_unique<MockSimRun>(okResult());
        });

    sim::SimulationManager manager(std::move(factory));
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

    sim::SimulationManager manager(std::move(factory));
    const auto report = manager.run(makeComposition(1, 1, 1), "/tmp/test_mgr_scores");
    ASSERT_EQ(report.runs.size(), 1u);
    EXPECT_DOUBLE_EQ(report.runs[0].mission_score, 90.0);
}

TEST_F(SimulationManager, CapturesExceptionsAsErrorRuns) {
    auto factory = std::make_unique<MockRunFactory>();
    EXPECT_CALL(*factory, create(_, _, _, _, _))
        .Times(1)
        .WillOnce([](const auto&, const auto&, const auto&, const auto&, const auto&)
                   -> std::unique_ptr<sim::ISimulationRun> {
            throw std::runtime_error("map file not found");
        });

    sim::SimulationManager manager(std::move(factory));
    sim::types::SimulationManagerReport report;
    ASSERT_NO_THROW(report = manager.run(makeComposition(1, 1, 1), "/tmp/test_mgr_err"));
    ASSERT_EQ(report.runs.size(), 1u);
    ASSERT_FALSE(report.runs[0].mission_results.empty());
    EXPECT_EQ(report.runs[0].mission_results[0].status, common::types::MissionRunStatus::Error);
}

TEST_F(SimulationManager, NullFactoryThrows) {
    EXPECT_THROW(sim::SimulationManager(nullptr), std::invalid_argument);
}

TEST_F(SimulationManager, RunThrowingDuringRunIsCaptured) {
    auto factory = std::make_unique<MockRunFactory>();
    EXPECT_CALL(*factory, create(_, _, _, _, _))
        .Times(1)
        .WillOnce([](const auto&, const auto&, const auto&, const auto&, const auto&)
                   -> std::unique_ptr<sim::ISimulationRun> {
            return std::make_unique<ThrowingSimRun>();
        });

    sim::SimulationManager manager(std::move(factory));
    sim::types::SimulationManagerReport report;
    ASSERT_NO_THROW(report = manager.run(makeComposition(1, 1, 1), "/tmp/test_mgr_run_err"));
    ASSERT_EQ(report.runs.size(), 1u);
    EXPECT_EQ(report.runs[0].mission_score, -1.0);
    EXPECT_EQ(report.runs[0].mission_results[0].status, common::types::MissionRunStatus::Error);
}

TEST_F(SimulationManager, TwoSimulationsProduceTwoRuns) {
    auto factory = std::make_unique<MockRunFactory>();
    EXPECT_CALL(*factory, create(_, _, _, _, _))
        .Times(2)
        .WillRepeatedly([](const auto&, const auto&, const auto&, const auto&, const auto&) {
            return std::make_unique<MockSimRun>(okResult());
        });

    // 2 sims × 1 mission each × 1 drone × 1 lidar = 2 runs
    sim::types::SimulationConfigData sim_cfg{
        "data_maps/single_voxel_x2_y4_z2.npy", 10.0*common::cm, common::Position3D{},
        common::Position3D{50.0*common::x_extent[common::cm], 50.0*common::y_extent[common::cm], 50.0*common::z_extent[common::cm]},
        0.0*common::horizontal_angle[common::deg]};
    sim::types::SimulationCompositionData composition;
    composition.simulation_mission_groups.emplace_back(sim_cfg, std::vector<common::types::MissionConfigData>{{100, 10.0*common::cm, 1}});
    composition.simulation_mission_groups.emplace_back(sim_cfg, std::vector<common::types::MissionConfigData>{{100, 10.0*common::cm, 1}});
    composition.drone_configs = {{30.0*common::cm, 45.0*common::horizontal_angle[common::deg], 50.0*common::cm, 40.0*common::cm}};
    composition.lidar_configs = {{20.0*common::cm, 120.0*common::cm, 2.5*common::cm, 3}};

    sim::SimulationManager manager(std::move(factory));
    const auto report = manager.run(composition, "/tmp/test_mgr_two_sims");
    EXPECT_EQ(report.runs.size(), 2u);
}

TEST_F(SimulationManager, ErrorRunsDoNotBlockSubsequentRuns) {
    auto factory = std::make_unique<MockRunFactory>();
    // First call throws, second succeeds
    EXPECT_CALL(*factory, create(_, _, _, _, _))
        .WillOnce([](const auto&, const auto&, const auto&, const auto&, const auto&)
                   -> std::unique_ptr<sim::ISimulationRun> {
            throw std::runtime_error("first run fails");
        })
        .WillOnce([](const auto&, const auto&, const auto&, const auto&, const auto&) {
            return std::make_unique<MockSimRun>(okResult(80.0));
        });

    sim::SimulationManager manager(std::move(factory));
    const auto report = manager.run(makeComposition(2, 1, 1), "/tmp/test_mgr_error_continue");
    ASSERT_EQ(report.runs.size(), 2u);
    EXPECT_EQ(report.runs[0].mission_results[0].status, common::types::MissionRunStatus::Error);
    EXPECT_DOUBLE_EQ(report.runs[1].mission_score, 80.0);
}

// Helpers for log-content tests
namespace {
std::string readFile(const std::filesystem::path& p) {
    std::ifstream f(p);
    std::ostringstream ss;
    ss << f.rdbuf();
    return ss.str();
}
} // namespace

TEST_F(SimulationManager, ExceptionIsWrittenToErrorLog) {
    auto factory = std::make_unique<MockRunFactory>();
    EXPECT_CALL(*factory, create(_, _, _, _, _))
        .WillOnce([](const auto&, const auto&, const auto&, const auto&, const auto&)
                   -> std::unique_ptr<sim::ISimulationRun> {
            throw std::runtime_error("map file not found");
        });

    sim::SimulationManager manager(std::move(factory));
    (void)manager.run(makeComposition(1, 1, 1), "/tmp/test_mgr_log_exception");

    const auto log_path = std::filesystem::path("/tmp/test_mgr_log_exception") / "output_results" / "error.log";
    ASSERT_TRUE(std::filesystem::exists(log_path)) << "error.log not created";
    const std::string contents = readFile(log_path);
    EXPECT_THAT(contents, ::testing::HasSubstr("map file not found"));
}

TEST_F(SimulationManager, AllMissionsFailedRunIsStillAggregated) {
    auto factory = std::make_unique<MockRunFactory>();
    EXPECT_CALL(*factory, create(_, _, _, _, _))
        .Times(1)
        .WillOnce([](const auto&, const auto&, const auto&, const auto&, const auto&) {
            return std::make_unique<MockSimRun>(errResult());
        });

    sim::SimulationManager manager(std::move(factory));
    const auto report = manager.run(makeComposition(1, 1, 1), "/tmp/test_mgr_all_failed");
    ASSERT_EQ(report.runs.size(), 1u)
        << "A run whose missions all failed must still be aggregated into report.runs";
    EXPECT_EQ(report.runs[0].mission_results[0].status, common::types::MissionRunStatus::Error);
}

TEST_F(SimulationManager, ScoreRangeStaysZeroToHundredEvenWithErrors) {
    auto factory = std::make_unique<MockRunFactory>();
    EXPECT_CALL(*factory, create(_, _, _, _, _))
        .Times(1)
        .WillOnce([](const auto&, const auto&, const auto&, const auto&, const auto&) {
            return std::make_unique<MockSimRun>(errResult());
        });

    sim::SimulationManager manager(std::move(factory));
    const auto report = manager.run(makeComposition(1, 1, 1), "/tmp/test_mgr_score_range");
    EXPECT_DOUBLE_EQ(std::get<0>(report.score_range), 0.0)
        << "score_range minimum must stay 0.0; error_score carries the error sentinel separately";
    EXPECT_DOUBLE_EQ(std::get<1>(report.score_range), 100.0);
}

TEST_F(SimulationManager, MissionErrorIsWrittenToErrorLog) {
    auto factory = std::make_unique<MockRunFactory>();
    EXPECT_CALL(*factory, create(_, _, _, _, _))
        .WillOnce([](const auto&, const auto&, const auto&, const auto&, const auto&) {
            return std::make_unique<MockSimRun>(errResult());
        });

    sim::SimulationManager manager(std::move(factory));
    (void)manager.run(makeComposition(1, 1, 1), "/tmp/test_mgr_log_mission_err");

    const auto log_path = std::filesystem::path("/tmp/test_mgr_log_mission_err") / "output_results" / "error.log";
    ASSERT_TRUE(std::filesystem::exists(log_path)) << "error.log not created";
    const std::string contents = readFile(log_path);
    EXPECT_THAT(contents, ::testing::HasSubstr("ERR"));
}
