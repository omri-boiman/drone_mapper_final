#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <drone_mapper/ISimulationRun.h>
#include <drone_mapper/ISimulationRunFactory.h>
#include <drone_mapper/SimulationManager.h>

#include <filesystem>
#include <fstream>
#include <sstream>

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
    dm::types::SimulationConfigData sim{
        "data_maps/single_voxel_x2_y4_z2.npy",
        10.0 * dm::cm,
        dm::Position3D{},
        dm::Position3D{50.0*dm::x_extent[dm::cm], 50.0*dm::y_extent[dm::cm], 50.0*dm::z_extent[dm::cm]},
        0.0 * dm::horizontal_angle[dm::deg],
    };
    std::vector<dm::types::MissionConfigData> missions;
    for (int i = 0; i < n_missions; ++i)
        missions.push_back({100, 10.0 * dm::cm, 1});

    std::vector<dm::types::DroneConfigData> drones;
    for (int i = 0; i < n_drones; ++i)
        drones.push_back({30.0*dm::cm, 45.0*dm::horizontal_angle[dm::deg], 50.0*dm::cm, 40.0*dm::cm});

    std::vector<dm::types::LidarConfigData> lidars;
    for (int i = 0; i < n_lidars; ++i)
        lidars.push_back({20.0*dm::cm, 120.0*dm::cm, 2.5*dm::cm, 3});

    dm::types::SimulationCompositionData comp;
    comp.composition_file = "composition.yaml";
    comp.simulation_mission_groups.emplace_back(std::move(sim), std::move(missions));
    comp.drones = std::move(drones);
    comp.lidars = std::move(lidars);
    return comp;
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

class ThrowingSimRun : public dm::ISimulationRun {
public:
    dm::types::SimulationResult run() override {
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

TEST_F(SimulationManager, NullFactoryThrows) {
    EXPECT_THROW(dm::SimulationManager(nullptr), std::invalid_argument);
}

TEST_F(SimulationManager, RunThrowingDuringRunIsCaptured) {
    auto factory = std::make_unique<MockRunFactory>();
    EXPECT_CALL(*factory, create(_, _, _, _, _))
        .Times(1)
        .WillOnce([](const auto&, const auto&, const auto&, const auto&, const auto&)
                   -> std::unique_ptr<dm::ISimulationRun> {
            return std::make_unique<ThrowingSimRun>();
        });

    dm::SimulationManager manager(std::move(factory));
    dm::types::SimulationManagerReport report;
    ASSERT_NO_THROW(report = manager.run(makeComposition(1, 1, 1), "/tmp/test_mgr_run_err"));
    ASSERT_EQ(report.runs.size(), 1u);
    EXPECT_EQ(report.runs[0].mission_score, -1.0);
    EXPECT_EQ(report.runs[0].mission_results[0].status, dm::types::MissionRunStatus::Error);
}

TEST_F(SimulationManager, TwoSimulationsProduceTwoRuns) {
    auto factory = std::make_unique<MockRunFactory>();
    EXPECT_CALL(*factory, create(_, _, _, _, _))
        .Times(2)
        .WillRepeatedly([](const auto&, const auto&, const auto&, const auto&, const auto&) {
            return std::make_unique<MockSimRun>(okResult());
        });

    // 2 sims × 1 mission each × 1 drone × 1 lidar = 2 runs
    dm::types::SimulationConfigData sim_cfg{
        "data_maps/single_voxel_x2_y4_z2.npy", 10.0*dm::cm, dm::Position3D{},
        dm::Position3D{50.0*dm::x_extent[dm::cm], 50.0*dm::y_extent[dm::cm], 50.0*dm::z_extent[dm::cm]},
        0.0*dm::horizontal_angle[dm::deg]};
    dm::types::SimulationCompositionData composition;
    composition.simulation_mission_groups.emplace_back(sim_cfg, std::vector<dm::types::MissionConfigData>{{100, 10.0*dm::cm, 1}});
    composition.simulation_mission_groups.emplace_back(sim_cfg, std::vector<dm::types::MissionConfigData>{{100, 10.0*dm::cm, 1}});
    composition.drones   = {{30.0*dm::cm, 45.0*dm::horizontal_angle[dm::deg], 50.0*dm::cm, 40.0*dm::cm}};
    composition.lidars   = {{20.0*dm::cm, 120.0*dm::cm, 2.5*dm::cm, 3}};

    dm::SimulationManager manager(std::move(factory));
    const auto report = manager.run(composition, "/tmp/test_mgr_two_sims");
    EXPECT_EQ(report.runs.size(), 2u);
}

TEST_F(SimulationManager, ErrorRunsDoNotBlockSubsequentRuns) {
    auto factory = std::make_unique<MockRunFactory>();
    // First call throws, second succeeds
    EXPECT_CALL(*factory, create(_, _, _, _, _))
        .WillOnce([](const auto&, const auto&, const auto&, const auto&, const auto&)
                   -> std::unique_ptr<dm::ISimulationRun> {
            throw std::runtime_error("first run fails");
        })
        .WillOnce([](const auto&, const auto&, const auto&, const auto&, const auto&) {
            return std::make_unique<MockSimRun>(okResult(80.0));
        });

    dm::SimulationManager manager(std::move(factory));
    const auto report = manager.run(makeComposition(2, 1, 1), "/tmp/test_mgr_error_continue");
    ASSERT_EQ(report.runs.size(), 2u);
    EXPECT_EQ(report.runs[0].mission_results[0].status, dm::types::MissionRunStatus::Error);
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
                   -> std::unique_ptr<dm::ISimulationRun> {
            throw std::runtime_error("map file not found");
        });

    dm::SimulationManager manager(std::move(factory));
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

    dm::SimulationManager manager(std::move(factory));
    const auto report = manager.run(makeComposition(1, 1, 1), "/tmp/test_mgr_all_failed");
    ASSERT_EQ(report.runs.size(), 1u)
        << "A run whose missions all failed must still be aggregated into report.runs";
    EXPECT_EQ(report.runs[0].mission_results[0].status, dm::types::MissionRunStatus::Error);
}

TEST_F(SimulationManager, ScoreRangeStaysZeroToHundredEvenWithErrors) {
    auto factory = std::make_unique<MockRunFactory>();
    EXPECT_CALL(*factory, create(_, _, _, _, _))
        .Times(1)
        .WillOnce([](const auto&, const auto&, const auto&, const auto&, const auto&) {
            return std::make_unique<MockSimRun>(errResult());
        });

    dm::SimulationManager manager(std::move(factory));
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

    dm::SimulationManager manager(std::move(factory));
    (void)manager.run(makeComposition(1, 1, 1), "/tmp/test_mgr_log_mission_err");

    const auto log_path = std::filesystem::path("/tmp/test_mgr_log_mission_err") / "output_results" / "error.log";
    ASSERT_TRUE(std::filesystem::exists(log_path)) << "error.log not created";
    const std::string contents = readFile(log_path);
    EXPECT_THAT(contents, ::testing::HasSubstr("ERR"));
}
