#include <gtest/gtest.h>

#include <Simulator/Map3DImpl.h>
#include <Simulator/SimulationManager.h>
#include <Simulator/SimulationRunFactoryImpl.h>

#include "plugin/PluginLoader.h"

#include <chrono>
#include <filesystem>
#include <optional>

namespace sim = simulator;

// Ported from ex2's tests/integration/test_integration_benchmark.cpp. Unlike
// test_integration_pipeline.cpp (which drives the SimulationManager/
// SimulationRunFactoryImpl/SimulationRunImpl wiring with trivial stub
// algorithm/mission-control factories -- see StubPlugins.h), these tests
// exercise the REAL, dynamically-loaded Algorithm_<ids>.so /
// MissionControl_<ids>.so plugins against the benchmark map, same as a real
// `-comparative`/`-competition` run would. That's only possible because this
// target (see Simulator/CMakeLists.txt) is built with ENABLE_EXPORTS/
// -rdynamic and links the plugin/registration sources, exactly like the real
// simulator_<ids> executable, and only when the sibling Algorithm/
// MissionControl projects are built alongside it (i.e. via the root build).
//
// Each .so is dlopen'd exactly ONCE for the whole test suite in
// SetUpTestSuite() and the resulting factories are reused across every
// TEST_F below -- matching the spec's own invariant ("Creating an Algorithm
// instance / MissionControl instance from their factories should be cheap.
// Do not cache instances... this is NOT the same as unloading and loading
// the same .so file, which should be avoided").
//
// Benchmark map: benchmark_map.npy
// Dimensions (x,y,z): 29 x 30 x 31 voxels at 1 cm/voxel
// Ground (solid):     z = 0..14  (15 layers)
// First floor:        z = 15..19 (height 5)
//   Main entrance 4x4: x=21..24, y=25 open, z=15..18
//   Interior wall: y=16 for x=9..26 (passage at x=3..8)
// Between floors:     z = 20 (solid, 4x4 staircase opening at x=15..18, y=5..8)
// Second floor:       z = 21..26 (height 6)
//   2x2 room entrance: x=3..4, z=22..23 through y=17 interior wall
//   3x3 room entrance: y=20..22, z=24..26 through x=20 wall
// Roof:               z = 27 (solid, 2x1 secret opening at x=7, y=18..19)
// Drone start: outside main entrance in 4-voxel gap (y=26..29), at x=22, y=27, z=17

namespace {

constexpr double kResCm = 1.0;

sim::types::SimulationConfigData benchmarkSimConfig() {
    return {
        "data_maps/benchmark_map.npy",
        kResCm * common::cm,
        common::Position3D{},
        common::Position3D{22.0*common::x_extent[common::cm], 27.0*common::y_extent[common::cm], 17.0*common::z_extent[common::cm]},
        0.0 * common::horizontal_angle[common::deg],
    };
}

// Fast lidar for integration tests (fov_circles=2 -> 5 beams, z_max=15cm)
common::types::LidarConfigData fastLidar() {
    return {0.5*common::cm, 15.0*common::cm, 0.5*common::cm, 2};
}

// Full lidar matching the scenario file (fov_circles=3 -> 21 beams, z_max=20cm)
common::types::LidarConfigData fullLidar() {
    return {0.5*common::cm, 20.0*common::cm, 0.5*common::cm, 3};
}

common::types::DroneConfigData largeDrone()  { return {3.0*common::cm, 45.0*common::horizontal_angle[common::deg], 3.0*common::cm, 3.0*common::cm}; }
common::types::DroneConfigData mediumDrone() { return {2.0*common::cm, 45.0*common::horizontal_angle[common::deg], 2.0*common::cm, 2.0*common::cm}; }
common::types::DroneConfigData smallDrone()  { return {1.0*common::cm, 45.0*common::horizontal_angle[common::deg], 1.0*common::cm, 1.0*common::cm}; }

sim::types::SimulationCompositionData makeComp(
    common::types::DroneConfigData drone,
    common::types::LidarConfigData lidar,
    std::size_t max_steps)
{
    sim::types::SimulationCompositionData comp;
    comp.composition_file = "benchmark_test";
    comp.simulation_mission_groups.emplace_back(
        benchmarkSimConfig(),
        std::vector<common::types::MissionConfigData>{{max_steps, kResCm*common::cm, 1}}
    );
    comp.drone_configs = {drone};
    comp.lidar_configs = {lidar};
    return comp;
}

} // namespace

class BenchmarkIntegration : public ::testing::Test {
protected:
    static void SetUpTestSuite() {
        std::string error;
        algorithm_plugin_ = sim::loadAlgorithmPlugin(REAL_ALGORITHM_SO_PATH, error);
        ASSERT_TRUE(algorithm_plugin_.has_value())
            << "failed to load real Algorithm .so (" REAL_ALGORITHM_SO_PATH "): " << error;
        mission_control_plugin_ = sim::loadMissionControlPlugin(REAL_MISSION_CONTROL_SO_PATH, error);
        ASSERT_TRUE(mission_control_plugin_.has_value())
            << "failed to load real MissionControl .so (" REAL_MISSION_CONTROL_SO_PATH "): " << error;
    }

    static void TearDownTestSuite() {
        // Order between these two doesn't matter -- unrelated .so's. Each
        // optional's own destructor already destroys its captured factory
        // before its DynamicLibraryHandle (dlclose), per LoadedAlgorithmPlugin/
        // LoadedMissionControlPlugin's declared field order.
        mission_control_plugin_.reset();
        algorithm_plugin_.reset();
    }

    static std::unique_ptr<sim::SimulationRunFactoryImpl> makeRealFactory() {
        return std::make_unique<sim::SimulationRunFactoryImpl>(
            algorithm_plugin_->factory, mission_control_plugin_->factory);
    }

    static std::optional<sim::LoadedAlgorithmPlugin> algorithm_plugin_;
    static std::optional<sim::LoadedMissionControlPlugin> mission_control_plugin_;
};

std::optional<sim::LoadedAlgorithmPlugin> BenchmarkIntegration::algorithm_plugin_;
std::optional<sim::LoadedMissionControlPlugin> BenchmarkIntegration::mission_control_plugin_;

// Verify the map parses correctly to the advertised 29x30x31 voxel dimensions.
TEST_F(BenchmarkIntegration, MapLoadsAs29x30x31Voxels) {
    sim::Map3DImpl map("data_maps/benchmark_map.npy", kResCm * common::cm, common::Position3D{});
    const auto cfg = map.getMapConfig();
    EXPECT_NEAR(cfg.boundaries.max_x.force_numerical_value_in(common::cm), 29.0, 0.5);
    EXPECT_NEAR(cfg.boundaries.max_y.force_numerical_value_in(common::cm), 30.0, 0.5);
    EXPECT_NEAR(cfg.boundaries.max_height.force_numerical_value_in(common::cm), 31.0, 0.5);
}

// Large drone (3cm diameter, radius 1.5cm) fits through the 4x4 main entrance.
TEST_F(BenchmarkIntegration, LargeDrone_3cm_NoErrorInRun) {
    sim::SimulationManager manager(makeRealFactory());
    sim::types::SimulationManagerReport report;
    ASSERT_NO_THROW(report = manager.run(
        makeComp(largeDrone(), fastLidar(), 2000), "/tmp/bm_large_ex3"));
    ASSERT_EQ(report.runs.size(), 1u);
    ASSERT_FALSE(report.runs[0].mission_results.empty());
    EXPECT_NE(report.runs[0].mission_results[0].status, common::types::MissionRunStatus::Error)
        << "Large drone (3cm) errored inside benchmark map";
}

// Medium drone (2cm diameter, radius 1cm) fits through the 3x3 second-floor room entrance.
TEST_F(BenchmarkIntegration, MediumDrone_2cm_NoErrorInRun) {
    sim::SimulationManager manager(makeRealFactory());
    sim::types::SimulationManagerReport report;
    ASSERT_NO_THROW(report = manager.run(
        makeComp(mediumDrone(), fastLidar(), 2000), "/tmp/bm_medium_ex3"));
    ASSERT_EQ(report.runs.size(), 1u);
    ASSERT_FALSE(report.runs[0].mission_results.empty());
    EXPECT_NE(report.runs[0].mission_results[0].status, common::types::MissionRunStatus::Error)
        << "Medium drone (2cm) errored inside benchmark map";
}

// Small drone (1cm diameter, radius 0.5cm) fits through the 2x2 second-floor room entrance.
TEST_F(BenchmarkIntegration, SmallDrone_1cm_NoErrorInRun) {
    sim::SimulationManager manager(makeRealFactory());
    sim::types::SimulationManagerReport report;
    ASSERT_NO_THROW(report = manager.run(
        makeComp(smallDrone(), fastLidar(), 2000), "/tmp/bm_small_ex3"));
    ASSERT_EQ(report.runs.size(), 1u);
    ASSERT_FALSE(report.runs[0].mission_results.empty());
    EXPECT_NE(report.runs[0].mission_results[0].status, common::types::MissionRunStatus::Error)
        << "Small drone (1cm) errored inside benchmark map";
}

// All three drone sizes produce an output .npy map file.
TEST_F(BenchmarkIntegration, AllDroneSizes_OutputMapFileCreated) {
    sim::SimulationManager manager(makeRealFactory());

    sim::types::SimulationCompositionData comp;
    comp.composition_file = "benchmark_multi";
    comp.simulation_mission_groups.emplace_back(
        benchmarkSimConfig(),
        std::vector<common::types::MissionConfigData>{{500, kResCm*common::cm, 1}}
    );
    comp.drone_configs = {largeDrone(), mediumDrone(), smallDrone()};
    comp.lidar_configs = {fastLidar()};

    const std::filesystem::path out = "/tmp/bm_multi_drone_ex3";
    const auto report = manager.run(comp, out);

    EXPECT_EQ(report.runs.size(), 3u);
    bool found_npy = false;
    for (const auto& e : std::filesystem::recursive_directory_iterator(out / "output_results"))
        if (e.path().extension() == ".npy") { found_npy = true; break; }
    EXPECT_TRUE(found_npy) << "No .npy output map found under " << out;
}

// Small drone achieves a positive mapping score on the benchmark map.
TEST_F(BenchmarkIntegration, SmallDrone_AchievesPositiveScore) {
    sim::SimulationManager manager(makeRealFactory());
    const auto report = manager.run(
        makeComp(smallDrone(), fastLidar(), 3000), "/tmp/bm_score_ex3");
    ASSERT_GE(report.runs.size(), 1u);
    EXPECT_GT(report.runs[0].mission_score, 0.0)
        << "Expected positive score; got " << report.runs[0].mission_score;
}

// Full run with the scenario-equivalent config must finish under 60 seconds.
// Uses small drone (most steps needed) and full lidar matching scenario file.
TEST_F(BenchmarkIntegration, FullRun_CompletesUnder60Seconds) {
    sim::SimulationManager manager(makeRealFactory());

    const auto t0 = std::chrono::steady_clock::now();
    sim::types::SimulationManagerReport report;
    ASSERT_NO_THROW(report = manager.run(
        makeComp(smallDrone(), fullLidar(), 100000), "/tmp/bm_runtime_ex3"));
    const auto elapsed =
        std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now() - t0).count();

    EXPECT_LT(elapsed, 60) << "Full benchmark run took " << elapsed << "s (limit: 60s)";
    ASSERT_GE(report.runs.size(), 1u);
    EXPECT_NE(report.runs[0].mission_results[0].status, common::types::MissionRunStatus::Error);
}
