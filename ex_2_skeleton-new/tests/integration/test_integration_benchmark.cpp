#include <gtest/gtest.h>

#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/SimulationManager.h>
#include <drone_mapper/SimulationRunFactoryImpl.h>

#include <chrono>
#include <filesystem>

using namespace drone_mapper;

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

types::SimulationConfigData benchmarkSimConfig() {
    return {
        "data_maps/benchmark_map.npy",
        kResCm * cm,
        Position3D{},
        Position3D{22.0*x_extent[cm], 27.0*y_extent[cm], 17.0*z_extent[cm]},
        0.0 * horizontal_angle[deg],
    };
}

// Fast lidar for integration tests (fov_circles=2 → 5 beams, z_max=15cm)
types::LidarConfigData fastLidar() {
    return {0.5*cm, 15.0*cm, 0.5*cm, 2};
}

// Full lidar matching the scenario file (fov_circles=3 → 21 beams, z_max=20cm)
types::LidarConfigData fullLidar() {
    return {0.5*cm, 20.0*cm, 0.5*cm, 3};
}

types::DroneConfigData largeDrone()  { return {3.0*cm, 45.0*horizontal_angle[deg], 3.0*cm, 3.0*cm}; }
types::DroneConfigData mediumDrone() { return {2.0*cm, 45.0*horizontal_angle[deg], 2.0*cm, 2.0*cm}; }
types::DroneConfigData smallDrone()  { return {1.0*cm, 45.0*horizontal_angle[deg], 1.0*cm, 1.0*cm}; }

types::SimulationCompositionData makeComp(
    types::DroneConfigData drone,
    types::LidarConfigData lidar,
    std::size_t max_steps)
{
    types::SimulationCompositionData comp;
    comp.composition_file = "benchmark_test";
    comp.simulation_mission_groups.emplace_back(
        benchmarkSimConfig(),
        std::vector<types::MissionConfigData>{{max_steps, kResCm*cm, 1}}
    );
    comp.drones = {drone};
    comp.lidars = {lidar};
    return comp;
}

} // namespace

class BenchmarkIntegration : public ::testing::Test {};

// Verify the map parses correctly to the advertised 29x30x31 voxel dimensions.
TEST_F(BenchmarkIntegration, MapLoadsAs29x30x31Voxels) {
    Map3DImpl map("data_maps/benchmark_map.npy", kResCm * cm, Position3D{});
    const auto cfg = map.getMapConfig();
    EXPECT_NEAR(cfg.boundaries.max_x.force_numerical_value_in(cm), 29.0, 0.5);
    EXPECT_NEAR(cfg.boundaries.max_y.force_numerical_value_in(cm), 30.0, 0.5);
    EXPECT_NEAR(cfg.boundaries.max_height.force_numerical_value_in(cm), 31.0, 0.5);
}

// Large drone (3cm diameter, radius 1.5cm) fits through the 4x4 main entrance.
TEST_F(BenchmarkIntegration, LargeDrone_3cm_NoErrorInRun) {
    auto factory = std::make_unique<SimulationRunFactoryImpl>();
    SimulationManager manager(std::move(factory));
    types::SimulationManagerReport report;
    ASSERT_NO_THROW(report = manager.run(
        makeComp(largeDrone(), fastLidar(), 2000), "/tmp/bm_large"));
    ASSERT_EQ(report.runs.size(), 1u);
    ASSERT_FALSE(report.runs[0].mission_results.empty());
    EXPECT_NE(report.runs[0].mission_results[0].status, types::MissionRunStatus::Error)
        << "Large drone (3cm) errored inside benchmark map";
}

// Medium drone (2cm diameter, radius 1cm) fits through the 3x3 second-floor room entrance.
TEST_F(BenchmarkIntegration, MediumDrone_2cm_NoErrorInRun) {
    auto factory = std::make_unique<SimulationRunFactoryImpl>();
    SimulationManager manager(std::move(factory));
    types::SimulationManagerReport report;
    ASSERT_NO_THROW(report = manager.run(
        makeComp(mediumDrone(), fastLidar(), 2000), "/tmp/bm_medium"));
    ASSERT_EQ(report.runs.size(), 1u);
    ASSERT_FALSE(report.runs[0].mission_results.empty());
    EXPECT_NE(report.runs[0].mission_results[0].status, types::MissionRunStatus::Error)
        << "Medium drone (2cm) errored inside benchmark map";
}

// Small drone (1cm diameter, radius 0.5cm) fits through the 2x2 second-floor room entrance.
TEST_F(BenchmarkIntegration, SmallDrone_1cm_NoErrorInRun) {
    auto factory = std::make_unique<SimulationRunFactoryImpl>();
    SimulationManager manager(std::move(factory));
    types::SimulationManagerReport report;
    ASSERT_NO_THROW(report = manager.run(
        makeComp(smallDrone(), fastLidar(), 2000), "/tmp/bm_small"));
    ASSERT_EQ(report.runs.size(), 1u);
    ASSERT_FALSE(report.runs[0].mission_results.empty());
    EXPECT_NE(report.runs[0].mission_results[0].status, types::MissionRunStatus::Error)
        << "Small drone (1cm) errored inside benchmark map";
}

// All three drone sizes produce an output .npy map file.
TEST_F(BenchmarkIntegration, AllDroneSizes_OutputMapFileCreated) {
    auto factory = std::make_unique<SimulationRunFactoryImpl>();
    SimulationManager manager(std::move(factory));

    types::SimulationCompositionData comp;
    comp.composition_file = "benchmark_multi";
    comp.simulation_mission_groups.emplace_back(
        benchmarkSimConfig(),
        std::vector<types::MissionConfigData>{{500, kResCm*cm, 1}}
    );
    comp.drones = {largeDrone(), mediumDrone(), smallDrone()};
    comp.lidars = {fastLidar()};

    const std::filesystem::path out = "/tmp/bm_multi_drone";
    const auto report = manager.run(comp, out);

    EXPECT_EQ(report.runs.size(), 3u);
    bool found_npy = false;
    for (const auto& e : std::filesystem::recursive_directory_iterator(out / "output_results"))
        if (e.path().extension() == ".npy") { found_npy = true; break; }
    EXPECT_TRUE(found_npy) << "No .npy output map found under " << out;
}

// Small drone achieves a positive mapping score on the benchmark map.
TEST_F(BenchmarkIntegration, SmallDrone_AchievesPositiveScore) {
    auto factory = std::make_unique<SimulationRunFactoryImpl>();
    SimulationManager manager(std::move(factory));
    const auto report = manager.run(
        makeComp(smallDrone(), fastLidar(), 3000), "/tmp/bm_score");
    ASSERT_GE(report.runs.size(), 1u);
    EXPECT_GT(report.runs[0].mission_score, 0.0)
        << "Expected positive score; got " << report.runs[0].mission_score;
}

// Full run with the scenario-equivalent config must finish under 60 seconds.
// Uses small drone (most steps needed) and full lidar matching scenario file.
TEST_F(BenchmarkIntegration, FullRun_CompletesUnder60Seconds) {
    auto factory = std::make_unique<SimulationRunFactoryImpl>();
    SimulationManager manager(std::move(factory));

    const auto t0 = std::chrono::steady_clock::now();
    types::SimulationManagerReport report;
    ASSERT_NO_THROW(report = manager.run(
        makeComp(smallDrone(), fullLidar(), 100000), "/tmp/bm_runtime"));
    const auto elapsed =
        std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now() - t0).count();

    EXPECT_LT(elapsed, 60) << "Full benchmark run took " << elapsed << "s (limit: 60s)";
    ASSERT_GE(report.runs.size(), 1u);
    EXPECT_NE(report.runs[0].mission_results[0].status, types::MissionRunStatus::Error);
}
