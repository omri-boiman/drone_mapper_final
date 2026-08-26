#include <gtest/gtest.h>

#include <Simulator/Map3DImpl.h>
#include <Simulator/SimulationManager.h>
#include <Simulator/SimulationRunFactoryImpl.h>

#include "StubPlugins.h"

#include <filesystem>

namespace sim = simulator;

namespace {

sim::types::SimulationCompositionData singleVoxelComposition() {
    sim::types::SimulationCompositionData comp;
    comp.composition_file = "composition.yaml";
    comp.simulation_mission_groups.emplace_back(
        sim::types::SimulationConfigData{
            "data_maps/single_voxel_x2_y4_z2.npy",
            10.0*common::cm,
            common::Position3D{},
            common::Position3D{5.0*common::x_extent[common::cm], 5.0*common::y_extent[common::cm], 5.0*common::z_extent[common::cm]},
            0.0*common::horizontal_angle[common::deg],
        },
        std::vector<common::types::MissionConfigData>{{500, 10.0*common::cm, 1}}
    );
    comp.drone_configs = {common::types::DroneConfigData{10.0*common::cm, 45.0*common::horizontal_angle[common::deg], 10.0*common::cm, 10.0*common::cm}};
    comp.lidar_configs = {common::types::LidarConfigData{5.0*common::cm, 50.0*common::cm, 2.5*common::cm, 2}};
    return comp;
}

std::unique_ptr<sim::SimulationRunFactoryImpl> makeStubFactory() {
    return std::make_unique<sim::SimulationRunFactoryImpl>(
        sim::test::stubAlgorithmFactory(), sim::test::stubMissionControlFactory());
}

} // namespace

// These exercise SimulationManager/SimulationRunFactoryImpl/SimulationRunImpl
// wiring end to end using trivial stub algorithm/mission-control factories --
// see StubPlugins.h for why real Algorithm/MissionControl aren't used here.
// The real end-to-end pipeline (actual .so plugins, reproducing the ex2
// baseline scores) is verified separately by running the built executable.
class Integration : public ::testing::Test {};

TEST_F(Integration, FullRunCompletesWithoutError) {
    sim::SimulationManager manager(makeStubFactory());

    const std::filesystem::path out = "/tmp/integration_pipeline_test";
    sim::types::SimulationManagerReport report;
    ASSERT_NO_THROW(report = manager.run(singleVoxelComposition(), out));

    ASSERT_GE(report.runs.size(), 1u);
    const auto& run = report.runs[0];
    ASSERT_FALSE(run.mission_results.empty());
    EXPECT_NE(run.mission_results[0].status, common::types::MissionRunStatus::Error)
        << "Integration run errored";
}

TEST_F(Integration, OutputMapFileIsCreated) {
    sim::SimulationManager manager(makeStubFactory());

    const std::filesystem::path out = "/tmp/integration_pipeline_file_test";
    const auto report = manager.run(singleVoxelComposition(), out);

    bool found_npy = false;
    for (const auto& entry :
         std::filesystem::recursive_directory_iterator(out / "output_results")) {
        if (entry.path().extension() == ".npy") { found_npy = true; break; }
    }
    EXPECT_TRUE(found_npy) << "No .npy output map found under " << out;
}

TEST_F(Integration, MultiMissionCompositionRunsAll) {
    // 1 sim × 2 missions × 1 drone × 1 lidar = 2 runs
    auto composition = singleVoxelComposition();
    std::get<1>(composition.simulation_mission_groups[0]).push_back(
        common::types::MissionConfigData{200, 10.0*common::cm, 1});

    sim::SimulationManager manager(makeStubFactory());

    sim::types::SimulationManagerReport report;
    ASSERT_NO_THROW(report = manager.run(composition, "/tmp/integration_pipeline_multi_mission"));
    EXPECT_EQ(report.runs.size(), 2u);
    for (const auto& run : report.runs)
        EXPECT_NE(run.mission_results[0].status, common::types::MissionRunStatus::Error);
}

TEST_F(Integration, InvalidMapFileRunsErrorWithMinusOneScore) {
    sim::types::SimulationCompositionData bad_composition = singleVoxelComposition();
    std::get<0>(bad_composition.simulation_mission_groups[0]).map_filename = "data_maps/does_not_exist.npy";

    sim::SimulationManager manager(makeStubFactory());

    sim::types::SimulationManagerReport report;
    ASSERT_NO_THROW(report = manager.run(bad_composition, "/tmp/integration_pipeline_bad_map"));
    ASSERT_EQ(report.runs.size(), 1u);
    EXPECT_DOUBLE_EQ(report.runs[0].mission_score, -1.0);
    EXPECT_EQ(report.runs[0].mission_results[0].status, common::types::MissionRunStatus::Error);
}

TEST_F(Integration, MapLoadsAs29x30x31Voxels) {
    sim::Map3DImpl map("data_maps/benchmark_map.npy", 1.0*common::cm, common::Position3D{});
    const auto cfg = map.getMapConfig();
    EXPECT_NEAR(cfg.boundaries.max_x.force_numerical_value_in(common::cm), 29.0, 0.5);
    EXPECT_NEAR(cfg.boundaries.max_y.force_numerical_value_in(common::cm), 30.0, 0.5);
    EXPECT_NEAR(cfg.boundaries.max_height.force_numerical_value_in(common::cm), 31.0, 0.5);
}
