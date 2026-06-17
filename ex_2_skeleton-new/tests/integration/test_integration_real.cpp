#include <gtest/gtest.h>

#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/SimulationManager.h>
#include <drone_mapper/SimulationRunFactoryImpl.h>

#include <filesystem>

using namespace drone_mapper;

namespace {

types::SimulationCompositionData singleVoxelComposition() {
    return {
        "composition.yaml",
        {types::SimulationConfigData{
            "data_maps/single_voxel_x2_y4_z2.npy",
            10.0*cm,
            Position3D{},
            Position3D{5.0*x_extent[cm], 5.0*y_extent[cm], 5.0*z_extent[cm]},
            0.0*horizontal_angle[deg],
        }},
        {types::MissionConfigData{500, 10.0*cm, 1}},
        {types::DroneConfigData{10.0*cm, 45.0*horizontal_angle[deg], 10.0*cm, 10.0*cm}},
        {types::LidarConfigData{5.0*cm, 50.0*cm, 2.5*cm, 2}},
    };
}

} // namespace

class Integration : public ::testing::Test {};

TEST_F(Integration, FullRunWithRealAlgorithmCompletesWithoutError) {
    auto factory = std::make_unique<SimulationRunFactoryImpl>();
    drone_mapper::SimulationManager manager(std::move(factory));

    const std::filesystem::path out = "/tmp/integration_real_test";
    types::SimulationManagerReport report;
    ASSERT_NO_THROW(report = manager.run(singleVoxelComposition(), out));

    ASSERT_GE(report.runs.size(), 1u);
    const auto& run = report.runs[0];
    ASSERT_FALSE(run.mission_results.empty());
    EXPECT_NE(run.mission_results[0].status, types::MissionRunStatus::Error)
        << "Integration run errored";
}

TEST_F(Integration, OutputMapFileIsCreated) {
    auto factory = std::make_unique<SimulationRunFactoryImpl>();
    drone_mapper::SimulationManager manager(std::move(factory));

    const std::filesystem::path out = "/tmp/integration_real_file_test";
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
    composition.missions.push_back(types::MissionConfigData{200, 10.0*cm, 1});

    auto factory = std::make_unique<SimulationRunFactoryImpl>();
    drone_mapper::SimulationManager manager(std::move(factory));

    types::SimulationManagerReport report;
    ASSERT_NO_THROW(report = manager.run(composition, "/tmp/integration_multi_mission"));
    EXPECT_EQ(report.runs.size(), 2u);
    for (const auto& run : report.runs)
        EXPECT_NE(run.mission_results[0].status, types::MissionRunStatus::Error);
}

TEST_F(Integration, InvalidMapFileRunsErrorWithMinusOneScore) {
    types::SimulationCompositionData bad_composition = singleVoxelComposition();
    bad_composition.simulations[0].map_filename = "data_maps/does_not_exist.npy";

    auto factory = std::make_unique<SimulationRunFactoryImpl>();
    drone_mapper::SimulationManager manager(std::move(factory));

    types::SimulationManagerReport report;
    ASSERT_NO_THROW(report = manager.run(bad_composition, "/tmp/integration_bad_map"));
    ASSERT_EQ(report.runs.size(), 1u);
    EXPECT_DOUBLE_EQ(report.runs[0].mission_score, -1.0);
    EXPECT_EQ(report.runs[0].mission_results[0].status, types::MissionRunStatus::Error);
}
