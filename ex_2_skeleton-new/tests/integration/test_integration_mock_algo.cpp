#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <drone_mapper/DroneControlImpl.h>
#include <drone_mapper/IMappingAlgorithm.h>
#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MissionControlImpl.h>
#include <drone_mapper/MockGPS.h>
#include <drone_mapper/MockLidar.h>
#include <drone_mapper/MockMovement.h>
#include <drone_mapper/SimulationRunImpl.h>

using namespace drone_mapper;
using ::testing::_;
using ::testing::Return;

class DoneAlgorithm final : public IMappingAlgorithm {
public:
    struct NullMap : IMap3D {
        types::VoxelOccupancy atVoxel(const Position3D&) const override {
            return types::VoxelOccupancy::Empty;
        }
        types::MapConfig getMapConfig() const override { return {}; }
        bool isInBounds(const Position3D&) const override { return false; }
    };
    DoneAlgorithm() : IMappingAlgorithm(types::MissionConfigData{}, types::LidarConfigData{}, types::DroneConfigData{}, null_map_) {}
    types::MappingStepCommand nextStep(const types::DroneState&,
                                       const types::LidarScanResult*) override {
        return {std::nullopt, std::nullopt, types::AlgorithmStatus::Finished};
    }
private:
    NullMap null_map_;
};

namespace {

types::MappingBounds testBounds() {
    return {
        0.0*x_extent[cm], 200.0*x_extent[cm],
        0.0*y_extent[cm], 200.0*y_extent[cm],
        0.0*z_extent[cm], 200.0*z_extent[cm],
    };
}

types::SimulationConfigData testSimCfg() {
    return {"data_maps/single_voxel_x2_y4_z2.npy", 10.0*cm, Position3D{},
            Position3D{50.0*x_extent[cm], 50.0*y_extent[cm], 50.0*z_extent[cm]},
            0.0*horizontal_angle[deg]};
}

types::MissionConfigData testMission() {
    return {500, 10.0*cm, 1};
}

} // namespace

class Integration : public ::testing::Test {};

TEST_F(Integration, MockAlgorithmRun_CompletesImmediately) {
    auto bounds  = testBounds();
    auto mission = testMission();
    types::DroneConfigData drone{30.0*cm, 45.0*horizontal_angle[deg], 50.0*cm, 40.0*cm};
    types::LidarConfigData lidar{20.0*cm, 120.0*cm, 2.5*cm, 2};

    auto hidden_map = std::make_unique<Map3DImpl>(
        "data_maps/single_voxel_x2_y4_z2.npy", 10.0*cm);
    auto output_map = std::make_unique<Map3DImpl>(bounds, 10.0*cm, Position3D{});

    auto gps = std::make_unique<MockGPS>(
        Position3D{50.0*x_extent[cm], 50.0*y_extent[cm], 50.0*z_extent[cm]},
        Orientation{0.0*horizontal_angle[deg], 0.0*altitude_angle[deg]},
        10.0*cm);
    auto movement = std::make_unique<MockMovement>(*gps, *hidden_map, drone);
    auto lidar_sensor = std::make_unique<MockLidar>(lidar, *hidden_map, *gps);
    auto algorithm = std::make_unique<DoneAlgorithm>();

    auto drone_ctrl = std::make_unique<DroneControlImpl>(
        drone, mission, lidar, *lidar_sensor, *gps, *movement, *output_map, *algorithm);

    const std::filesystem::path out_file = "/tmp/integration_mock_algo.npy";
    auto mission_ctrl = std::make_unique<MissionControlImpl>(
        mission, drone, *hidden_map, *output_map, *drone_ctrl, out_file);

    SimulationRunImpl run(
        std::move(hidden_map), std::move(output_map),
        std::move(gps), std::move(movement), std::move(lidar_sensor),
        std::move(algorithm), std::move(drone_ctrl), std::move(mission_ctrl),
        testSimCfg(), testMission(), out_file,
        types::ResolutionRequestStatus::Accepted);

    types::SimulationResult result;
    ASSERT_NO_THROW(result = run.run());

    ASSERT_FALSE(result.mission_results.empty());
    EXPECT_EQ(result.mission_results[0].status, types::MissionRunStatus::Completed);
    EXPECT_EQ(result.mission_results[0].steps, 0u);
    EXPECT_GE(result.mission_score, 0.0);
}

TEST_F(Integration, MockAlgorithmRun_OutputMapFileExists) {
    auto bounds  = testBounds();
    auto mission = testMission();
    types::DroneConfigData drone{30.0*cm, 45.0*horizontal_angle[deg], 50.0*cm, 40.0*cm};
    types::LidarConfigData lidar{20.0*cm, 120.0*cm, 2.5*cm, 2};

    auto hidden_map = std::make_unique<Map3DImpl>(
        "data_maps/single_voxel_x2_y4_z2.npy", 10.0*cm);
    auto output_map = std::make_unique<Map3DImpl>(bounds, 10.0*cm, Position3D{});
    auto gps = std::make_unique<MockGPS>(
        Position3D{50.0*x_extent[cm], 50.0*y_extent[cm], 50.0*z_extent[cm]},
        Orientation{0.0*horizontal_angle[deg], 0.0*altitude_angle[deg]},
        10.0*cm);
    auto movement = std::make_unique<MockMovement>(*gps, *hidden_map, drone);
    auto lidar_sensor = std::make_unique<MockLidar>(lidar, *hidden_map, *gps);
    auto algorithm = std::make_unique<DoneAlgorithm>();
    auto drone_ctrl = std::make_unique<DroneControlImpl>(
        drone, mission, lidar, *lidar_sensor, *gps, *movement, *output_map, *algorithm);
    const std::filesystem::path out_file = "/tmp/integration_mock_file_check.npy";
    std::filesystem::remove(out_file);
    auto mission_ctrl = std::make_unique<MissionControlImpl>(
        mission, drone, *hidden_map, *output_map, *drone_ctrl, out_file);

    SimulationRunImpl run(
        std::move(hidden_map), std::move(output_map),
        std::move(gps), std::move(movement), std::move(lidar_sensor),
        std::move(algorithm), std::move(drone_ctrl), std::move(mission_ctrl),
        testSimCfg(), testMission(), out_file,
        types::ResolutionRequestStatus::Accepted);
    std::ignore = run.run();

    EXPECT_TRUE(std::filesystem::exists(out_file))
        << "Output map .npy was not created at " << out_file;
}
