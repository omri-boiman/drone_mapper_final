#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <drone_mapper/DroneControlImpl.h>
#include <drone_mapper/IDroneMovement.h>
#include <drone_mapper/IGPS.h>
#include <drone_mapper/ILidar.h>
#include <drone_mapper/IMappingAlgorithm.h>
#include <drone_mapper/IMutableMap3D.h>
#include <drone_mapper/Map3DImpl.h>

using namespace drone_mapper;
using ::testing::Return;
using ::testing::_;
using ::testing::AtLeast;

class MockGPS_DC : public IGPS {
public:
    MOCK_METHOD(Position3D,  position, (), (const, override));
    MOCK_METHOD(Orientation, heading,  (), (const, override));
};

class MockLidar_DC : public ILidar {
public:
    MOCK_METHOD(types::LidarScanResult, scan, (Orientation), (const, override));
};

class MockMovement_DC : public IDroneMovement {
public:
    MOCK_METHOD(types::MovementResult, rotate,
                (types::RotationDirection, HorizontalAngle), (override));
    MOCK_METHOD(types::MovementResult, advance, (PhysicalLength), (override));
    MOCK_METHOD(types::MovementResult, elevate, (PhysicalLength), (override));
};

class MockMap_DC : public IMutableMap3D {
public:
    MOCK_METHOD(types::VoxelOccupancy, atVoxel, (const Position3D&), (const, override));
    MOCK_METHOD(types::MapConfig,      getMapConfig, (), (const, override));
    MOCK_METHOD(bool, isInBounds, (const Position3D&), (const, override));
    MOCK_METHOD(void, set,  (const Position3D&, types::VoxelOccupancy), (override));
    MOCK_METHOD(void, save, (const std::filesystem::path&), (const, override));
};

class MockAlgorithm_DC : public IMappingAlgorithm {
public:
    struct NullMap : IMap3D {
        types::VoxelOccupancy atVoxel(const Position3D&) const override {
            return types::VoxelOccupancy::Empty;
        }
        types::MapConfig getMapConfig() const override { return {}; }
        bool isInBounds(const Position3D&) const override { return false; }
    };
    MockAlgorithm_DC() : IMappingAlgorithm(types::MissionConfigData{}, types::LidarConfigData{}, types::DroneConfigData{}, null_map_) {}
    MOCK_METHOD(types::MappingStepCommand, nextStep,
                (const types::DroneState&, const types::LidarScanResult*), (override));
private:
    NullMap null_map_;
};

namespace {

types::DroneConfigData defaultDrone() {
    return {30.0*cm, 45.0*horizontal_angle[deg], 50.0*cm, 40.0*cm};
}

types::MissionConfigData defaultMission() {
    return {2400, 10.0*cm, 1};
}

Position3D defaultPos() {
    return {100.0*x_extent[cm], 100.0*y_extent[cm], 100.0*z_extent[cm]};
}

Orientation defaultHeading() {
    return {0.0*horizontal_angle[deg], 0.0*altitude_angle[deg]};
}

types::MapConfig defaultMapConfig() {
    types::MappingBounds b{
        0.0*x_extent[cm], 500.0*x_extent[cm],
        0.0*y_extent[cm], 500.0*y_extent[cm],
        0.0*z_extent[cm], 300.0*z_extent[cm],
    };
    return {b, Position3D{}, 10.0*cm};
}

} // namespace

class DroneControl : public ::testing::Test {
protected:
    void SetUp() override {
        ON_CALL(gps, position()).WillByDefault(Return(defaultPos()));
        ON_CALL(gps, heading()).WillByDefault(Return(defaultHeading()));
        ON_CALL(lidar, scan(_)).WillByDefault(Return(types::LidarScanResult{}));
        ON_CALL(map, atVoxel(_)).WillByDefault(Return(types::VoxelOccupancy::Empty));
        ON_CALL(map, getMapConfig()).WillByDefault(Return(defaultMapConfig()));
        ON_CALL(map, isInBounds(_)).WillByDefault(Return(true));
        ON_CALL(algo, nextStep(_, _)).WillByDefault(Return(
            types::MappingStepCommand{
                std::nullopt,
                Orientation{0.0*horizontal_angle[deg], 0.0*altitude_angle[deg]},
                types::AlgorithmStatus::Working
            }));
        ON_CALL(movement, rotate(_, _)).WillByDefault(Return(types::MovementResult{true}));
        ON_CALL(movement, advance(_)).WillByDefault(Return(types::MovementResult{true}));
        ON_CALL(movement, elevate(_)).WillByDefault(Return(types::MovementResult{true}));
    }

    MockGPS_DC       gps;
    MockLidar_DC     lidar;
    MockMovement_DC  movement;
    MockMap_DC       map;
    MockAlgorithm_DC algo;
};

TEST_F(DroneControl, StepCallsLidarScan) {
    EXPECT_CALL(lidar, scan(_)).Times(AtLeast(1));
    DroneControlImpl ctrl(defaultDrone(), defaultMission(), {}, lidar, gps, movement, map, algo);
    std::ignore = ctrl.step();
}

TEST_F(DroneControl, StepCallsNextStep) {
    EXPECT_CALL(algo, nextStep(_, _)).Times(AtLeast(1));
    DroneControlImpl ctrl(defaultDrone(), defaultMission(), {}, lidar, gps, movement, map, algo);
    std::ignore = ctrl.step();
}

TEST_F(DroneControl, StepReturnsCompletedWhenAlgorithmReturnsFinished) {
    ON_CALL(algo, nextStep(_, _)).WillByDefault(Return(
        types::MappingStepCommand{std::nullopt, std::nullopt, types::AlgorithmStatus::Finished}));
    DroneControlImpl ctrl(defaultDrone(), defaultMission(), {}, lidar, gps, movement, map, algo);
    const auto result = ctrl.step();
    EXPECT_EQ(result.status, types::DroneStepStatus::Completed);
}

TEST_F(DroneControl, StepReturnsErrorOnCollision) {
    ON_CALL(algo, nextStep(_, _)).WillByDefault(Return(
        types::MappingStepCommand{
            types::MovementCommand{types::MovementCommandType::Advance,
                                   types::RotationDirection::Left,
                                   0.0*horizontal_angle[deg],
                                   30.0*cm},
            std::nullopt,
            types::AlgorithmStatus::Working
        }));
    ON_CALL(movement, advance(_)).WillByDefault(
        Return(types::MovementResult{false, "DRONE_HITS_OBSTACLE"}));

    DroneControlImpl ctrl(defaultDrone(), defaultMission(), {}, lidar, gps, movement, map, algo);
    const auto result = ctrl.step();
    EXPECT_EQ(result.status, types::DroneStepStatus::Error);
    EXPECT_EQ(result.message, "DRONE_HITS_OBSTACLE");
}

TEST_F(DroneControl, StepReturnsContinueOnSuccessfulAdvance) {
    ON_CALL(algo, nextStep(_, _)).WillByDefault(Return(
        types::MappingStepCommand{
            types::MovementCommand{types::MovementCommandType::Advance,
                                   types::RotationDirection::Left,
                                   0.0*horizontal_angle[deg],
                                   30.0*cm},
            std::nullopt,
            types::AlgorithmStatus::Working
        }));
    DroneControlImpl ctrl(defaultDrone(), defaultMission(), {}, lidar, gps, movement, map, algo);
    const auto result = ctrl.step();
    EXPECT_EQ(result.status, types::DroneStepStatus::Continue);
}

TEST_F(DroneControl, StateReflectsGPSValues) {
    DroneControlImpl ctrl(defaultDrone(), defaultMission(), {}, lidar, gps, movement, map, algo);
    const auto s = ctrl.state();
    EXPECT_DOUBLE_EQ(s.position.x.force_numerical_value_in(cm), 100.0);
    EXPECT_DOUBLE_EQ(s.position.y.force_numerical_value_in(cm), 100.0);
    EXPECT_DOUBLE_EQ(s.position.z.force_numerical_value_in(cm), 100.0);
}

TEST_F(DroneControl, RotateCommandCallsMovementRotate) {
    ON_CALL(algo, nextStep(_, _)).WillByDefault(Return(
        types::MappingStepCommand{
            types::MovementCommand{types::MovementCommandType::Rotate,
                                   types::RotationDirection::Right,
                                   45.0*horizontal_angle[deg], 0.0*cm},
            std::nullopt, types::AlgorithmStatus::Working}));
    EXPECT_CALL(movement, rotate(types::RotationDirection::Right, _)).Times(AtLeast(1));
    EXPECT_CALL(movement, advance(_)).Times(0);
    DroneControlImpl ctrl(defaultDrone(), defaultMission(), {}, lidar, gps, movement, map, algo);
    std::ignore = ctrl.step();
}

TEST_F(DroneControl, RotateFailureReturnsError) {
    ON_CALL(algo, nextStep(_, _)).WillByDefault(Return(
        types::MappingStepCommand{
            types::MovementCommand{types::MovementCommandType::Rotate,
                                   types::RotationDirection::Left,
                                   45.0*horizontal_angle[deg], 0.0*cm},
            std::nullopt, types::AlgorithmStatus::Working}));
    ON_CALL(movement, rotate(_, _)).WillByDefault(
        Return(types::MovementResult{false, "DRONE_HITS_OBSTACLE"}));
    DroneControlImpl ctrl(defaultDrone(), defaultMission(), {}, lidar, gps, movement, map, algo);
    const auto result = ctrl.step();
    EXPECT_EQ(result.status, types::DroneStepStatus::Error);
}

TEST_F(DroneControl, ElevateCommandCallsMovementElevate) {
    ON_CALL(algo, nextStep(_, _)).WillByDefault(Return(
        types::MappingStepCommand{
            types::MovementCommand{types::MovementCommandType::Elevate,
                                   types::RotationDirection::Left,
                                   0.0*horizontal_angle[deg], 20.0*cm},
            std::nullopt, types::AlgorithmStatus::Working}));
    EXPECT_CALL(movement, elevate(_)).Times(AtLeast(1));
    EXPECT_CALL(movement, advance(_)).Times(0);
    DroneControlImpl ctrl(defaultDrone(), defaultMission(), {}, lidar, gps, movement, map, algo);
    std::ignore = ctrl.step();
}

TEST_F(DroneControl, ElevateFailureReturnsError) {
    ON_CALL(algo, nextStep(_, _)).WillByDefault(Return(
        types::MappingStepCommand{
            types::MovementCommand{types::MovementCommandType::Elevate,
                                   types::RotationDirection::Left,
                                   0.0*horizontal_angle[deg], 20.0*cm},
            std::nullopt, types::AlgorithmStatus::Working}));
    ON_CALL(movement, elevate(_)).WillByDefault(
        Return(types::MovementResult{false, "DRONE_HITS_OBSTACLE"}));
    DroneControlImpl ctrl(defaultDrone(), defaultMission(), {}, lidar, gps, movement, map, algo);
    const auto result = ctrl.step();
    EXPECT_EQ(result.status, types::DroneStepStatus::Error);
    EXPECT_EQ(result.message, "DRONE_HITS_OBSTACLE");
}

TEST_F(DroneControl, ScanOrientationIsPassedToLidar) {
    const Orientation expected_scan{45.0*horizontal_angle[deg], 10.0*altitude_angle[deg]};
    ON_CALL(algo, nextStep(_, _)).WillByDefault(Return(
        types::MappingStepCommand{std::nullopt, expected_scan, types::AlgorithmStatus::Working}));
    EXPECT_CALL(lidar, scan(_)).WillOnce([&](Orientation o) -> types::LidarScanResult {
        EXPECT_DOUBLE_EQ(o.horizontal.force_numerical_value_in(deg), 45.0);
        EXPECT_DOUBLE_EQ(o.altitude.force_numerical_value_in(deg), 10.0);
        return {};
    });
    DroneControlImpl ctrl(defaultDrone(), defaultMission(), {}, lidar, gps, movement, map, algo);
    std::ignore = ctrl.step();
}

TEST_F(DroneControl, ScanHitIsAppliedToOutputMap) {
    // Drone at (50,50,50) facing east. Lidar returns a hit at 30cm along the
    // forward beam. After one step, the voxel at (80,50,50) must be Occupied.
    types::MappingBounds b{
        0.0*x_extent[cm], 200.0*x_extent[cm],
        0.0*y_extent[cm], 200.0*y_extent[cm],
        0.0*z_extent[cm], 200.0*z_extent[cm],
    };
    Map3DImpl real_map(b, 10.0*cm, Position3D{});

    MockGPS_DC      my_gps;
    MockLidar_DC    my_lidar;
    MockMovement_DC my_movement;
    MockAlgorithm_DC my_algo;

    const Position3D drone_pos{50.0*x_extent[cm], 50.0*y_extent[cm], 50.0*z_extent[cm]};
    const Orientation east{0.0*horizontal_angle[deg], 0.0*altitude_angle[deg]};

    ON_CALL(my_gps, position()).WillByDefault(Return(drone_pos));
    ON_CALL(my_gps, heading()).WillByDefault(Return(east));
    ON_CALL(my_movement, rotate(_, _)).WillByDefault(Return(types::MovementResult{true}));
    ON_CALL(my_movement, advance(_)).WillByDefault(Return(types::MovementResult{true}));
    ON_CALL(my_movement, elevate(_)).WillByDefault(Return(types::MovementResult{true}));

    // Algorithm requests a forward scan, no movement
    ON_CALL(my_algo, nextStep(_, _)).WillByDefault(Return(
        types::MappingStepCommand{std::nullopt, east, types::AlgorithmStatus::Working}));

    // Lidar returns one hit at 30cm straight ahead (relative angle {0,0})
    ON_CALL(my_lidar, scan(_)).WillByDefault(Return(
        types::LidarScanResult{{30.0*cm, east}}));

    types::LidarConfigData lidar_cfg{5.0*cm, 80.0*cm, 2.5*cm, 1};
    DroneControlImpl ctrl(defaultDrone(), defaultMission(), lidar_cfg,
                          my_lidar, my_gps, my_movement, real_map, my_algo);
    std::ignore = ctrl.step();

    // Hit position: drone (50,50,50) + 30cm east = (80,50,50)
    const Position3D hit_pos{80.0*x_extent[cm], 50.0*y_extent[cm], 50.0*z_extent[cm]};
    EXPECT_EQ(real_map.atVoxel(hit_pos), types::VoxelOccupancy::Occupied)
        << "Voxel at scan hit position must be marked Occupied after step()";

    // The path before the hit must be Empty
    const Position3D mid_pos{65.0*x_extent[cm], 50.0*y_extent[cm], 50.0*z_extent[cm]};
    EXPECT_EQ(real_map.atVoxel(mid_pos), types::VoxelOccupancy::Empty)
        << "Voxel along beam before hit must be marked Empty";
}
