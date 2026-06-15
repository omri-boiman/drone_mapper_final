//
// sensor_test — smoke test for Phase 5 mocks and BuildingMapImpl.
//
// Scenario:
//   Ground-truth wall: Occupied cells at x=300, y=80..120, z=100..200 cm.
//   Drone starts at (100, 100, 150) cm, orientation 0/0 deg (+X direction).
//   No GTest — prints PASS / FAIL for each check.
//

#include <iostream>
#include <memory>
#include <cmath>
#include <string>

#include "simulation/SimulationState.h"
#include "simulation/MockPositionSensor.h"
#include "simulation/MockMovementDriver.h"
#include "simulation/MockLidarSensor.h"
#include "simulation/CellMap.h"
#include "drone/BuildingMapImpl.h"
#include "drone/Drone.h"
#include "io/MapIO.h"
#include "io/ConfigParser.h"

using namespace drone;

static int g_pass = 0;
static int g_fail = 0;

static void Check(const std::string& label, bool condition)
{
    if (condition) { std::cout << "  PASS  " << label << "\n"; ++g_pass; }
    else           { std::cout << "  FAIL  " << label << "\n"; ++g_fail; }
}

static DroneConfig MakeTestDroneConfig()
{
    DroneConfig c;
    c.lidarBeamMin       = 20.0  * cm;
    c.lidarBeamMax       = 500.0 * cm;
    c.lidarCircleSpacing = 10.0  * cm;
    c.lidarFovCircles    = 3;
    c.maxRotate          = 45.0  * deg;
    c.maxAdvance         = 100.0 * cm;
    c.maxElevate         = 50.0  * cm;
    return c;
}

// Build a LidarConfig from a DroneConfig
static LidarConfig MakeLidarConfig(const DroneConfig& dc)
{
    return {dc.lidarBeamMin, dc.lidarBeamMax, dc.lidarCircleSpacing, dc.lidarFovCircles};
}

// Build a ParsedMap with a wall: Occupied cells at x=300, y=80..120, z=100..200 cm
static ParsedMap MakeWallMap()
{
    ParsedMap pm;
    pm.valid = true;
    pm.bounds = { 0*cm, 500*cm, 0*cm, 500*cm, 0*cm, 300*cm };

    for (int y = 80; y <= 120; ++y)
        for (int z = 100; z <= 200; ++z)
            pm.cells.push_back({
                300.0 * cm,
                static_cast<double>(y) * cm,
                static_cast<double>(z) * cm,
                MapValue::Occupied
            });
    return pm;
}

// ---------------------------------------------------------------------------

static void TestPositionSensor()
{
    std::cout << "\n-- MockPositionSensor --\n";

    auto state = std::make_shared<SimulationState>();
    state->position.x = 100.0 * x_extent[cm];
    state->position.y = 200.0 * y_extent[cm];
    state->position.z = 150.0 * z_extent[cm];

    MockPositionSensor sensor(state);
    const Position3D pos = sensor.position();

    Check("x == 100 cm", pos.x.numerical_value_in(cm) == 100.0);
    Check("y == 200 cm", pos.y.numerical_value_in(cm) == 200.0);
    Check("z == 150 cm", pos.z.numerical_value_in(cm) == 150.0);

    state->position.x = 999.0 * x_extent[cm];
    Check("x tracks state mutation",
          sensor.position().x.numerical_value_in(cm) == 999.0);
}

static void TestMovementDriver()
{
    std::cout << "\n-- MockMovementDriver --\n";

    const DroneConfig config = MakeTestDroneConfig();
    const ParsedMap   wmap   = MakeWallMap();
    const CellMap     cellMap(wmap);

    auto state = std::make_shared<SimulationState>();
    state->position.x            = 100.0 * x_extent[cm];
    state->position.y            = 100.0 * y_extent[cm];
    state->position.z            = 150.0 * z_extent[cm];
    state->orientation.horizontal = 0.0 * horizontal_angle[deg];
    state->orientation.altitude   = 0.0 * altitude_angle[deg];

    MockMovementDriver driver(state, config, cellMap);

    // Rotate 60 deg, max is 45 — should clamp to 45
    const MoveResult rotResult = driver.Rotate(60.0 * horizontal_angle[deg]);
    Check("Rotate returns Success", rotResult == MoveResult::Success);
    Check("Rotate 60 deg clamped to 45 deg",
          std::abs(state->orientation.horizontal.numerical_value_in(deg) - 45.0) < 0.001);

    state->orientation.horizontal = 0.0 * horizontal_angle[deg];

    // Advance 100 cm into free space (wall is 200 cm away)
    const MoveResult advOk = driver.Advance(100.0 * cm);
    Check("Advance into free space returns Success", advOk == MoveResult::Success);
    Check("Position updated after free advance",
          std::abs(state->position.x.numerical_value_in(cm) - 200.0) < 0.5);

    // Advance another 100 cm: hits wall at x=300
    const MoveResult advCollide = driver.Advance(100.0 * cm);
    Check("Advance into wall returns CollisionDetected",
          advCollide == MoveResult::CollisionDetected);
    Check("Position NOT updated on collision",
          std::abs(state->position.x.numerical_value_in(cm) - 200.0) < 0.5);

    // Elevate into free space
    const double startZ = state->position.z.numerical_value_in(cm);
    const MoveResult elevOk = driver.Elevate(30.0 * cm);
    Check("Elevate into free space returns Success", elevOk == MoveResult::Success);
    Check("z updated after elevate",
          std::abs(state->position.z.numerical_value_in(cm) - (startZ + 30.0)) < 0.5);
}

static void TestLidarSensor()
{
    std::cout << "\n-- MockLidarSensor --\n";

    const DroneConfig config  = MakeTestDroneConfig();
    const ParsedMap   wmap    = MakeWallMap();
    const CellMap     cellMap(wmap);

    auto state = std::make_shared<SimulationState>();
    state->position.x            = 100.0 * x_extent[cm];
    state->position.y            = 100.0 * y_extent[cm];
    state->position.z            = 150.0 * z_extent[cm];
    state->orientation.horizontal = 0.0 * horizontal_angle[deg];
    state->orientation.altitude   = 0.0 * altitude_angle[deg];

    MockPositionSensor posSensor(state);
    MockLidarSensor    lidar(MakeLidarConfig(config), cellMap, posSensor);

    // Scan with orientation {0,0} — beam_0 points in +X, wall at x=300, distance ≈ 200 cm
    const ScanResults results = lidar.scan(Orientation{});
    Check("Scan returns at least one hit", !results.empty());

    bool foundHit = false;
    for (const auto& hit : results) {
        const double d = hit.distance.numerical_value_in(cm);
        if (d >= 190.0 && d <= 210.0) { foundHit = true; break; }
    }
    Check("At least one hit in range 190–210 cm (wall ≈ 200 cm away)", foundHit);
}

static void TestBuildingMap()
{
    std::cout << "\n-- BuildingMapImpl --\n";

    MissionConfig mc;
    mc.boundaryPolygon = { {0,0}, {500,0}, {500,500}, {0,500} };
    mc.minHeight       = ZLength{0.0   * cm};
    mc.maxHeight       = ZLength{300.0 * cm};
    mc.outputResXYCm   = 1.0;
    mc.outputResHCm    = 1.0;

    BuildingMapImpl map(mc);

    Check("Unset cell returns NotMapped",
          map.Get(100.0 * x_extent[cm],
                  100.0 * y_extent[cm],
                  150.0 * z_extent[cm]) == MapValue::NotMapped);

    Check("Out-of-bounds cell returns BeyondBounds",
          map.Get(600.0 * x_extent[cm],
                  100.0 * y_extent[cm],
                  150.0 * z_extent[cm]) == MapValue::BeyondBounds);

    map.Set(100.0 * x_extent[cm],
            100.0 * y_extent[cm],
            150.0 * z_extent[cm], MapValue::Occupied);
    Check("Set then Get returns Occupied",
          map.Get(100.0 * x_extent[cm],
                  100.0 * y_extent[cm],
                  150.0 * z_extent[cm]) == MapValue::Occupied);

    map.Set(600.0 * x_extent[cm],
            100.0 * y_extent[cm],
            150.0 * z_extent[cm], MapValue::Occupied);
    Check("Set out-of-bounds is silently ignored",
          map.Get(600.0 * x_extent[cm],
                  100.0 * y_extent[cm],
                  150.0 * z_extent[cm]) == MapValue::BeyondBounds);

    // 100.5 and 100.0 share the same 1 cm voxel (floor(100.5) == floor(100.0) == 100)
    map.Set(100.5 * x_extent[cm],
            100.0 * y_extent[cm],
            150.0 * z_extent[cm], MapValue::Empty);
    Check("100.5 cm shares voxel with 100.0 cm (floor-based 1 cm cells)",
          map.Get(100.0 * x_extent[cm],
                  100.0 * y_extent[cm],
                  150.0 * z_extent[cm]) == MapValue::Empty);
}

static void TestDrone()
{
    std::cout << "\n-- Drone (hardware abstraction layer) --\n";

    const DroneConfig config  = MakeTestDroneConfig();
    const ParsedMap   wmap    = MakeWallMap();
    const CellMap     cellMap(wmap);

    auto state = std::make_shared<SimulationState>();
    state->position.x            = 100.0 * x_extent[cm];
    state->position.y            = 100.0 * y_extent[cm];
    state->position.z            = 150.0 * z_extent[cm];
    state->orientation.horizontal = 0.0 * horizontal_angle[deg];
    state->orientation.altitude   = 0.0 * altitude_angle[deg];

    MockPositionSensor posSensor(state);
    MockMovementDriver driver(state, config, cellMap);
    MockLidarSensor    lidar(MakeLidarConfig(config), cellMap, posSensor);

    MissionConfig mc;
    mc.boundaryPolygon = { {0,0}, {500,0}, {500,500}, {0,500} };
    mc.minHeight       = ZLength{0.0   * cm};
    mc.maxHeight       = ZLength{300.0 * cm};
    mc.outputResXYCm   = 1.0;
    mc.outputResHCm    = 1.0;
    BuildingMapImpl buildingMap(mc);

    Drone drone(lidar, posSensor, driver, buildingMap);

    // GetLocation
    const Position3D pos = drone.GetLocation();
    Check("GetLocation x == 100 cm", pos.x.numerical_value_in(cm) == 100.0);
    Check("GetLocation y == 100 cm", pos.y.numerical_value_in(cm) == 100.0);
    Check("GetLocation z == 150 cm", pos.z.numerical_value_in(cm) == 150.0);

    // Rotate (clamped to 45)
    const MoveResult rotResult = drone.Rotate(60.0 * horizontal_angle[deg]);
    Check("Rotate returns Success", rotResult == MoveResult::Success);
    Check("Rotate 60 deg clamped to 45 deg",
          std::abs(state->orientation.horizontal.numerical_value_in(deg) - 45.0) < 0.001);
    state->orientation.horizontal = 0.0 * horizontal_angle[deg];

    // Advance into free space
    const MoveResult advOk = drone.Advance(100.0 * cm);
    Check("Advance into free space returns Success", advOk == MoveResult::Success);
    Check("Position updated after Advance",
          std::abs(drone.GetLocation().x.numerical_value_in(cm) - 200.0) < 0.5);

    // Advance into wall
    const MoveResult advCollide = drone.Advance(100.0 * cm);
    Check("Advance into wall returns CollisionDetected",
          advCollide == MoveResult::CollisionDetected);
    Check("Position NOT updated on collision",
          std::abs(drone.GetLocation().x.numerical_value_in(cm) - 200.0) < 0.5);

    // Elevate
    const double startZ = drone.GetLocation().z.numerical_value_in(cm);
    const MoveResult elevOk = drone.Elevate(30.0 * cm);
    Check("Elevate returns Success", elevOk == MoveResult::Success);
    Check("z updated after Elevate",
          std::abs(drone.GetLocation().z.numerical_value_in(cm) - (startZ + 30.0)) < 0.5);

    // Scan returns hits
    const ScanResults scan = drone.Scan();
    Check("Scan returns at least one hit", !scan.empty());

    // RecordCell / QueryCell round-trip
    drone.RecordCell(200.0 * x_extent[cm],
                     100.0 * y_extent[cm],
                     150.0 * z_extent[cm], MapValue::Occupied);
    Check("RecordCell then QueryCell returns Occupied",
          drone.QueryCell(200.0 * x_extent[cm],
                          100.0 * y_extent[cm],
                          150.0 * z_extent[cm]) == MapValue::Occupied);

    Check("QueryCell on unset location returns NotMapped",
          drone.QueryCell(1.0 * x_extent[cm],
                          1.0 * y_extent[cm],
                          1.0 * z_extent[cm]) == MapValue::NotMapped);
}

int main()
{
    std::cout << "=== Sensor & Map smoke tests ===\n";

    TestPositionSensor();
    TestMovementDriver();
    TestLidarSensor();
    TestBuildingMap();
    TestDrone();

    std::cout << "\n================================\n";
    std::cout << "  Passed: " << g_pass << "\n";
    std::cout << "  Failed: " << g_fail << "\n";
    std::cout << "================================\n";

    return (g_fail == 0) ? 0 : 1;
}
