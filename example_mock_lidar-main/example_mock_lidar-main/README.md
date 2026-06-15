# Mock LiDAR Sensor

An implementation for a mock LiDAR sensor scanning a 3D voxel occupancy map. The code is intended as a starting point for your Mapping Drone project in the 2026 "Advanced Topics in Programming" course.

Note: If you wish to use this implementation, you are responsible for adapting it to your project.

The attached container includes all the requirements you will need to build and run this project.

## What Is Included

This project includes:

- A mock LiDAR sensor implementation.
- Interfaces for a 3D map, LiDAR sensor, and position sensor.
- A voxel-grid map loader for `.npy` files.
- Strongly typed position, distance, and angle units.
- Example maps under `data_maps/`.
- A small runnable example.
- Unit tests using GoogleTest.

Note - The implementation for PositionSensor is ad-hoc. In your project these will be different!
## Project Structure

```text
include/cpp_course/      Public interfaces and type definitions
src/                     Implementation files
examples/                Example executable
tests/                   Unit tests
data_maps/               Example voxel maps
.devcontainer/           Development container setup
CMakeLists.txt           CMake build configuration
CMakePresets.json        CMake preset configuration
vcpkg.json               Dependency list
```

## Building the Project

Open the project inside the provided container, then run:

```bash
cmake --preset default
cmake --build --preset default
```

This creates the build output under the `build/` directory.

## Running the Example

After building, run:

```bash
./build/mock_lidar_test
```

You can also run the example with a specific map:

```bash
./build/mock_lidar_test data_maps/five_voxels_y4_pattern.npy
```

## Running the Tests

Run all tests with:

```bash
ctest --test-dir build --output-on-failure
```

Or run the test executable directly:

```bash
./build/cpp_course_tests
```

## Main Components

`IMap3D` is the interface used by the LiDAR sensor to query whether a position in space is occupied.

`VoxelGrid` is an implementation of `IMap3D` that loads a 3D NumPy `.npy` file. The map is interpreted as a voxel grid, where each voxel represents 1 cm.

`IPositionSensor` provides the current position and heading of the drone.

`MockLidarSensor` scans the map by tracing beams from the drone position. It returns the distance and relative angle for each hit.

`Units.h` defines the physical units used by the project, including X/Y/Z lengths and horizontal/altitude angles.

## Map Format

The sample maps are stored in `data_maps/` as `.npy` files.

Each map is expected to be a row-major 3D array with shape `[X, Y, Z]`. A value of `0` means empty space. A non-zero value means occupied space.

Coordinates are interpreted in centimeters. For example, the position `(2 cm, 4 cm, 2 cm)` maps to voxel index `(2, 4, 2)`.

Positions outside the map are treated as empty space.

## Adapting This Code

This code is intentionally written as a starting point, not as a complete Mapping Drone solution.

Before using it in your project, consider:

- Whether the map format matches your project.
- Whether the LiDAR scan pattern matches your needs.
- How your drone should convert relative LiDAR hits into world-space coordinates.
- Whether the sensor interfaces should be extended.
- What additional tests are required for your final implementation.

Generated files such as `build/` and `vcpkg_installed/` should not be committed to your repository.
