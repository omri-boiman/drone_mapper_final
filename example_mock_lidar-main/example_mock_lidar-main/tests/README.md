# Tests

This folder contains the unit tests for the mock LiDAR project. The tests are written with GoogleTest and are built as part of the `cpp_course_tests` executable.

## Running the Tests

From the project root, inside the provided container, run:

```bash
cmake --preset default
cmake --build --preset default
ctest --test-dir build --output-on-failure
```

If the project is already built, you can run only:

```bash
ctest --test-dir build --output-on-failure
```

## Test Files

```text
MockLidarSensorTests.cpp   Tests for the mock LiDAR behavior
VoxelGridTests.cpp         Tests for loading and querying voxel maps
TestHelpers.h              Shared helper functions for constructing positions,
                           orientations, and reconstructed hit points
```

## MockLidarSensorTests.cpp

These tests use small fake implementations instead of loading real map files. This keeps the tests focused on the LiDAR sensor logic.

The test doubles are:

- `EmptyMap`: always returns empty space.
- `AlwaysOccupiedMap`: always returns occupied space.
- `SingleVoxelMap`: returns occupied space only at one requested voxel.
- `FixedPositionSensor`: returns a fixed drone position and heading.

The tests check that `MockLidarSensor`:

- Stores and exposes its configuration.
- Returns no results when no field-of-view circles are configured.
- Returns the expected number of hits when every beam hits.
- Returns no hits when the map is empty.
- Ignores occupied voxels beyond `beam_length_max`.
- Returns `0 cm` when an obstacle is closer than `beam_length_min`.
- Reports the expected distance for a center-beam hit.
- Uses the requested scan orientation.
- Uses the sensor heading internally for tracing.
- Returns beam angles relative to the scan request, not absolute world angles.
- Supports altitude scans.
- Produces hits that can be reconstructed back into occupied voxels.

## VoxelGridTests.cpp

These tests load the sample `.npy` maps from `data_maps/` and check that `VoxelGrid`:

- Loads a 3D map and reports the expected shape.
- Returns occupied values for known occupied voxels.
- Returns `0` for empty voxels.
- Returns `0` for negative or out-of-bounds coordinates.
- Supports both `int` and `uint8_t` map values.

The tests use `CPP_COURSE_SOURCE_DIR`, which is defined in `CMakeLists.txt`, to locate the sample map files regardless of the current working directory.

## TestHelpers.h

`TestHelpers.h` contains shared helper functions used by the tests:

- `make_position(...)`
- `make_orientation(...)`
- `absolute_beam(...)`
- `hit_to_position(...)`
- `voxel_coordinates(...)`
- `degrees(...)`
- `wrap_degrees(...)`

These helpers make the tests easier to read and keep unit conversions in one place.
