TAU - Advanced Topics in Programming, Semester B 2026
Assignment 1 - Drone Mapper
=============================================================

Contributors
------------
Name: Amit Halfon    ID: 211781141
Name: Omri Boiman  ID: 325049575


Building & Running
------------------
The project uses a VS Code Dev Container. All tools (GCC 13, CMake, Conan 2.x)
and the mp-units library are installed automatically inside the container.

Step 1 — Open in Dev Container
  - Install VS Code and the "Dev Containers" extension
      (ms-vscode-remote.remote-containers)
  - Open this folder (drone_mapper/) in VS Code
  - When prompted "Reopen in Container", click it
      (or: F1 -> "Dev Containers: Reopen in Container")
  - Wait for the container to build and for the postCreateCommand to finish
      (this sets up the Conan compiler profile automatically)

Step 2 — Build
  Inside the container terminal:
    make rebuild

  This installs mp-units via Conan, configures CMake, and compiles.
  The binary is placed at: build/drone_mapper

Step 3 — Run
  ./build/drone_mapper <path-to-scenario-folder>

  Examples:
    ./build/drone_mapper ./scenario1
    ./build/drone_mapper ./scenario2
    ./build/drone_mapper ./scenario3

  If no path is given, the current working directory is used.
  Output is written to map_output.txt in the scenario folder.


Input File Formats
------------------
All three input files must reside in the same directory.

1. drone_config.txt
   Key = value pairs (one per line). Supported keys:
     min_pass_width_cm        (default 60)
     min_pass_length_cm       (default 60)
     min_pass_height_cm       (default 120)
     lidar_beam_min_cm        (default 20)
     lidar_beam_max_cm        (default 1000)
     lidar_circle_spacing_cm  (default 10)
     lidar_fov_circles        (default 3)
     max_rotate_deg           (default 45)
     max_advance_cm           (default 50)
     max_elevate_cm           (default 30)

2. mission_config.txt
   Key = value pairs. Supported keys:
     boundary_polygon         comma-separated (x,y) pairs e.g. (0,0),(500,0),(500,500),(0,500)
     min_height_cm            (default 0)
     max_height_cm            (default 300)
     output_resolution_xy_cm  cell size in XY, cm (default 1.0)
     output_resolution_h_cm   cell size in Height, cm (default 1.0)
     start_x_cm               drone start X (default 0)
     start_y_cm               drone start Y (default 0)
     start_height_cm          drone start height (default 150)

3. map_input.txt
   Ground-truth building map (used only by the lidar mock sensor).
   Format:
     VERSION 1
     BOUNDS xmin=<v> xmax=<v> ymin=<v> ymax=<v> hmin=<v> hmax=<v>
     CELL <x_cm> <y_cm> <h_cm> <value>
     ...
     END
   value: 1 = occupied. All coordinates and dimensions in cm (floats).


Output File Format
------------------
map_output.txt — written to the same directory as the input files.
Same CELL format as map_input.txt. Contains only the cells the drone
mapped as Occupied (value=1). Cells not visited remain NotMapped (-1)
and are omitted from the file.

If any input file has recoverable errors, a short description is written
to input_errors.txt in the same directory.


Scoring
-------
F1 score (0-100) printed to stdout after each run:

  F1 = 2 * TP / (2*TP + FP + FN) * 100

  TP = cells mapped as Occupied that are in ground truth
  FP = cells mapped as Occupied that are NOT in ground truth
  FN = ground truth occupied cells NOT mapped by the drone

A score of 100 means a perfect match. Scores below 100 occur when parts
of the building are physically unreachable or hidden behind solid walls.


Test Scenarios
--------------
scenario1/  24x24x16 cm room with a 4x4x10 solid pillar in the centre.
            Interior voxels of the pillar are unreachable by lidar.
            Expected score: 98.4 / 100

scenario2/  20x20x20 cm hollow room, no internal obstacles.
            All surfaces are reachable.
            Expected score: 100.0 / 100

scenario3/  5x5x5 cm space with a double-thickness wall.
            Cells hidden behind the solid wall cannot be detected.
            Expected score: 77.8 / 100

Reference outputs are stored in each scenario's original_output/ folder.


mp-units Library
----------------
We use the open-source mp-units library (https://mpusz.github.io/mp-units/)
for strong physical types in C++20.

Rationale:
  The assignment requires that all distance and angle values use strong
  types to prevent unit confusion bugs (e.g. passing centimetres where
  degrees are expected). mp-units provides compile-time dimensional
  analysis with zero runtime overhead.

  Key types used:
    PhysicalLength  = mp::quantity<isq::length[cm], double>
    XLength         = mp::quantity<x_extent[cm],    double>   // X axis
    YLength         = mp::quantity<y_extent[cm],    double>   // Y axis
    ZLength         = mp::quantity<z_extent[cm],    double>   // Height
    HorizontalAngle = mp::quantity<horizontal_angle[deg], double>
    Altitude        = mp::quantity<altitude_angle[deg],   double>

  Trigonometric functions (si::cos, si::sin, si::atan2) operate directly
  on angle quantities, eliminating manual degree-to-radian conversions
  and the precision errors they introduce.

  The library is fetched automatically via Conan 2.x during build.
  No manual installation is required.
