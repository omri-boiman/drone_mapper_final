# Drone Mapper — Assignment 2

Full implementation of the drone-mapping simulator for TAU Advanced Topics in Programming 2026B.

## Quick Start

```bash
# Build
cmake --preset default
cmake --build --preset default

# Run simulation (from the inputs/ subdirectory so relative map paths resolve)
cd inputs
../build/drone_mapper_simulation sim_compose.yaml ../output_results/

# Run tests
./build/drone_mapper_simulation_test
```

See **readme.txt** for complete documentation: build instructions, run options,
output file formats, and test filters.

See **bonus.txt** for bonus features implemented.
