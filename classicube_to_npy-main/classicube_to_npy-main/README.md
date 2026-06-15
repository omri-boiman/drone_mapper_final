# Classicube Mapper Helper

This helper converts a [Classicube](https://www.classicube.net/server/play/) `.cw` world file into a NumPy array that is compatible with the VoxelGrid implementation provided in the example MockLidarSensor implementation.

Use it to check what blocks exist in a saved world, compare the drone output with the expected map, and debug coordinates without opening the world manually.

## Requirements

- Python 3
- `numpy`
- `nbtlib`

Install the Python packages with:

```powershell
pip install numpy nbtlib
```

## Convert a World

Run:

```powershell
python main.py worlds\example.cw
```

This creates:

- `worlds\example.npy` - the block array

You can choose the output path explicitly:

```powershell
python main.py worlds\example.cw -o output.npy
```

## Inspect the Result

Use `test.py` to load a `.npy` file, check one coordinate, and print all height layers:

```powershell
python test.py output.npy --coord 2 2 2
```

Block value `0` means empty air. Any other value means there is a block at that position.

## Notes for the Project

The generated array stores block IDs from the Classicube world. Your Mapping Drone code can use these values to decide whether a position is empty or occupied.
