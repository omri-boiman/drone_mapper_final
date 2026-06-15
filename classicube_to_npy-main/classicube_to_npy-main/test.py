#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np


def print_layer_xz(arr: np.ndarray, y: int) -> None:
    """
    Print the XZ plane for a fixed Y.
    Assumes array layout is arr[y, z, x].
    """
    _, z_size, x_size = arr.shape

    print(f"\n=== Y = {y} ===")
    print("z\\x " + " ".join(f"{x:3d}" for x in range(x_size)))

    for z in range(z_size):
        row = [f"{int(arr[y, z, x]):3d}" for x in range(x_size)]
        print(f"{z:3d} " + " ".join(row))


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Test a voxel numpy array and print all Y layers as XZ planes."
    )
    parser.add_argument("input", type=Path, help="Path to input .npy file")
    parser.add_argument(
        "--coord",
        nargs=3,
        type=int,
        default=(2, 2, 2),
        metavar=("X", "Y", "Z"),
        help="Coordinate to test, default is 2 2 2",
    )
    args = parser.parse_args()

    arr = np.load(args.input)

    if arr.ndim != 3:
        raise ValueError(f"Expected a 3D array, got shape {arr.shape}")

    y_size, z_size, x_size = arr.shape
    print(f"Loaded array shape: {arr.shape}")
    print("Assumed layout: arr[y, z, x]")

    x, y, z = args.coord

    if not (0 <= x < x_size and 0 <= y < y_size and 0 <= z < z_size):
        raise IndexError(
            f"Coordinate ({x}, {y}, {z}) is out of bounds for shape {arr.shape}"
        )

    value = arr[y, z, x]
    print(f"\nValue at (x={x}, y={y}, z={z}): {int(value)}")

    if int(value) != 0:
        print("Block exists at that coordinate.")
    else:
        print("No block at that coordinate (value is 0).")

    for current_y in range(y_size):
        print_layer_xz(arr, current_y)


if __name__ == "__main__":
    main()