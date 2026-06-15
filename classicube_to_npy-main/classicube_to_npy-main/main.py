#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path

import nbtlib
import numpy as np


def load_cw(path: Path):
    nbt_file = nbtlib.load(path)

    required = {"X", "Y", "Z", "BlockArray"}
    missing = required - set(nbt_file.keys())
    if missing:
        raise ValueError(
            f"Not a supported ClassicWorld root. Missing keys: {sorted(missing)}. "
            f"Found keys: {list(nbt_file.keys())}"
        )

    x = int(nbt_file["X"])
    y = int(nbt_file["Y"])
    z = int(nbt_file["Z"])

    blocks = np.frombuffer(bytes(nbt_file["BlockArray"]), dtype=np.uint8)
    expected = x * y * z
    if blocks.size != expected:
        raise ValueError(
            f"BlockArray size mismatch: expected {expected}, got {blocks.size}"
        )

    arr = blocks.reshape((x, y, z))

    return arr


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=Path, help="Input .cw file")
    parser.add_argument("-o", "--output", type=Path, help="Output .npy file")
    args = parser.parse_args()

    output = args.output or args.input.with_suffix(".npy")

    arr = load_cw(args.input)

    np.save(output, arr)

    print(f"Saved array to {output}")
    print(f"Shape: {arr.shape}, dtype: {arr.dtype}")


if __name__ == "__main__":
    main()
