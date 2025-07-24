#!/usr/bin/env python3
import os
import sys
import numpy as np

def main():
    if len(sys.argv) != 2:
        print("Usage: scale.py <input_track.txt>")
        sys.exit(1)
    infile = sys.argv[1]
    base   = os.path.basename(infile)
    outfile = "scaled_" + base

    # Read & preserve header
    with open(infile, 'r') as f:
        header = f.readline()

    # Load data (skip header)
    data = np.loadtxt(infile, skiprows=1)   # shape (M,4): [s, x, y, z]

    # Scale x,y,z independently (leave the original s column alone for now)
    # Adjust these scale factors to your needs:
    scales = np.array([1.0, 20.0, 20.0, 100.0])
    data[:, :len(scales)] *= scales

    # Recompute column 0 as true arc length along the scaled 3D curve:
    xyz = data[:, 1:4]      # shape (M,3)
    # compute pairwise distances
    diffs = xyz[1:] - xyz[:-1]       # (M-1,3)
    dists = np.linalg.norm(diffs, axis=1)  # (M-1,)
    s = np.zeros(data.shape[0], dtype=float)
    s[1:] = np.cumsum(dists)
    data[:, 0] = s

    # Write out scaled track
    with open(outfile, 'w') as f:
        f.write(header)
        fmt = ["%.6e"] * data.shape[1]
        np.savetxt(f, data, fmt=fmt)

    print(f"Wrote scaled & re-arclengthed track to {outfile}")

if __name__ == "__main__":
    main()
