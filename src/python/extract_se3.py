#!/usr/bin/env python3
import numpy as np
from scipy.spatial.transform import Rotation as R
import sys

# Parameters (adjust as needed)
w, d, h = 2.0, 2.0, 1.0   # payload dimensions
l = 1.0                   # cable length

def compute_se3_line(state):
    # Payload SE(3) (position: indices 0-2, quaternion: indices 3-6)
    payload_pos = state[0:3]
    payload_quat = state[3:7]
    
    # Compute payload's rotated corners (box)
    corners = np.array([
        [-w/2, -d/2, -h/2],
        [ w/2, -d/2, -h/2],
        [ w/2,  d/2, -h/2],
        [-w/2,  d/2, -h/2],
        [-w/2, -d/2,  h/2],
        [ w/2, -d/2,  h/2],
        [ w/2,  d/2,  h/2],
        [-w/2,  d/2,  h/2]
    ])
    R_payload = R.from_quat(payload_quat).as_matrix()
    translated_corners = (R_payload @ corners.T).T + payload_pos
    # Cable origins are on the top face (indices 4-7)
    cable_origins = translated_corners[4:8]

    # Number of drones = (total state length - 13) // 11
    num_drones = 4
    drone_se3 = []
    for j in range(num_drones):
        base = 13 + j * 11  # start index for drone j
        drone_quat = state[base : base + 4]
        # Cable angles (theta, phi) at indices base+7 and base+8
        theta = state[base + 7]
        phi = state[base + 8]
        # Compute cable (and thus drone) position in Cartesian coordinates
        cable_vector = np.array([
            l * np.sin(theta) * np.cos(phi),
            l * np.sin(theta) * np.sin(phi),
            l * np.cos(theta)
        ])
        # Attach drone to corresponding payload top-corner (wrap if needed)
        origin = cable_origins[j % len(cable_origins)]
        drone_pos = origin + cable_vector
        drone_se3.append(np.concatenate([drone_pos, drone_quat]))
    
    # Concatenate payload SE(3) and all drone SE(3)
    return np.concatenate([payload_pos, payload_quat] + drone_se3)

def process_file(input_file, output_file):
    data = np.loadtxt(input_file)
    output_data = np.array([compute_se3_line(line) for line in data])
    np.savetxt(output_file, output_data, fmt="%.6f")

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python extract_se3.py <input_file> <output_file>")
        sys.exit(1)
    process_file(sys.argv[1], sys.argv[2])
