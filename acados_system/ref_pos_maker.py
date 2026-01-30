import numpy as np

# Parameters
start_pos = np.array([-10.0, -40.0, 20.0])
posMaxDelta = 3.0
pos_ref_size = 50
num_segments = 1  # Set to >1 if you want a multi-segment reference

# For a single trajectory segment:
with open("pos_ref.txt", "w") as f:
    prev_pos = start_pos.copy()
    for segment in range(num_segments):
        # Sample a random delta in [-posMaxDelta, posMaxDelta] for each axis
        delta = np.random.uniform(-posMaxDelta, posMaxDelta, 3)
        next_pos = prev_pos + delta

        pos_traj = np.zeros((pos_ref_size, 3))
        for j in range(pos_ref_size):
            alpha = j / (pos_ref_size - 1)
            pos_traj[j, :] = prev_pos * (1.0 - alpha) + next_pos * alpha

        # Write all points to file
        for row in pos_traj:
            f.write(f"{row[0]:.6f} {row[1]:.6f} {row[2]:.6f}\n")

        # For multi-segment, update prev_pos:
        prev_pos = next_pos
