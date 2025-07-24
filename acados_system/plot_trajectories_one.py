import matplotlib.pyplot as plt
from matplotlib.ticker import ScalarFormatter
import numpy as np
from scipy.spatial.transform import Rotation as R
import re
from common import getTrack, DRONE_COUNT
import sys

def plot_trajectories(filename="trajectory.txt", w=2.0, d=2.0, h=1.0, l=1.0, block=False, save_path=None):
    # Load data from the solution path file
    data = np.loadtxt(filename)
    if data.ndim == 1:
        # Only one timepoint or empty
        data = data[None, :]  # Make it a single-row 2D array
    if data.size == 0:
        print("Trajectory file is empty.")
        return

    # Determine the number of drones based on the number of columns
    num_drones = DRONE_COUNT  # 13 columns for payload (SE3 + velocity), 11 per drone
    print(f"System contains payload and {num_drones} drones.")

    # Number of states to plot
    to_plot = 100

    # Randomly select states if there are too many
    if data.shape[0] > to_plot:
        indices = np.random.choice(data.shape[0], to_plot, replace=False)
        data = data[indices]

    # Create a scatter plot for the payload and each drone's trajectory
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    all_x, all_y, all_z = [], [], []

    # Plot payload position
    payload_x = data[:, 0]
    payload_y = data[:, 1]
    payload_z = data[:, 2]

    all_x.append(payload_x)
    all_y.append(payload_y)
    all_z.append(payload_z)

    # Define the payload's corners
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

    # Define edges of the box (pairs of vertex indices)
    edges = [
        (0, 1), (1, 2), (2, 3), (3, 0),  # Bottom face
        (4, 5), (5, 6), (6, 7), (7, 4),  # Top face
        (0, 4), (1, 5), (2, 6), (3, 7)   # Vertical edges
    ]

    for i in range(len(payload_x)):

        # Convert quaternion to rotation matrix (q_x, q_y, q_z, q_w)
        if np.linalg.norm([data[i, 3], data[i, 4], data[i, 5], data[i, 6]]) > 1e-8:
            # SciPy wants [qx, qy, qz, qw]
            qw, qx, qy, qz = data[i, 3], data[i, 4], data[i, 5], data[i, 6]
            r = R.from_quat([qx, qy, qz, qw])
            # print(f"Quaternion at point {i}:\n{r.as_quat()}")
            rotation_matrix = r.as_matrix()
            # print(f"Rotation matrix at point {i}:\n{rotation_matrix}")
        else:
            continue

        # Rotate and translate all points
        translated_corners = (np.dot(rotation_matrix, corners.T).T +
                            np.array([payload_x[i], payload_y[i], payload_z[i]]))
        
        # print(f"Translated corners at point {i}:\n{translated_corners}")

        for edge in edges:
            points = translated_corners[list(edge)]
            ax.plot(points[:, 0], points[:, 1], points[:, 2], 'b')

        # Define cable origins (Top Face)
        if num_drones == 1:
            cable_origins = [(rotation_matrix @ np.array([0, 0, h / 2])) + np.array([payload_x[i], payload_y[i], payload_z[i]])]
        elif num_drones == 4:
            cable_origins = translated_corners[4:]

        for j, cable_origin in enumerate(cable_origins):
            # Cable angles (theta, phi)
            theta = data[i, 13 + j * 11 + 7]
            phi = data[i, 13 + j * 11 + 8]

            # Convert spherical coordinates to Cartesian
            cable_x = l * np.sin(theta) * np.cos(phi)
            cable_y = - l * np.cos(theta)
            cable_z = l * np.sin(theta) * np.sin(phi) 

            cable_vector = np.array([cable_x, cable_y, cable_z])
            cable_end = cable_origin + cable_vector
            
            # Plot the cable
            ax.plot([cable_origin[0], cable_end[0]],
                    [cable_origin[1], cable_end[1]],
                    [cable_origin[2], cable_end[2]], color='r', linewidth=1.5)

            # Plot drone at the end of the cable
            drone_color = 'k'
            ax.scatter(cable_end[0], cable_end[1], cable_end[2], color=drone_color, s=20)

            # Define the scale for the drone's square
            drone_scale = 0.4

            # Plot a square for the drone
            drone_square_local = np.array([
                [-drone_scale, -drone_scale, 0],  # Bottom-left
                [drone_scale, -drone_scale, 0],   # Bottom-right
                [drone_scale, drone_scale, 0],    # Top-right
                [-drone_scale, drone_scale, 0],   # Top-left
                [-drone_scale, -drone_scale, 0]   # Close the square
            ]) * l  # Scale the drone's square size based on `l`

            drone_idx = 13 + j * 11

            if np.linalg.norm([data[i, drone_idx], data[i, drone_idx + 1], data[i, drone_idx + 2], data[i, drone_idx + 3]]) > 1e-8:
                # Compute the drone's rotation matrix from quaternion
                # SciPy expects [qx, qy, qz, qw], not [qw, qx, qy, qz]
                qw, qx, qy, qz = (
                    data[i, drone_idx],
                    data[i, drone_idx + 1],
                    data[i, drone_idx + 2],
                    data[i, drone_idx + 3]
                )
                drone_rotation = R.from_quat([qx, qy, qz, qw]).as_matrix()
                
                # Transform the local square to global coordinates
                drone_square_global = (drone_rotation @ drone_square_local.T).T + cable_end
                ax.plot(drone_square_global[:, 0], drone_square_global[:, 1], drone_square_global[:, 2], color='orange', linewidth=1.5)

                # # Compute the thrust direction (normal to the square, in the drone's local z-direction)
                # thrust_dir = drone_rotation @ np.array([0, 0, 1])  # Global thrust direction

                # # Plot an arrow representing the thrust direction
                # ax.quiver(cable_end[0], cable_end[1], cable_end[2],  # Arrow start position
                #         thrust_dir[0], thrust_dir[1], thrust_dir[2],  # Arrow direction
                #         color='green', length=0.3, normalize=True, linewidth=2)
            else:
                continue

            if i == 0:  # Add legend entry only for the first point
                ax.scatter([], [], color=drone_color, s=0, label="Drone")

    # ── overlay 700 random points from helix_track.txt ────────────────
    x_t, y_t, z_t = getTrack()
    idx = np.random.choice(len(x_t), min(700, len(x_t)), replace=False)
    ax.scatter(x_t[idx], y_t[idx], z_t[idx],
               c="gray", alpha=0.6, s=20, label="Track samples")
    # ─────────────────────────────────────────────────────────────────────

    # Flatten simulation lists to calculate global ranges for uniform scaling
    all_x = np.concatenate(all_x)
    all_y = np.concatenate(all_y)
    all_z = np.concatenate(all_z)

    # # **Include track points in the extents so we don't zoom past them**
    # all_x = np.concatenate([all_x, x_t])
    # all_y = np.concatenate([all_y, y_t])
    # all_z = np.concatenate([all_z, z_t])

    # Compute the overall limits with a little padding
    padding = 2  # Adjust for more/less
    x_min, x_max = all_x.min() - padding, all_x.max() + padding
    y_min, y_max = all_y.min() - padding, all_y.max() + padding
    z_min, z_max = all_z.min() - padding, all_z.max() + padding

    # Now set equal‐aspect, centered limits
    max_range = max(x_max - x_min, y_max - y_min, z_max - z_min) / 2.0
    mid_x = (x_max + x_min) / 2.0
    mid_y = (y_max + y_min) / 2.0
    mid_z = (z_max + z_min) / 2.0

    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)

    # Labels, legend, tick formatting as before
    ax.set_xlabel("X Position")
    ax.set_ylabel("Y Position")
    ax.set_zlabel("Z Position")
    # ax.legend()
    ax.xaxis.set_major_formatter(ScalarFormatter(useOffset=False))
    ax.yaxis.set_major_formatter(ScalarFormatter(useOffset=False))
    ax.zaxis.set_major_formatter(ScalarFormatter(useOffset=False))

    # Full‐screen and show
    manager = plt.get_current_fig_manager()
    manager.window.showMaximized()

    # --- Parameter annotation on plot ---
    def extract_params_from_filename(filename):
        if filename is None:
            return {}
        base = filename.split('/')[-1].replace('.txt', '')
        base = re.sub(r'^trajectory_\d+_', '', base)
        if not base.startswith('_'):
            base = '_' + base
        pattern = re.compile(r'_([a-zA-Z0-9_]+?)(-?\d+(?:\.\d+)?)')
        matches = pattern.findall(base)
        return {k: v for k, v in matches}

    params = extract_params_from_filename(filename)
    params_str = (
        f"N = {params.get('N', '?')}\n"
        f"velocity_ref = {params.get('velocity_ref', '?')}\n"
        "\n"
        "State weights:\n"
        f"  payload_pos_w    = {params.get('payload_pos_w', '?')}\n"
        f"  payload_vel_w    = {params.get('payload_vel_w', '?')}\n"
        f"  payload_quat_w   = {params.get('payload_quat_w', '?')}\n"
        f"  drone_quat_w     = {params.get('drone_quat_w', '?')}\n"
        f"  payload_angvel_w = {params.get('payload_angvel_w', '?')}\n"
        f"  drone_angvel_w   = {params.get('drone_angvel_w', '?')}\n"
        f"  cable_angles_w   = {params.get('cable_angles_w', '?')}\n"
        "\n"
        "Input weights:\n"
        f"  r_T   = {params.get('r_T', '?')}\n"
        f"  r_tau = {params.get('r_tau', '?')}\n"
        "\n"
        "Final time weight:\n"
        f"  W_Tf  = {params.get('W_Tf', '?')}"
    )

    plt.gcf().text(
        0.01, 0.99, params_str,
        ha='left', va='top', fontsize=13,
        bbox=dict(facecolor='white', alpha=0.8, edgecolor='gray')
    )

    if save_path:
        plt.savefig(save_path)
        plt.close()  # Prevents figure from popping up when batch running
    else:
        plt.show()
    
if __name__ == "__main__":
    # Usage: python3 plot_trajectories_one.py [trajectory.txt] [save_path.png]
    filename = None
    save_path = None
    block = False

    if len(sys.argv) > 1:
        filename = sys.argv[1]
        if len(sys.argv) > 2:
            save_path = sys.argv[2]
    else:
        # If no args, plot default file and show
        filename = "trajectory.txt"
        block = True

    plot_trajectories(filename=filename, save_path=save_path, block=block)