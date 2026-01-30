import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import ScalarFormatter
from matplotlib.animation import FuncAnimation
from scipy.spatial.transform import Rotation as R
from common import getTrack, DRONE_COUNT

def animate_trajectories(filename="trajectory.txt",
                         w=2.0, d=2.0, h=1.0, l=1.0, interval=1, save_path=None):
    # --- Load data
    data = np.loadtxt(filename)
    num_drones = DRONE_COUNT

    # --- Track data (for consistent axis limits)
    x_t, y_t, z_t = getTrack()

    # --- Get limits from all positions and track
    payload_x = data[:, 0]
    payload_y = data[:, 1]
    payload_z = data[:, 2]
    all_x = np.concatenate([payload_x, x_t])
    all_y = np.concatenate([payload_y, y_t])
    all_z = np.concatenate([payload_z, z_t])
    padding = 2
    x_min, x_max = all_x.min() - padding, all_x.max() + padding
    y_min, y_max = all_y.min() - padding, all_y.max() + padding
    z_min, z_max = all_z.min() - padding, all_z.max() + padding
    max_range = max(x_max - x_min, y_max - y_min, z_max - z_min) / 2.0
    mid_x = (x_max + x_min) / 2.0
    mid_y = (y_max + y_min) / 2.0
    mid_z = (z_max + z_min) / 2.0

    # --- Box corners and edges (fixed)
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
    edges = [
        (0, 1), (1, 2), (2, 3), (3, 0),  # Bottom face
        (4, 5), (5, 6), (6, 7), (7, 4),  # Top face
        (0, 4), (1, 5), (2, 6), (3, 7)   # Vertical edges
    ]

    # --- Figure setup
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel("X Position")
    ax.set_ylabel("Y Position")
    ax.set_zlabel("Z Position")
    ax.xaxis.set_major_formatter(ScalarFormatter(useOffset=False))
    ax.yaxis.set_major_formatter(ScalarFormatter(useOffset=False))
    ax.zaxis.set_major_formatter(ScalarFormatter(useOffset=False))
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)
    ax.scatter(x_t, y_t, z_t, c="gray", alpha=0.6, s=20, label="Track samples")

    # --- Initialize plot elements (lists for box, cables, drones, thrust)
    box_lines = [ax.plot([], [], [], 'b')[0] for _ in edges]
    cable_lines = [ax.plot([], [], [], 'r', linewidth=1.5)[0] for _ in range(num_drones)]
    drone_scatters = [ax.scatter([], [], [], color='k', s=20) for _ in range(num_drones)]
    drone_squares = [ax.plot([], [], [], color='orange', linewidth=1.5)[0] for _ in range(num_drones)]
    thrust_arrows = [ax.quiver(0,0,0,0,0,0, color='green', length=0.3, normalize=True, linewidth=2) for _ in range(num_drones)]

    def update(frame):
        i = frame
        # --- Clear dynamic elements
        for line in box_lines + cable_lines + drone_squares:
            line.set_data_3d([], [], [])
        for sc in drone_scatters:
            sc._offsets3d = ([], [], [])
        for th in thrust_arrows:
            th.remove()
        thrust_arrows.clear()

        # --- Payload pose
        px, py, pz = data[i, 0:3]
        qw, qx, qy, qz = data[i, 3:7]
        r = R.from_quat([qx, qy, qz, qw])
        rotation_matrix = r.as_matrix()
        translated_corners = (rotation_matrix @ corners.T).T + np.array([px, py, pz])

        # --- Draw box
        for edge_idx, edge in enumerate(edges):
            points = translated_corners[list(edge)]
            box_lines[edge_idx].set_data_3d(points[:, 0], points[:, 1], points[:, 2])

        # --- Cable origins (Top face of box)
        if num_drones == 1:
            cable_origins = [(rotation_matrix @ np.array([0, 0, h / 2])) + np.array([px, py, pz])]
        elif num_drones == 4:
            cable_origins = translated_corners[4:]

        # --- Drones, cables, drone squares, thrust arrows
        for j in range(num_drones):
            cable_origin = cable_origins[j]
            # Angles
            theta = data[i, 13 + j * 11 + 7]
            phi   = data[i, 13 + j * 11 + 8]
            # Convert spherical coordinates to Cartesian
            cable_x = l * np.sin(theta) * np.cos(phi)
            cable_y = - l * np.cos(theta)
            cable_z = l * np.sin(theta) * np.sin(phi) 
            cable_vector = np.array([cable_x, cable_y, cable_z])
            cable_end = cable_origin + cable_vector

            # Cable line
            cable_lines[j].set_data_3d([cable_origin[0], cable_end[0]],
                                       [cable_origin[1], cable_end[1]],
                                       [cable_origin[2], cable_end[2]])
            # Drone
            drone_scatters[j]._offsets3d = ([cable_end[0]], [cable_end[1]], [cable_end[2]])

            # Drone orientation
            drone_idx = 13 + j * 11
            dq_w, dq_x, dq_y, dq_z = data[i, drone_idx:drone_idx + 4]
            drone_rotation = R.from_quat([dq_x, dq_y, dq_z, dq_w]).as_matrix()
            drone_scale = 0.4
            drone_square_local = np.array([
                [-drone_scale, -drone_scale, 0],
                [ drone_scale, -drone_scale, 0],
                [ drone_scale,  drone_scale, 0],
                [-drone_scale,  drone_scale, 0],
                [-drone_scale, -drone_scale, 0]
            ]) * l
            drone_square_global = (drone_rotation @ drone_square_local.T).T + cable_end
            drone_squares[j].set_data_3d(drone_square_global[:,0], drone_square_global[:,1], drone_square_global[:,2])

            # Thrust direction (drone's +Z)
            thrust_dir = drone_rotation @ np.array([0,0,1])
            qv = ax.quiver(
                cable_end[0], cable_end[1], cable_end[2],
                thrust_dir[0], thrust_dir[1], thrust_dir[2],
                color='green', length=0.3, normalize=True, linewidth=2)
            thrust_arrows.append(qv)

        return box_lines + cable_lines + drone_squares + drone_scatters + thrust_arrows

    # --- Animation
    anim = FuncAnimation(fig, update, frames=len(data), interval=interval, blit=False)
    fig.legend()

    manager = plt.get_current_fig_manager()
    try:
        manager.window.showMaximized()
    except Exception:
        pass

    if save_path:
        anim.save(save_path)
        plt.close()
    else:
        plt.show()

if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1:
        animate_trajectories(filename=sys.argv[1])
    else:
        animate_trajectories()
