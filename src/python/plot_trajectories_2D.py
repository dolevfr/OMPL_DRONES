import matplotlib.pyplot as plt
from matplotlib.ticker import ScalarFormatter
import numpy as np
from scipy.spatial.transform import Rotation as R

def plot_trajectories(filename="/home/dolev/Desktop/Research/OMPL_drones/build/solution_path.txt", a=2.0, l=1.0):
    # Load data from the solution path file
    data = np.loadtxt(filename)
    # print(f"Quaternion raw values: {data[:, [6, 3, 4, 5]]}")
    # return

    # Determine the number of drones based on the number of columns
    num_drones = (data.shape[1] - 13) // 10  # 13 columns for payload (SE3 + velocity), 10 per drone
    print(f"System contains payload and {num_drones} drones.")

    # Number of points to plot
    to_plot = 50

    # Randomly select points if there are too many
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

    # Plot the start position
    start_x, start_y, start_z = 125, 125, -150
    ax.scatter(start_x, start_y, start_z, label="Start", color='c', s=100, marker='*')

    # Plot the goal position
    goal_x, goal_y, goal_z = 150, 125, -150
    ax.scatter(goal_x, goal_y, goal_z, label="Goal", color='m', s=100, marker='*')

    ax.scatter(payload_x, payload_y, payload_z, label="Payload", color='b', s=50)

    for i in range(len(payload_x)):
        # Define the rod's endpoints (aligned along local x-axis)
        rod_start_local = np.array([-a / 2, 0, 0])  # One end of the rod
        rod_end_local = np.array([a / 2, 0, 0])    # Other end of the rod
        # print(f"quaternion at point {i}:\n{[data[i, 6], data[i, 3], data[i, 4], data[i, 5]]}")
        # Convert quaternion to rotation matrix (q_x, q_y, q_z, q_w)
        if np.linalg.norm([data[i, 6], data[i, 3], data[i, 4], data[i, 5]]) > 1e-8:
            r = R.from_quat([data[i, 6], data[i, 3], data[i, 4], data[i, 5]])
            # print(f"Quaternion at point {i}:\n{r.as_quat()}")
            rotation_matrix = r.as_matrix()
            # print(f"Rotation matrix at point {i}:\n{rotation_matrix}")
        else:
            continue

        # Rotate and translate rod endpoints
        rod_start = rotation_matrix @ rod_start_local + np.array([payload_x[i], payload_y[i], payload_z[i]])
        rod_end = rotation_matrix @ rod_end_local + np.array([payload_x[i], payload_y[i], payload_z[i]])
        # print(f"Rod start at point {i}:\n{rod_start}")
        # print(f"Rod end at point {i}:\n{rod_end}")

        # Plot the rod
        ax.plot([rod_start[0], rod_end[0]],
                [rod_start[1], rod_end[1]],
                [rod_start[2], rod_end[2]], color='g', linewidth=2)

        # Plot a square for the payload
        square_local = np.array([
            [-0.5, -0.5, 0],  # Bottom-left
            [0.5, -0.5, 0],   # Bottom-right
            [0.5, 0.5, 0],    # Top-right
            [-0.5, 0.5, 0],   # Top-left
            [-0.5, -0.5, 0]   # Close the square
        ]) * a  # Scale the square size based on `a`

        square_global = (rotation_matrix @ square_local.T).T + np.array([payload_x[i], payload_y[i], payload_z[i]])
        # ax.plot(square_global[:, 0], square_global[:, 1], square_global[:, 2], color='b', linewidth=1.5)

        # Define cable origins (rod ends)
        cable_origins = [rod_start, rod_end]

        for j, cable_origin in enumerate(cable_origins):
            # Cable angles (theta, phi)
            theta = data[i, 12 + j * 11 + 8]
            phi = data[i, 12 + j * 11 + 9]

            # print(f"Theta at point {i}:\n{theta}")
            # print(f"Phi at point {i}:\n{phi}")

            # Convert spherical coordinates to Cartesian
            cable_x = l * np.sin(theta) * np.cos(phi)
            cable_y = l * np.sin(theta) * np.sin(phi)
            cable_z = l * np.cos(theta)

            cable_vector = np.array([cable_x, cable_y, cable_z])
            # print(f"Cable vector at point {i}:\n{cable_vector}")
            cable_end = cable_origin + cable_vector

            # Plot the cable
            ax.plot([cable_origin[0], cable_end[0]],
                    [cable_origin[1], cable_end[1]],
                    [cable_origin[2], cable_end[2]], color='r', linewidth=1.5)

            # Plot drone at the end of the cable
            drone_color = 'k' if j == 0 else 'brown'
            ax.scatter(cable_end[0], cable_end[1], cable_end[2], color=drone_color, s=40)

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

            drone_idx = 12 + j * 11

            if np.linalg.norm([data[i, drone_idx + 3], data[i, drone_idx], data[i, drone_idx + 1], data[i, drone_idx + 2]]) > 1e-8:
                drone_rotation = R.from_quat([data[i, drone_idx + 3], data[i, drone_idx], data[i, drone_idx + 1], data[i, drone_idx + 2]]).as_matrix()
                drone_square_global = (drone_rotation @ drone_square_local.T).T + cable_end
                ax.plot(drone_square_global[:, 0], drone_square_global[:, 1], drone_square_global[:, 2], color='orange', linewidth=1.5)
            else:
                continue

            if i == 0:  # Add legend entry only for the first point
                ax.scatter([], [], color=drone_color, s=40, label=f"Drone {j + 1}")


    # Flatten lists to calculate global ranges for uniform scaling
    all_x = np.concatenate(all_x)
    all_y = np.concatenate(all_y)
    all_z = np.concatenate(all_z)

    # Zoom into the plot by setting tighter limits
    padding = 2  # Adjust this value for more or less zoom
    x_limits = (all_x.min() - padding, all_x.max() + padding)
    y_limits = (all_y.min() - padding, all_y.max() + padding)
    z_limits = (all_z.min() - padding, all_z.max() + padding)

    ax.set_xlim(*x_limits)
    ax.set_ylim(*y_limits)
    ax.set_zlim(*z_limits)

    # Make axes equal
    max_range = max(
        x_limits[1] - x_limits[0],
        y_limits[1] - y_limits[0],
        z_limits[1] - z_limits[0],
    ) / 2.0

    mid_x = (x_limits[1] + x_limits[0]) / 2.0
    mid_y = (y_limits[1] + y_limits[0]) / 2.0
    mid_z = (z_limits[1] + z_limits[0]) / 2.0

    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)

    # Set labels and legend
    ax.set_xlabel("X Position")
    ax.set_ylabel("Y Position")
    ax.set_zlabel("Z Position")
    ax.legend()

    # Set axis tick formatting to avoid scientific notation
    ax.xaxis.set_major_formatter(ScalarFormatter(useOffset=False))
    ax.yaxis.set_major_formatter(ScalarFormatter(useOffset=False))
    ax.zaxis.set_major_formatter(ScalarFormatter(useOffset=False))

    # Show the plot in full screen
    manager = plt.get_current_fig_manager()
    manager.window.showMaximized()
    plt.show()
    


# Call the function if the script is executed as a standalone program
if __name__ == "__main__":
    plot_trajectories()
