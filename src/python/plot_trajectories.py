import matplotlib.pyplot as plt
from matplotlib.ticker import ScalarFormatter
import numpy as np
from scipy.spatial.transform import Rotation as R

def plot_trajectories(filename="/home/dolev/Desktop/Research/OMPL_drones/build/solution_path.txt", a=2.0, l=1.0):
    # Load data from the solution path file
    data = np.loadtxt(filename)

    # Determine the number of drones based on the number of columns
    num_drones = (data.shape[1] - 13) // 10  # 13 columns for payload (SE3 + velocity), 10 per drone
    print(f"System contains payload and {num_drones} drones.")

    # Number of points to plot
    to_plot = 50  # For testing a single state

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

    ax.scatter(payload_x, payload_y, payload_z, label="Payload", color='b', s=50)

    for i in range(len(payload_x)):
        # Define the rod's endpoints (aligned along local x-axis)
        rod_start_local = np.array([-a / 2, 0, 0])  # One end of the rod
        rod_end_local = np.array([a / 2, 0, 0])    # Other end of the rod

        # Convert quaternion to rotation matrix (q_x, q_y, q_z, q_w)
        r = R.from_quat([data[i, 4], data[i, 5], data[i, 6], data[i, 3]])
        rotation_matrix = r.as_matrix()

        # Rotate and translate rod endpoints
        rod_start = rotation_matrix @ rod_start_local + np.array([payload_x[i], payload_y[i], payload_z[i]])
        rod_end = rotation_matrix @ rod_end_local + np.array([payload_x[i], payload_y[i], payload_z[i]])

        # Plot the rod
        ax.plot([rod_start[0], rod_end[0]],
                [rod_start[1], rod_end[1]],
                [rod_start[2], rod_end[2]], color='g', linewidth=2)

        # Define cable origins (rod ends)
        cable_origins = [rod_start, rod_end]

        for j, cable_origin in enumerate(cable_origins):
            # Cable angles (theta, phi)
            theta = data[i, 7]
            phi = data[i, 8]

            # Convert spherical coordinates to Cartesian
            cable_x = l * np.sin(theta) * np.cos(phi)
            cable_y = l * np.sin(theta) * np.sin(phi)
            cable_z = l * np.cos(theta)

            cable_vector = np.array([cable_x, cable_y, cable_z])
            cable_end = cable_origin + cable_vector

            # Plot the cable
            ax.plot([cable_origin[0], cable_end[0]],
                    [cable_origin[1], cable_end[1]],
                    [cable_origin[2], cable_end[2]], color='r', linewidth=1.5)

            # Plot drone at the end of the cable
            ax.scatter(cable_end[0], cable_end[1], cable_end[2], color='k', s=40)
            if i == 0:  # Add legend entry only for the first point
                ax.scatter([], [], color='k', s=40, label=f"Drone {j + 1}")

    # Flatten lists to calculate global ranges for uniform scaling
    all_x = np.concatenate(all_x)
    all_y = np.concatenate(all_y)
    all_z = np.concatenate(all_z)

    # Zoom into the plot by setting tighter limits
    padding = 2  # Adjust this value for more or less zoom
    ax.set_xlim(all_x.min() - padding, all_x.max() + padding)
    ax.set_ylim(all_y.min() - padding, all_y.max() + padding)
    ax.set_zlim(all_z.min() - padding, all_z.max() + padding)

    # Set labels and legend
    ax.set_xlabel("X Position")
    ax.set_ylabel("Y Position")
    ax.set_zlabel("Z Position")
    ax.legend()

    # Set axis tick formatting to avoid scientific notation
    ax.xaxis.set_major_formatter(ScalarFormatter(useOffset=False))
    ax.yaxis.set_major_formatter(ScalarFormatter(useOffset=False))
    ax.zaxis.set_major_formatter(ScalarFormatter(useOffset=False))

    # Show the plot
    plt.show()

# Call the function if the script is executed as a standalone program
if __name__ == "__main__":
    plot_trajectories()
