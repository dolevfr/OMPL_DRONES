import matplotlib.pyplot as plt
from matplotlib.ticker import ScalarFormatter
import numpy as np
from scipy.spatial.transform import Rotation as R

def plot_trajectories(filename="/home/dolev/Desktop/Research/OMPL_drones/build/solution_path.txt", a=20.0):
    # Load data from the solution path file
    data = np.loadtxt(filename)
    
    # Determine the number of drones based on the number of columns
    num_drones = data.shape[1] // 7
    print(f"Number of drones: {num_drones}")

    # Number of points to plot
    to_plot = 500

    # Randomly select 10,000 points
    if data.shape[0] > to_plot:
        indices = np.random.choice(data.shape[0], to_plot, replace=False)
        data = data[indices]

    # Create a scatter plot for each drone's trajectory
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    all_x, all_y, all_z = [], [], []

    for drone_id in range(num_drones):
        x = data[:, drone_id * 7]
        y = data[:, drone_id * 7 + 1]
        z = data[:, drone_id * 7 + 2]
        
        # Quaternion components
        qw = data[:, drone_id * 7 + 3]
        qx = data[:, drone_id * 7 + 4]
        qy = data[:, drone_id * 7 + 5]
        qz = data[:, drone_id * 7 + 6]

        all_x.append(x)
        all_y.append(y)
        all_z.append(z)
        
        ax.scatter(x, y, z, label=f"Drone {drone_id + 1}")

        # Plot edges of squares for each point
        for i in range(len(x)):
            # Define a square in the drone's local frame
            half_a = a / 2
            local_square = np.array([
                [-half_a, -half_a, 0],
                [ half_a, -half_a, 0],
                [ half_a,  half_a, 0],
                [-half_a,  half_a, 0],
                [-half_a, -half_a, 0]  # Close the square
            ])

            # Convert quaternion to rotation matrix
            r = R.from_quat([qx[i], qy[i], qz[i], qw[i]])
            rotation_matrix = r.as_matrix()

            # Rotate and translate the square
            global_square = (rotation_matrix @ local_square.T).T + np.array([x[i], y[i], z[i]])

            # Plot edges
            ax.plot(global_square[:, 0], global_square[:, 1], global_square[:, 2], color='r')

    # Flatten lists to calculate global ranges for uniform scaling
    all_x = np.concatenate(all_x)
    all_y = np.concatenate(all_y)
    all_z = np.concatenate(all_z)

    # Set labels and legend
    ax.set_xlabel("X Position")
    ax.set_ylabel("Y Position")
    ax.set_zlabel("Z Position")
    ax.legend()

    # Make axes uniform in scale
    max_range = max(all_x.max() - all_x.min(), all_y.max() - all_y.min(), all_z.max() - all_z.min()) / 2
    mid_x = (all_x.max() + all_x.min()) / 2
    mid_y = (all_y.max() + all_y.min()) / 2
    mid_z = (all_z.max() + all_z.min()) / 2

    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)

    # Set axis tick formatting to avoid scientific notation
    ax.xaxis.set_major_formatter(ScalarFormatter(useOffset=False))
    ax.yaxis.set_major_formatter(ScalarFormatter(useOffset=False))
    ax.zaxis.set_major_formatter(ScalarFormatter(useOffset=False))

    # Show the plot
    plt.show()

# Call the function if the script is executed as a standalone program
if __name__ == "__main__":
    plot_trajectories()
