import numpy as np

points_per_segment = 20
exp_difference = 3

num_targets = 500
delta_max = 10 ** (-exp_difference)

desired_accel_list = [np.zeros(3)]  # start at [0,0,0]
for _ in range(1, num_targets):
    # Small random delta in each direction
    delta = np.random.uniform(-delta_max, delta_max, size=3)
    new_target = desired_accel_list[-1] + delta
    desired_accel_list.append(new_target)

# 2. Interpolate between each pair with linspace (exclude endpoint of each except last to avoid duplicates)
accel_traj = []
for i in range(len(desired_accel_list) - 1):
    start = desired_accel_list[i]
    end = desired_accel_list[i + 1]
    # 20 points between each pair (including start, excluding end)
    segment = np.linspace(start, end, points_per_segment, endpoint=False)
    accel_traj.extend(segment)
# Add the final target to complete the trajectory (to have exactly points_per_segment * num_targets)
accel_traj.append(desired_accel_list[-1])

accel_traj = np.array(accel_traj)  # shape: (points_per_segment*num_targets, 3)

np.savetxt("acc_ref.txt", accel_traj, fmt="%.8f")