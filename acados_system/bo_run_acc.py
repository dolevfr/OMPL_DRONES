import numpy as np
import json
import os
from skopt import gp_minimize
from skopt.space import Real, Integer
import csv
from common import N_sim, accel_traj

# Experiment goal: maximize number of acceleration targets reached, then minimize n_steps

MIN_STEPS = 100         # smallest possible number of steps (set reasonably)
MAX_STEPS = N_sim       # largest possible number of steps

logfile = "bo_log_acc.csv"
with open(logfile, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow([
        "score", "targets_reached", "n_steps", 
        # "exp_difference",
        # "payload_pos_w", "payload_vel_w", 
        "payload_accel_w",
        "payload_quat_w_0", "payload_quat_w_1", "payload_quat_w_2", "payload_quat_w_3",
        "drone_quat_w_0", "drone_quat_w_1", "drone_quat_w_2", "drone_quat_w_3",
        "payload_angvel_w", "drone_angvel_w", "cable_angles_w",
        "N", "velocity_ref", "r_T", "r_tau", "W_Tf", "T_f_initial"
    ])

def write_config_json(params):
    # Unpack parameter vector, including exp_differenc
    param_dict = {
        # "exp_difference": int(params[0]),
        # "payload_pos_w": params[-1],  # Not used
        # "payload_vel_w": params[-1],  # Not used
        "payload_accel_w": params[0],
        "payload_quat_w": [params[1], params[2], params[3], params[4]],
        "drone_quat_w": [params[5], params[6], params[7], params[8]],
        "payload_angvel_w": params[9],
        "drone_angvel_w": params[10],
        "cable_angles_w": params[11],
        "N": int(params[12]),
        "velocity_ref": params[13],
        "r_T": params[14],
        "r_tau": params[15],
        "W_Tf": params[16],
        "T_f_initial": params[17],
    }

    with open("config.json", "w") as f:
        json.dump(param_dict, f, indent=2)

def run_simulation():
    os.system("python3 main.py")

def parse_trajectory():
    try:
        lines = open("trajectory.txt").readlines()
    except FileNotFoundError:
        return np.zeros(3), 0, 1e6

    n_steps = len(lines)
    if not lines:
        return np.zeros(3), 0, 1e6
    # Last acceleration is the last 3 before self.i0 in the row
    last_i0 = int(float(lines[-1].split()[-1]))
    return last_i0, n_steps

def objective(params):
    if os.path.exists("trajectory.txt"):
        os.remove("trajectory.txt")
    write_config_json(params)
    run_simulation()
    last_i0, n_steps = parse_trajectory()

    # If you reach the last point in the reference
    if last_i0 >= len(accel_traj) - 1:
        score = float(n_steps - MIN_STEPS) / (MAX_STEPS - MIN_STEPS)
    else:
        score = len(accel_traj) - last_i0

    row = [score, last_i0, n_steps, int(params[0])] + list(params[1:])
    print("TRIAL RESULT:", row)
    with open(logfile, "a", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(row)
    return score



# Parameter search space
space = [
    # Integer(2, 5, name='exp_difference'),
    # Real(0, 2, name='payload_pos_w'),
    # Real(0, 0.5, name='payload_vel_w'),
    Real(0, 10, name='payload_accel_w'),
    Real(0, 0.5, name='payload_quat_w_0'),
    Real(0, 0.5, name='payload_quat_w_1'),
    Real(0, 0.5, name='payload_quat_w_2'),
    Real(0, 0.5, name='payload_quat_w_3'),
    Real(0, 0.5, name='drone_quat_w_0'),
    Real(0, 0.5, name='drone_quat_w_1'),
    Real(0, 0.5, name='drone_quat_w_2'),
    Real(0, 0.5, name='drone_quat_w_3'),
    Real(0, 0.5, name='payload_angvel_w'),
    Real(0, 0.5, name='drone_angvel_w'),
    Real(0, 0.5, name='cable_angles_w'),
    Integer(13, 20, name='N'),
    Real(0, 10, name='velocity_ref'),
    Real(0, 0.5, name='r_T'),
    Real(0, 0.5, name='r_tau'),
    Real(0, 0.5, name='W_Tf'),
    Real(0, 2, name='T_f_initial'),
]

x0 = [
    # 4,                              # exp_difference
    # 0,                              # payload_pos_w
    # 0.0204933213230895,             # payload_vel_w
    2,                              # payload_accel_w
    0,                              # payload_quat_w_0
    0,                              # payload_quat_w_1
    0,                              # payload_quat_w_2
    0,                              # payload_quat_w_3
    0,                              # drone_quat_w_0
    0,                              # drone_quat_w_1
    0,                              # drone_quat_w_2
    0,                              # drone_quat_w_3
    0.1,                            # payload_angvel_w
    0.1,                            # drone_angvel_w
    0.1,                            # cable_angles_w
    18,                             # N
    10,                             # velocity_ref
    0.01,                           # r_T
    0.01,                           # r_tau
    0.1,                            # W_Tf
    2,                              # T_f_initial
]

if __name__ == "__main__":
    res = gp_minimize(objective, space, n_calls=200, x0=x0, random_state=40)
    print("Best parameters:", res.x)
    print("Best score:", res.fun)
