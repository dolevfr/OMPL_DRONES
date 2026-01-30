import numpy as np
import json
import os
import subprocess
from skopt import gp_minimize
from skopt.space import Real, Integer
import csv
from common import N_sim, x_ref, y_ref, z_ref

GOAL = np.array([60, -15, 15])

THRESH = 0.5
MIN_STEPS = 1000      # smallest possible number of steps (adjust as needed)
MAX_STEPS = N_sim  # largest possible number of steps (adjust as needed)

logfile = "bo_log.csv"
with open(logfile, "w", newline="") as f:
    writer = csv.writer(f)
    # Write header
    writer.writerow([
        "score", "distance_to_goal", "n_steps", "sum_dists",
        "payload_pos_w", "payload_vel_w", "payload_accel_w",
        "payload_quat_w_0", "payload_quat_w_1", "payload_quat_w_2", "payload_quat_w_3",
        "drone_quat_w_0", "drone_quat_w_1", "drone_quat_w_2", "drone_quat_w_3",
        "payload_angvel_w", "drone_angvel_w", "cable_angles_w",
        "N", "velocity_ref", "r_T", "r_tau", "W_Tf", "T_f_initial", "N_lookahead"
    ])

def write_config_json(params):
    # Unpack parameter vector, adjust this mapping as needed!
    param_dict = {
        "payload_pos_w": params[0],
        "payload_vel_w": params[1],
        "payload_accel_w": params[2],
        "payload_quat_w": [params[3], params[4], params[5], params[6]],
        "drone_quat_w": [params[7], params[8], params[9], params[10]],
        "payload_angvel_w": params[11],
        "drone_angvel_w": params[12],
        "cable_angles_w": params[13],
        "N": int(params[14]),
        "velocity_ref": params[15],
        "r_T": params[16],
        "r_tau": params[17],
        "W_Tf": params[18],
        "T_f_initial": params[19],
        "N_lookahead": int(params[20])
    }
    with open("config.json", "w") as f:
        json.dump(param_dict, f, indent=2)

def run_simulation():
    # Run your main simulation, change to subprocess if you want error capture.
    os.system("python3 main.py")

MAX_SUM_DISTS = 50000

def parse_trajectory():
    lines = open("trajectory.txt").readlines()
    n_steps = len(lines)
    if not lines:
        return 1e6, 1e6, 1e6  # Add sum_dists=1e6 penalty
    traj_points = np.array([[float(x) for x in line.split()[:3]] for line in lines])
    last = traj_points[-1]
    dist = np.linalg.norm(last - GOAL)
    # Compute sum of squared distances to reference path
    ref_points = np.stack([x_ref, y_ref, z_ref], axis=-1)
    sum_dists = 0.0
    for p in traj_points:
        sq_dists = np.sum((ref_points - p) ** 2, axis=1)
        sum_dists += np.min(sq_dists)
    return dist, n_steps, sum_dists

def objective(params):
    if os.path.exists("trajectory.txt"):
        os.remove("trajectory.txt")
    write_config_json(params)
    run_simulation()
    dist, n_steps, sum_dists = parse_trajectory()
    if dist >= THRESH:
        score = dist
    else:
        score = n_steps / MAX_STEPS + sum_dists / MAX_SUM_DISTS
    row = [score, dist, n_steps, sum_dists] + list(params)
    print("TRIAL RESULT:", row)
    with open(logfile, "a", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(row)
    return score


# Parameter search space: adjust as needed
space = [
    Real(5, 50, name='payload_pos_w'),
    Real(0, 1, name='payload_vel_w'),
    Real(0, 1, name='payload_accel_w'),
    Real(0, 1, name='payload_quat_w_0'),
    Real(0, 1, name='payload_quat_w_1'),
    Real(0, 1, name='payload_quat_w_2'),
    Real(0, 1, name='payload_quat_w_3'),
    Real(0, 1, name='drone_quat_w_0'),
    Real(0, 1, name='drone_quat_w_1'),
    Real(0, 1, name='drone_quat_w_2'),
    Real(0, 1, name='drone_quat_w_3'),
    Real(0, 1, name='payload_angvel_w'),
    Real(0, 1, name='drone_angvel_w'),
    Real(0, 1, name='cable_angles_w'),
    Integer(5, 20, name='N'),
    Real(0, 10, name='velocity_ref'),
    Real(0, 1, name='r_T'),
    Real(0, 1, name='r_tau'),
    Real(0, 1, name='W_Tf'),
    Real(0, 3, name='T_f_initial'),
    Integer(1, 10, name='N_lookahead')
]

x0=[
    9.81753931330093,        # payload_pos_w
    0.191324202558108,       # payload_vel_w
    0.101040322577169,       # payload_accel_w
    0.0906526699893534,      # payload_quat_w_0
    0.231870121357868,       # payload_quat_w_1
    0.246607324703919,       # payload_quat_w_2
    0.0201462697714508,      # payload_quat_w_3
    0.272467757523852,       # drone_quat_w_0
    0.279022962377033,       # drone_quat_w_1
    0.0379100888996108,      # drone_quat_w_2
    0.291848075042411,       # drone_quat_w_3
    0.263800935539326,       # payload_angvel_w
    0.242434653206998,       # drone_angvel_w
    0.239028910167166,       # cable_angles_w
    11,                      # N
    4.48465183171528,        # velocity_ref
    0.095294066664569,       # r_T
    0.132363467070303,       # r_tau
    0.124419682092534,       # W_Tf
    1.58763406393806,        # T_f_initial
    5                        # N_lookahead
]

if __name__ == "__main__":
    res = gp_minimize(objective, space, n_calls=500, x0=x0, random_state=42)
    print("Best parameters:", res.x)
    print("Best score:", res.fun)