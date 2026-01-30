import numpy as np
import json
import os
import subprocess
from skopt import gp_minimize
from skopt.space import Real, Integer
import csv
from common import N_sim

GOAL = np.array([60, -15, 15])

THRESH = 0.5
MIN_STEPS = 1000      # smallest possible number of steps (adjust as needed)
MAX_STEPS = N_sim  # largest possible number of steps (adjust as needed)

logfile = "bo_log.csv"
with open(logfile, "w", newline="") as f:
    writer = csv.writer(f)
    # Write header
    writer.writerow([
        "score", "distance_to_goal", "n_steps",
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

def parse_trajectory():
    lines = open("trajectory.txt").readlines()
    n_steps = len(lines)
    if not lines:
        return 1e6, 1e6  # Penalty if trajectory is empty
    last = np.array([float(x) for x in lines[-1].split()[:3]])
    dist = np.linalg.norm(last - GOAL)
    return dist, n_steps

def objective(params):
    if os.path.exists("trajectory.txt"):
        os.remove("trajectory.txt")
    write_config_json(params)
    run_simulation()
    dist, n_steps = parse_trajectory()
    if dist >= THRESH:
        score = dist
    else:
        score = THRESH * (n_steps - MIN_STEPS) / (MAX_STEPS - MIN_STEPS)
    row = [score, dist, n_steps] + list(params)
    print("TRIAL RESULT:", row)
    with open(logfile, "a", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(row)
    return score

# Parameter search space: adjust as needed
space = [
    Real(0.3, 10, name='payload_pos_w'),
    Real(0, 0.3, name='payload_vel_w'),
    Real(0, 0.3, name='payload_accel_w'),
    Real(0, 0.3, name='payload_quat_w_0'),
    Real(0, 0.3, name='payload_quat_w_1'),
    Real(0, 0.3, name='payload_quat_w_2'),
    Real(0, 0.3, name='payload_quat_w_3'),
    Real(0, 0.3, name='drone_quat_w_0'),
    Real(0, 0.3, name='drone_quat_w_1'),
    Real(0, 0.3, name='drone_quat_w_2'),
    Real(0, 0.3, name='drone_quat_w_3'),
    Real(0, 0.3, name='payload_angvel_w'),
    Real(0, 0.3, name='drone_angvel_w'),
    Real(0, 0.3, name='cable_angles_w'),
    Integer(5, 20, name='N'),
    Real(0, 10, name='velocity_ref'),
    Real(0, 0.3, name='r_T'),
    Real(0, 0.3, name='r_tau'),
    Real(0, 0.3, name='W_Tf'),
    Real(0, 2, name='T_f_initial'),
    Integer(1, 10, name='N_lookahead')
]

x0=[
    0.717772658441723,      # payload_pos_w
    0.1,                    # payload_vel_w
    0.0187917205764987,     # payload_accel_w
    0.0331155008882317,     # payload_quat_w_0
    0.1,                    # payload_quat_w_1
    0.07329845579838,       # payload_quat_w_2
    0.0108886275568025,     # payload_quat_w_3
    0.0625989314601602,     # drone_quat_w_0
    0.0695436339210184,     # drone_quat_w_1
    0.0394924473090709,     # drone_quat_w_2
    0.0281401674579522,     # drone_quat_w_3
    0.0607238685856127,     # payload_angvel_w
    0.0282264616868958,     # drone_angvel_w
    0.0428453232747861,     # cable_angles_w
    12,                     # N
    7.6310767728206,        # velocity_ref
    0.0754469120902419,     # r_T
    0.0484885762410102,     # r_tau
    0.0901042538465612,     # W_Tf
    1.1366754275882,        # T_f_initial
    2                       # N_lookahead
]

if __name__ == "__main__":
    res = gp_minimize(objective, space, n_calls=500, x0=x0, random_state=42)
    print("Best parameters:", res.x)
    print("Best score:", res.fun)