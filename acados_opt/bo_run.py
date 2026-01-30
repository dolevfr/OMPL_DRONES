import os
import subprocess
import json
import csv
import shutil
import numpy as np
import optuna
from multiprocessing import Lock
import multiprocessing
from concurrent.futures import ProcessPoolExecutor
from common import N_sim, x_ref, y_ref, z_ref
import time

X_REF_PATH = np.column_stack((x_ref, y_ref, z_ref))  # shape (N_path, 3)

# Constants
GOAL = np.array([60, -15, 15])
THRESH = 0.5
MIN_STEPS = 1000
MAX_STEPS = N_sim
MAX_SUM_DISTS = 30000
LOGFILE = "bo_log.csv"
os.makedirs("trial_runs", exist_ok=True)  # Folder to isolate trial files

# Lock for thread-safe file writing
log_lock = Lock()

def write_config_json(params, path):
    param_dict = {
        "payload_pos_w": params[0],
        "payload_vel_w": params[1],
        "payload_accel_w": params[2],
        "payload_quat_w": params[3:7],
        "drone_quat_w": params[7:11],
        "payload_angvel_w": params[11],
        "drone_angvel_w": params[12],
        "cable_angles_w": params[13],
        "N": int(params[14]),
        "r_T": params[15],
        "r_tau": params[16],
        "W_Tf": params[17],
        "T_f_initial": params[18],
        "N_lookahead": int(params[19]),
        "velocity_ref": params[20]
    }
    with open(path, "w") as f:
        json.dump(param_dict, f, indent=2)

def parse_trajectory(path):
    traj_path = os.path.join(path, "trajectory.txt")
    print(f"[parse_trajectory] Checking {traj_path}")
    if not os.path.exists(traj_path):
        print("[parse_trajectory] File does not exist!")
        return 1e6, 1e6, 1e6  # Now returns (dist, n_steps, sum_dists)
    with open(traj_path, "r") as f:
        lines = f.readlines()
    if not lines:
        print("[parse_trajectory] File is empty!")
        return 1e6, 1e6, 1e6
    # Parse trajectory points
    traj_points = np.array([[float(x) for x in line.split()[:3]] for line in lines])
    last = traj_points[-1]
    dist = np.linalg.norm(last - GOAL)
    n_steps = len(traj_points)
    # Compute sum of squared distances to closest ref point
    ref_points = np.stack([x_ref, y_ref, z_ref], axis=-1)
    sum_dists = 0.0
    for p in traj_points:
        sq_dists = np.sum((ref_points - p) ** 2, axis=1)
        sum_dists += np.min(sq_dists)
    print(f"[parse_trajectory] dist={dist}, steps={n_steps}, sum_dists={sum_dists}")
    return dist, n_steps, sum_dists

def run_trial(trial_params, trial_number):
    trial_dir = os.path.join("trial_runs", f"trial_{trial_number}")
    os.makedirs(trial_dir, exist_ok=True)

    config_path = os.path.join(trial_dir, "config.json")
    traj_path = os.path.join(trial_dir, "trajectory.txt")
    log_path = os.path.join(trial_dir, "log.txt")

    write_config_json(trial_params, config_path)
    with open(config_path) as f:
        print("[in bo_run.py] Config for this trial:", config_path, f.read())

    env = os.environ.copy()
    env["CONFIG_PATH"] = "config.json"
    env["TRAJECTORY_PATH"] = "trajectory.txt"
    env["TRIAL_BUILD_DIR"] = "."

    with open(log_path, 'w') as log_file:
        subprocess.run(
            ["python3", os.path.abspath("main.py")],
            env=env,
            cwd=trial_dir,
            stdout=log_file,
            stderr=subprocess.STDOUT
        )

    dist, n_steps, sum_dists = parse_trajectory(trial_dir)

    # New cost logic
    if dist >= THRESH:
        score = dist
    else:
        score = THRESH * (n_steps / MAX_STEPS + sum_dists / MAX_SUM_DISTS)

    row = [score, dist, n_steps, sum_dists] + list(trial_params)

    with log_lock:
        with open(LOGFILE, "a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(row)

    print(f"[Trial {trial_number}] score={score:.6f}, dist={dist:.3f}, steps={n_steps}, sum_dists={sum_dists:.3f}")
    return score


def objective(trial):
    params = [
        trial.suggest_float('payload_pos_w', 0.3, 10),
        trial.suggest_float('payload_vel_w', 0, 0.3),
        trial.suggest_float('payload_accel_w', 0, 0.3),
        trial.suggest_float('payload_quat_w_0', 0, 0.3),
        trial.suggest_float('payload_quat_w_1', 0, 0.3),
        trial.suggest_float('payload_quat_w_2', 0, 0.3),
        trial.suggest_float('payload_quat_w_3', 0, 0.3),
        trial.suggest_float('drone_quat_w_0', 0, 0.3),
        trial.suggest_float('drone_quat_w_1', 0, 0.3),
        trial.suggest_float('drone_quat_w_2', 0, 0.3),
        trial.suggest_float('drone_quat_w_3', 0, 0.3),
        trial.suggest_float('payload_angvel_w', 0, 0.3),
        trial.suggest_float('drone_angvel_w', 0, 0.3),
        trial.suggest_float('cable_angles_w', 0, 0.3),
        trial.suggest_int('N', 5, 20),
        trial.suggest_float('r_T', 0, 0.3),
        trial.suggest_float('r_tau', 0, 0.3),
        trial.suggest_float('W_Tf', 0, 0.3),
        trial.suggest_float('T_f_initial', 0, 2),
        trial.suggest_int('N_lookahead', 1, 10),
        trial.suggest_float('velocity_ref', 0.5, 5)
    ]
    return run_trial(params, trial.number)

x0 = [
    0.717772658441723,        # payload_pos_w
    0.1,                      # payload_vel_w
    0.0187917205764987,       # payload_accel_w
    0.0331155008882317,       # payload_quat_w_0
    0.1,                      # payload_quat_w_1
    0.07329845579838,         # payload_quat_w_2
    0.0108886275568025,       # payload_quat_w_3
    0.0625989314601602,       # drone_quat_w_0
    0.0695436339210184,       # drone_quat_w_1
    0.0394924473090709,       # drone_quat_w_2
    0.0281401674579522,       # drone_quat_w_3
    0.0607238685856127,       # payload_angvel_w
    0.0282264616868958,       # drone_angvel_w
    0.0428453232747861,       # cable_angles_w
    12,                       # N
    0.0754469120902419,       # r_T
    0.0484885762410102,       # r_tau
    0.0901042538465612,       # W_Tf
    1.1366754275882,          # T_f_initial
    2,                        # N_lookahead
    2                         # velocity_ref
]

param_names = [
    'payload_pos_w', 'payload_vel_w', 'payload_accel_w',
    'payload_quat_w_0', 'payload_quat_w_1', 'payload_quat_w_2', 'payload_quat_w_3',
    'drone_quat_w_0', 'drone_quat_w_1', 'drone_quat_w_2', 'drone_quat_w_3',
    'payload_angvel_w', 'drone_angvel_w', 'cable_angles_w',
    'N', 'r_T', 'r_tau', 'W_Tf', 'T_f_initial', 'N_lookahead', 'velocity_ref'
]

x0_dict = dict(zip(param_names, x0))

if __name__ == "__main__":
    multiprocessing.set_start_method('spawn', force=True)

    # Clear previous runs
    if os.path.exists("trial_runs"):
        shutil.rmtree("trial_runs")
    os.makedirs("trial_runs", exist_ok=True)
    if os.path.exists(LOGFILE):
        os.remove(LOGFILE)

    # Write CSV header once
    if not os.path.exists(LOGFILE):
        with open(LOGFILE, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "score", "distance_to_goal", "n_steps", "sum_dists",
                "payload_pos_w", "payload_vel_w", "payload_accel_w",
                "payload_quat_w_0", "payload_quat_w_1", "payload_quat_w_2", "payload_quat_w_3",
                "drone_quat_w_0", "drone_quat_w_1", "drone_quat_w_2", "drone_quat_w_3",
                "payload_angvel_w", "drone_angvel_w", "cable_angles_w",
                "N", "r_T", "r_tau", "W_Tf", "T_f_initial", "N_lookahead", "velocity_ref"
            ])

    study = optuna.create_study(direction="minimize")
    study.enqueue_trial(x0_dict)
    study.optimize(objective, n_trials=500, n_jobs=15)
    
    print("Best parameters:", study.best_params)
    print("Best score:", study.best_value)
