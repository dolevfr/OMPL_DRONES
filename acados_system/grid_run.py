import itertools
import shutil
import os
import glob
import json
import matplotlib.pyplot as plt

# Remove all files in saved_trajs (e.g., old .txt and .png files)
print("Cleaning up saved_trajs directory...")
for f in glob.glob("saved_trajs/*"):
    print(f"Removing file: {f}")
    os.remove(f)

print("Cleaning up images directory...")
for f in glob.glob("images/*"):
    print(f"Removing file: {f}")
    os.remove(f)

if os.path.exists("config.json"):
    print("Removing old config.json...")
    os.remove("config.json")

# --- Define the parameter sweep grid here ---
print("Defining parameter grid...")
grid = {
    'payload_pos_w': [1, 5, 10],
    'payload_vel_w': [1], # Not used in this example
    'payload_quat_w': [[0, 0.1, 0.1, 0],
                       [0, 1, 1, 0],],
    'drone_quat_w': [[0, 0, 0, 0]],
    'payload_angvel_w': [1e-4, 1e-2, 1, 10],
    'drone_angvel_w': [1e-4, 1e-2, 1, 10],
    'cable_angles_w': [1e-4, 1e-2, 1, 10],
    'N': [10],
    'velocity_ref': [1], # Not used in this example
    'r_T': [0.1],
    'r_tau': [0.01],
    'W_Tf': [0.001]
}

param_names = list(grid.keys())
combinations = list(itertools.product(*[grid[k] for k in param_names]))

def write_config_json(param_dict):
    with open("config.json", "w") as f:
        json.dump(param_dict, f, indent=2)
    
def make_run_id(i, param_names, param_dict):
    items = []
    for k in param_names:
        v = param_dict[k]
        if isinstance(v, float):
            vstr = f"{v:.5g}"  # short float
        elif isinstance(v, list):
            vstr = str(v)      # e.g. [0, 0, 0, 0]
        else:
            vstr = str(v)
        items.append(f"{k}{vstr}")
    return f"{i+1}_" + '_'.join(items)

for i, combo in enumerate(combinations):
    param_dict = {k: v for k, v in zip(param_names, combo)}
    write_config_json(param_dict)

    run_id = make_run_id(i, param_names, param_dict)

    traj_tmp = "trajectory.txt"
    traj_out = f"saved_trajs/trajectory_{run_id}.txt"
    plot_out = f"images/plot_{run_id}.png"

    os.system("python3 main.py")  # This should create trajectory.txt

    # --- Only save/plot if trajectory is "long enough" ---
    save_it = False
    if os.path.exists(traj_tmp):
        with open(traj_tmp, "r") as f:
            linecount = sum(1 for _ in f)
        if linecount >= 1000:
            save_it = True

    if save_it:
        shutil.move(traj_tmp, traj_out)
        from plot_trajectories_one import plot_trajectories
        plot_trajectories(filename=traj_out, save_path=plot_out, block=False)
        print(f"Saved {traj_out} and {plot_out} (length={linecount})")
    else:
        # Remove failed/short file if exists
        if os.path.exists(traj_tmp):
            os.remove(traj_tmp)
        print(f"Skipped run {run_id} (trajectory too short: {linecount if 'linecount' in locals() else 0})")


# Optionally, remove config.json after the sweep
if os.path.exists("config.json"):
    os.remove("config.json")

# # --- Optionally plot all results together ---
# traj_files = sorted(glob.glob("saved_trajs/trajectory_*.txt"))
# for traj_file in traj_files:
#     print(f"Plotting {traj_file} ...")
#     plot_trajectories(filename=traj_file, block=False)
# plt.show()
