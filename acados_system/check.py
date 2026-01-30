import numpy as np
import argparse
import common

def compute_path_cost(traj_points, ref_path, thresh=0.5):
    dists = []
    for pt in traj_points:
        closest_ref = ref_path[np.argmin(np.linalg.norm(ref_path - pt, axis=1))]
        d = np.linalg.norm(pt - closest_ref)
        dists.append(d**2)
    total_cost = np.sum(dists)
    scaled_cost = thresh * total_cost / (1.0 + total_cost)
    return total_cost, scaled_cost

def main(trajectory_file, thresh=0.5):
    # Load reference path
    X_REF_PATH = np.column_stack((common.x_ref, common.y_ref, common.z_ref))

    # Load trajectory points
    with open(trajectory_file, "r") as f:
        lines = f.readlines()
    traj_points = np.array([[float(x) for x in line.split()[:3]] for line in lines])

    total_cost, scaled_cost = compute_path_cost(traj_points, X_REF_PATH, thresh=thresh)

    print(f"Sum of squared distances to path: {total_cost:.6f}")
    print(f"Scaled cost (for THRESH={thresh}): {scaled_cost:.6f}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--trajectory", type=str, default="trajectory.txt", help="Path to trajectory.txt")
    parser.add_argument("--thresh", type=float, default=0.5, help="THRESH value for scaling")
    args = parser.parse_args()

    main(args.trajectory, thresh=args.thresh)
