import common
from acados_settings import AcadosCustomOcp
import os
import multiprocessing

TRAJECTORY_PATH = os.environ.get("TRAJECTORY_PATH", "trajectory.txt")
build_dir = os.environ.get("TRIAL_BUILD_DIR", ".")
os.makedirs(build_dir, exist_ok=True)

def plan_ocp(ocp_wrapper: AcadosCustomOcp):
    """
    Motion control problem of drone trajectory tracking,
    using solve_and_sim() to handle the solve + simulate in one call.
    Immediately writes each [x..., u..., t] row to trajectory.txt after each step.
    """
    import numpy as np
    from time import time

    # bookkeeping
    t0 = 0.0
    total_solve = 0.0
    T_f_value = common.T_f_initial
    solve_times = []

    zeta_real = np.copy(ocp_wrapper.zeta_0)
    u_real    = np.copy(common.U_REF)
    ocp_wrapper.zeta_N = np.tile(zeta_real.reshape(-1,1), (1, common.N+1))
    ocp_wrapper.u_N    = np.tile(u_real.reshape(-1,1),  (1, common.N))
    state_hist   = [zeta_real.copy()]
    control_hist = [u_real.copy()]
    misc_hist    = [(0.0, 0.0)]

    with open(TRAJECTORY_PATH, 'w') as traj_file:
        row = np.hstack([zeta_real, u_real, t0])
        np.savetxt(traj_file, row.reshape(1, -1), fmt="%.6f")
        traj_file.flush()

        for k in range(common.N_sim):
            # print(f"\n=== MPC iter {k}, sim t={t0:.2f}s ===")
            done = ocp_wrapper.cost_update_ref(zeta_real, common.U_REF)
            if done:
                print("Track complete!")
                break

            ocp_wrapper.solver.set(0, "lbx", zeta_real)
            ocp_wrapper.solver.set(0, "ubx", zeta_real)
            for i in range(common.N+1):
                ocp_wrapper.solver.set(i, "x", ocp_wrapper.zeta_N[:, i])
            for j in range(common.N):
                ocp_wrapper.solver.set(j, "u", ocp_wrapper.u_N[:, j])

            t_start = time()
            ocp_wrapper.solve_and_sim(T_f_value)
            # ocp_wrapper.solve_and_sim()
            dt = time() - t_start

            total_solve += dt
            solve_times.append(dt)

            # pull out the new real state & applied control
            zeta_real = np.array(ocp_wrapper.zeta_0).flatten()
            u_real    = np.array(ocp_wrapper.u_N[:, 0]).flatten()

            # # -- Structured state print --
            # payload_pos = zeta_real[0:3]
            # payload_quat = zeta_real[3:7]
            # payload_vel = zeta_real[7:10]
            # payload_angvel = zeta_real[10:13]
            # print("Payload pos:   ", np.round(payload_pos, 4))
            # print("Payload quat:  ", np.round(payload_quat, 4))
            # print("Payload vel:   ", np.round(payload_vel, 4))
            # print("Payload angvel:", np.round(payload_angvel, 4))

            # drone_state_len = 11  # quat(4), angvel(3), θ, φ, θ̇, φ̇ (5)
            # for i in range(common.DRONE_COUNT):
                # offset = 13 + i * drone_state_len
                # drone_quat = zeta_real[offset:offset+4]
                # drone_angvel = zeta_real[offset+4:offset+7]
                # drone_theta = zeta_real[offset+7]
                # drone_phi   = zeta_real[offset+8]
                # drone_theta_dot = zeta_real[offset+9]
                # drone_phi_dot   = zeta_real[offset+10]
                # print(f"Drone {i+1} quat:      {np.round(drone_quat, 4)}")
                # print(f"Drone {i+1} angvel:    {np.round(drone_angvel, 4)}")
                # print(f"Drone {i+1} θ, φ:       ({drone_theta:.4f}, {drone_phi:.4f})")
                # print(f"Drone {i+1} θ̇, φ̇:      ({drone_theta_dot:.4f}, {drone_phi_dot:.4f})")

            # print("T_f (final time var):", zeta_real[-1])

            # # -- Structured control print --
            # for i in range(common.DRONE_COUNT):
            #     ctrl_offset = 4 * i
            #     T = u_real[ctrl_offset]
            #     taux = u_real[ctrl_offset+1]
            #     tauy = u_real[ctrl_offset+2]
            #     tauz = u_real[ctrl_offset+3]
            #     print(f"Drone {i+1} control: T={T:.4f}, τx={taux:.6f}, τy={tauy:.6f}, τz={tauz:.6f}")

            cost_now = ocp_wrapper.get_cost()
            # print(f"Cost: {cost_now:.4f}")
            # print("-"*60)

            t0 += common.T_del
            state_hist.append(zeta_real.copy())
            control_hist.append(u_real.copy())
            misc_hist.append((t0, cost_now))

            row = np.hstack([zeta_real, u_real, t0])
            np.savetxt(traj_file, row.reshape(1, -1), fmt="%.6f")
            traj_file.flush()

    solve_times = np.array(solve_times)
    print(f"Max. solve time: {solve_times.max()*1000:.1f} ms")
    print(f"Avg. solve time: {solve_times.mean()*1000:.1f} ms")

    misc_arr     = np.array(misc_hist).T
    states_arr   = np.stack(state_hist,   axis=1)
    controls_arr = np.stack(control_hist, axis=1)
    return misc_arr, states_arr, controls_arr



if __name__ == '__main__':

    build_dir = os.environ.get("TRIAL_BUILD_DIR", ".")

    common._try_override_from_config(common.__dict__)

    custom_ocp = AcadosCustomOcp(build_dir=build_dir)
    custom_ocp.setup_acados_ocp()

    # Print all key parameters after config is loaded
    from pprint import pprint
    # print("\n========= [main.py] PARAMETERS IN USE =========")
    # pprint({
    #     "payload_pos_w": common.payload_pos_w,
    #     "payload_vel_w": common.payload_vel_w,
    #     "payload_accel_w": common.payload_accel_w,
    #     "payload_quat_w": common.payload_quat_w,
    #     "drone_quat_w": common.drone_quat_w,
    #     "payload_angvel_w": common.payload_angvel_w,
    #     "drone_angvel_w": common.drone_angvel_w,
    #     "cable_angles_w": common.cable_angles_w,
    #     "N": common.N,
    #     "N_lookahead": common.N_lookahead,
    #     "velocity_ref": common.velocity_ref,
    #     "r_T": common.r_T,
    #     "r_tau": common.r_tau,
    #     "W_Tf": common.W_Tf,
    #     "T_f_initial": common.T_f_initial,
    # })
    # print("=====================================\n")

    print("[main.py] PID:", os.getpid())
    print("[main.py] CONFIG_PATH:", os.environ.get("CONFIG_PATH"))
    print("[main.py] TRAJECTORY_PATH:", os.environ.get("TRAJECTORY_PATH"))

    try:
        print(">>> [main.py] Starting plan_ocp()")
        plan_ocp(custom_ocp)
        print(">>> [main.py] Finished plan_ocp()")        # os.system("python3 animate_trajectories.py")
        # os.system("python3 plot_trajectories_one.py")
    except Exception as e:
        print("[main.py] Planning failed with exception:", repr(e))
        raise  # or at least print full traceback
        # print("Planning failed:", e)
        # print("Partial trajectory written to trajectory.txt")
        # os.system("python3 animate_trajectories.py")
        # os.system("python3 plot_trajectories_one.py")