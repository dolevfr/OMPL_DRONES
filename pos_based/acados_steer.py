from common import *
from acados_settings import AcadosCustomOcp
import time

def plan_ocp(ocp_wrapper: AcadosCustomOcp, pos_traj):
    """
    Motion control problem of drone trajectory tracking (POSITION version),
    using solve_and_sim() to handle the solve + simulate in one call.
    """

    # bookkeeping
    t0 = 0.0
    total_solve = 0.0
    T_f_value = T_f_initial
    solve_times = []

    zeta_real = np.copy(ocp_wrapper.zeta_0)
    u_real    = np.copy(U_REF)
    ocp_wrapper.zeta_N = np.tile(zeta_real.reshape(-1,1), (1, N+1))
    ocp_wrapper.u_N    = np.tile(u_real.reshape(-1,1),  (1, N))
    state_hist   = [zeta_real.copy()]
    control_hist = [u_real.copy()]
    misc_hist    = [(0.0, 0.0)]

    for k in range(N_sim):
        # Update: cost_update_ref now takes position trajectory
        done = ocp_wrapper.cost_update_ref(zeta_real, pos_traj)
        
        if done:
            print("Track complete!")
            break

        ocp_wrapper.solver.set(0, "lbx", zeta_real)
        ocp_wrapper.solver.set(0, "ubx", zeta_real)
        for i in range(N+1):
            ocp_wrapper.solver.set(i, "x", ocp_wrapper.zeta_N[:, i])
        for j in range(N):
            ocp_wrapper.solver.set(j, "u", ocp_wrapper.u_N[:, j])

        t_start = time.time()
        ocp_wrapper.solve_and_sim(T_f_value)
        dt = time.time() - t_start

        total_solve += dt
        solve_times.append(dt)

        # pull out the new real state & applied control
        zeta_real = np.array(ocp_wrapper.zeta_0).flatten()
        u_real    = np.array(ocp_wrapper.u_N[:, 0]).flatten()

        # print(f"\n=== MPC iter {k}, sim t={t0:.2f}s ===")
        # payload_pos = zeta_real[0:3]
        # payload_quat = zeta_real[3:7]
        # payload_vel = zeta_real[7:10]
        # payload_angvel = zeta_real[10:13]
        # print("Payload pos:   ", np.round(payload_pos, 4))
        # print("Payload quat:  ", np.round(payload_quat, 4))
        # print("Payload vel:   ", np.round(payload_vel, 4))
        # print("Payload angvel:", np.round(payload_angvel, 4))

        # # Print payload acceleration
        # payload_accel = accel_fun(zeta_real, u_real, T_f_value).full().flatten()
        # print("Payload accel: ", np.round(payload_accel, 4))

        # drone_state_len = 11  # quat(4), angvel(3), θ, φ, θ̇, φ̇ (5)
        # for i in range(DRONE_COUNT):
        #     offset = 13 + i * drone_state_len
        #     drone_quat = zeta_real[offset:offset+4]
        #     drone_angvel = zeta_real[offset+4:offset+7]
        #     drone_theta = zeta_real[offset+7]
        #     drone_phi   = zeta_real[offset+8]
        #     drone_theta_dot = zeta_real[offset+9]
        #     drone_phi_dot   = zeta_real[offset+10]
        #     print(f"Drone {i+1} quat:      {np.round(drone_quat, 4)}")
        #     print(f"Drone {i+1} angvel:    {np.round(drone_angvel, 4)}")
        #     print(f"Drone {i+1} θ, φ:       ({drone_theta:.4f}, {drone_phi:.4f})")
        #     print(f"Drone {i+1} θ̇, φ̇:      ({drone_theta_dot:.4f}, {drone_phi_dot:.4f})")

        # print("T_f (final time var):", zeta_real[-1])

        # # -- Structured control print --
        # for i in range(DRONE_COUNT):
        #     ctrl_offset = 4 * i
        #     T = u_real[ctrl_offset]
        #     taux = u_real[ctrl_offset+1]
        #     tauy = u_real[ctrl_offset+2]
        #     tauz = u_real[ctrl_offset+3]
        #     print(f"Drone {i+1} control: T={T:.4f}, τx={taux:.6f}, τy={tauy:.6f}, τz={tauz:.6f}")

        cost_now = ocp_wrapper.get_cost()

        t0 += T_del
        state_hist.append(zeta_real.copy())
        control_hist.append(u_real.copy())
        misc_hist.append((t0, cost_now))

    misc_arr     = np.array(misc_hist).T
    states_arr   = np.stack(state_hist,   axis=0)
    controls_arr = np.stack(control_hist, axis=0)
    return misc_arr, states_arr, controls_arr

def setup_ocp():
    ocp = AcadosCustomOcp()
    ocp.setup_acados_ocp()
    return ocp

# The function you call for each segment expansion
def steer_acados(ocp, current_state, pos_traj):
    """
    ocp: AcadosCustomOcp object (already set up)
    current_state: np.ndarray, shape = (n_state,)
    pos_traj: np.ndarray, shape = (N, 3)  # desired positions
    Returns: [states], [controls], [final_position]
    """

    # print("[DEBUG][OCP] Received initial state:")
    # zeta_real = np.copy(current_state)
    # payload_pos = zeta_real[0:3]
    # payload_quat = zeta_real[3:7]
    # payload_vel = zeta_real[7:10]
    # payload_angvel = zeta_real[10:13]
    # print("Payload pos:   ", np.round(payload_pos, 4))
    # print("Payload quat:  ", np.round(payload_quat, 4))
    # print("Payload vel:   ", np.round(payload_vel, 4))
    # print("Payload angvel:", np.round(payload_angvel, 4))

    # drone_state_len = 11  # quat(4), angvel(3), θ, φ, θ̇, φ̇ (5)
    # for i in range(DRONE_COUNT):
    #     offset = 13 + i * drone_state_len
    #     drone_quat = zeta_real[offset:offset+4]
    #     drone_angvel = zeta_real[offset+4:offset+7]
    #     drone_theta = zeta_real[offset+7]
    #     drone_phi   = zeta_real[offset+8]
    #     drone_theta_dot = zeta_real[offset+9]
    #     drone_phi_dot   = zeta_real[offset+10]
    #     print(f"Drone {i+1} quat:      {np.round(drone_quat, 4)}")
    #     print(f"Drone {i+1} angvel:    {np.round(drone_angvel, 4)}")
    #     print(f"Drone {i+1} θ, φ:       ({drone_theta:.4f}, {drone_phi:.4f})")
    #     print(f"Drone {i+1} θ̇, φ̇:      ({drone_theta_dot:.4f}, {drone_phi_dot:.4f})")

    # print("pos_traj right in steer_acados:")
    # print("[DEBUG] Position trajectory (first row):", pos_traj[0])
    # print("[DEBUG] Position trajectory (last row):", pos_traj[-1])

    ocp.i0 = 0
    ocp.last_nn_idx = 0
    ocp.update_initial_state(current_state)

    global pos_traj_global
    pos_traj_global = pos_traj

    # Now run the planner (plan_ocp) as usual
    misc_arr, states, controls = plan_ocp(ocp, pos_traj)

    # Get final position for the last state
    final_zeta = states[-1, :]  # (shape n_state,)
    final_pos = final_zeta[:3]  # First 3 elements: payload position

    return states, controls, final_pos



