#!/usr/bin/env python3
import numpy as np
import casadi as ca
from sys_dynamics import SysDyn
from common import m_payload, m_drone, g0, DRONE_COUNT, T_del, l
import subprocess
from plot_trajectories_one import plot_trajectories

def main():
    # 1) Build the CasADi model
    sys_model = SysDyn()
    zeta_f, dyn_f, u, proj_constr, dyn_fun, a_p, T_f_sym = sys_model.SetupOde()

    # 2) Initial state: payload + DRONE_COUNT × (drone+cable)
    payload_state = np.array([
        0.0, 0.0, 0.0,          # px, py, pz
        1.0, 0.0, 0.0, 0.0,     # payload quaternion
        0.0, 0.0, 0.0,         # payload linear vel
        0.0, 0.0, 0.0          # payload angular vel
    ])
    drone_state = np.array([
        1.0, 0.0, 0.0, 0.0,    # drone quaternion
        0.0, 0.0, 0.0,         # drone angular vel
        np.pi/2, np.pi/2, 0.0, 0.0     # theta, phi, thetaDot, phiDot
    ])
    state = np.concatenate([payload_state] + [drone_state]*DRONE_COUNT)

    # 3) Constant hover input
    hover_force = (m_drone + m_payload/DRONE_COUNT) * g0
    u0 = np.concatenate([[hover_force * 1.1, 0.001, 0.001, 0.0] for _ in range(DRONE_COUNT)])

    # 4) Open output file
    out_path = "dynamics_output.txt"
    NUM_STEPS = 250
    with open(out_path, "w") as f:
        # simulate NUM_STEPS steps
        for step in range(NUM_STEPS):
            # one forward-euler step
            state_dot = np.array(dyn_fun(state, u0, 1.0).full()).flatten()

            # --- Nicely formatted state_dot print ---
            # Define state names based on your system structure
            payload_state_names = [
                "payload px", "payload py", "payload pz",
                "payload quat_w", "payload quat_x", "payload quat_y", "payload quat_z",
                "payload vx", "payload vy", "payload vz",
                "payload ωx", "payload ωy", "payload ωz"
            ]

            drone_state_names = [
                "drone{}_quat_w", "drone{}_quat_x", "drone{}_quat_y", "drone{}_quat_z",
                "drone{}_ωx", "drone{}_ωy", "drone{}_ωz",
                "drone{}_θ", "drone{}_φ", "drone{}_θ̇", "drone{}_φ̇"
            ]

            state_labels = payload_state_names.copy()
            for d in range(DRONE_COUNT):
                state_labels += [name.format(d+1) for name in drone_state_names]

            # -- Structured state_dot (derivatives) print --
            print("\nState derivatives (state_dot):")
            # Payload derivatives
            payload_dot_pos = state_dot[0:3]
            payload_dot_quat = state_dot[3:7]
            payload_dot_vel = state_dot[7:10]
            payload_dot_angvel = state_dot[10:13]
            print("Payload pos_dot:   ", np.round(payload_dot_pos, 4))
            print("Payload quat_dot:  ", np.round(payload_dot_quat, 4))
            print("Payload vel_dot:   ", np.round(payload_dot_vel, 4))
            print("Payload angvel_dot:", np.round(payload_dot_angvel, 4))

            drone_state_len = 11  # quat(4), angvel(3), θ, φ, θ̇, φ̇ (5)
            for i in range(DRONE_COUNT):
                offset = 13 + i * drone_state_len
                drone_dot_quat = state_dot[offset:offset+4]
                drone_dot_angvel = state_dot[offset+4:offset+7]
                drone_dot_theta = state_dot[offset+7]
                drone_dot_phi   = state_dot[offset+8]
                drone_dot_theta_dot = state_dot[offset+9]
                drone_dot_phi_dot   = state_dot[offset+10]
                print(f"Drone {i+1} quat_dot:      {np.round(drone_dot_quat, 4)}")
                print(f"Drone {i+1} angvel_dot:    {np.round(drone_dot_angvel, 4)}")
                print(f"Drone {i+1} θ_dot, φ_dot:   ({drone_dot_theta:.4f}, {drone_dot_phi:.4f})")
                print(f"Drone {i+1} θ̇_dot, φ̇_dot:  ({drone_dot_theta_dot:.4f}, {drone_dot_phi_dot:.4f})")

            state = state + 0.01 * state_dot

            # # -- Structured state print --
            # zeta_real = state
            u_real = u0

            # payload_pos = zeta_real[0:3]
            # payload_quat = zeta_real[3:7]
            # payload_vel = zeta_real[7:10]
            # payload_angvel = zeta_real[10:13]
            # print(f"\nStep {step+1}/{NUM_STEPS}")
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
            #     # --- Cartesian cable tip for each drone ---
            #     l_cable = l  # Or your variable for cable length
            #     x_tip = l_cable * np.sin(drone_theta) * np.cos(drone_phi)
            #     y_tip = l_cable * np.sin(drone_theta) * np.sin(drone_phi)
            #     z_tip = l_cable * np.cos(drone_theta)
            #     angle_xy = np.arctan2(y_tip, x_tip) * 180 / np.pi
            #     print(f"Drone {i+1} cable tip: x={x_tip:.4f}, y={y_tip:.4f}, z={z_tip:.4f}, xy-angle={angle_xy:.2f}°")

            #     drone_theta_dot = zeta_real[offset+9]
            #     drone_phi_dot   = zeta_real[offset+10]
            #     print(f"Drone {i+1} quat:      {np.round(drone_quat, 4)}")
            #     print(f"Drone {i+1} angvel:    {np.round(drone_angvel, 4)}")
            #     print(f"Drone {i+1} θ, φ:       ({drone_theta:.4f}, {drone_phi:.4f})")
            #     print(f"Drone {i+1} θ̇, φ̇:      ({drone_theta_dot:.4f}, {drone_phi_dot:.4f})")

            # print("T_f (final time var):", zeta_real[-1])

            # -- Structured control print --
            for i in range(DRONE_COUNT):
                ctrl_offset = 4 * i
                T = u_real[ctrl_offset]
                taux = u_real[ctrl_offset+1]
                tauy = u_real[ctrl_offset+2]
                tauz = u_real[ctrl_offset+3]
                print(f"Drone {i+1} control: T={T:.4f}, τx={taux:.4f}, τy={tauy:.4f}, τz={tauz:.4f}")

            # pack line: state, inputs, duration
            line = np.concatenate([state, u0, [T_del]])
            f.write(" ".join(f"{v:.3f}" for v in line) + "\n")

    print(f"Wrote {NUM_STEPS} lines to {out_path}")

    # Optionally, plot the generated trajectories
    plot_trajectories(out_path)

if __name__ == "__main__":
    main()
