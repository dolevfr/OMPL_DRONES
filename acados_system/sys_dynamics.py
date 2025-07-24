import casadi as ca
import numpy as np
from common import (
    m_payload,       # payload mass
    m_drone,         # single-drone mass
    payloadBeta,     # payload linear damping
    droneBeta,       # drone angular damping
    payloadInertiaInv,  # inverse of payload inertia
    droneInertiaInv,    # inverse of drone inertia
    l,               # cable length
    payload_depth,  # payload half-depth
    payload_width,   # payload half-width
    payload_height,  # payload half-height
    g0,              # gravity
    DRONE_COUNT,     # number of drones
)

# ── 1)  GLOBAL SYMBOLIC STATE & CONTROL VARIABLES ────────────────

T_f_sym = ca.MX.sym('T_f')  # free final time

# Payload: position (3), quaternion (4), linear vel (3), ang vel (3)
px, py, pz = ca.MX.sym('px'), ca.MX.sym('py'), ca.MX.sym('pz')
quat_pw, quat_px, quat_py, quat_pz = \
    ca.MX.sym('quat_pw'), ca.MX.sym('quat_px'), ca.MX.sym('quat_py'), ca.MX.sym('quat_pz')
vpx, vpy, vpz = ca.MX.sym('vpx'), ca.MX.sym('vpy'), ca.MX.sym('vpz')
opx, opy, opz = ca.MX.sym('opx'), ca.MX.sym('opy'), ca.MX.sym('opz')

payload_pos  = ca.vertcat(px, py, pz)
payload_quat = ca.vertcat(quat_pw, quat_px, quat_py, quat_pz)
payload_vel  = ca.vertcat(vpx, vpy, vpz)
payload_ω    = ca.vertcat(opx, opy, opz)

# Now each drone has: quaternion (4), body-ang-vel (3),
# cable angles θ, φ (2) and their rates θ̇, φ̇ (2) → 11 total per drone.
drone_syms = []
for i in range(DRONE_COUNT):
    qw = ca.MX.sym(f'qw_{i}')
    qx = ca.MX.sym(f'qx_{i}')
    qy = ca.MX.sym(f'qy_{i}')
    qz = ca.MX.sym(f'qz_{i}')
    wr = ca.MX.sym(f'wr_{i}')
    wp = ca.MX.sym(f'wp_{i}')
    wy = ca.MX.sym(f'wy_{i}')
    θ   = ca.MX.sym(f'theta_{i}')
    φ   = ca.MX.sym(f'phi_{i}')
    θd  = ca.MX.sym(f'thetaDot_{i}')
    φd  = ca.MX.sym(f'phiDot_{i}')
    drone_syms += [qw, qx, qy, qz, wr, wp, wy, θ, φ, θd, φd]

# Stack full state zeta_f
zeta_f = ca.vertcat(
    payload_pos,      # 3
    payload_quat,     # 4
    payload_vel,      # 3
    payload_ω,        # 3 → 13 total so far
    *drone_syms       # 11 × DRONE_COUNT
)

# Controls: per drone → [thrust, τx, τy, τz]
u_syms = []
for i in range(DRONE_COUNT):
    T  = ca.MX.sym(f'T_{i}')    # thrust magnitude
    τx = ca.MX.sym(f'taux_{i}')
    τy = ca.MX.sym(f'tauy_{i}')
    τz = ca.MX.sym(f'tauz_{i}')
    u_syms += [T, τx, τy, τz]
u = ca.vertcat(*u_syms)


def quat_to_rot(q):
    """
    Build a 3×3 rotation matrix from a unit quaternion q = [qw, qx, qy, qz].
    """
    qw, qx, qy, qz = q[0], q[1], q[2], q[3]
    return ca.vertcat(
        ca.horzcat(1-2*(qy**2 + qz**2), 2*(qx*qy - qz*qw),   2*(qx*qz + qy*qw)),
        ca.horzcat(2*(qx*qy + qz*qw),   1-2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)),
        ca.horzcat(2*(qx*qz - qy*qw),   2*(qy*qz + qx*qw),   1-2*(qx**2 + qy**2))
    )

def quat_mul(a, b):
    """
    Quaternion multiplication a ⊗ b, both 4×1 [qw, qx, qy, qz].
    """
    a1, b1, c1, d1 = a[0], a[1], a[2], a[3]
    a2, b2, c2, d2 = b[0], b[1], b[2], b[3]
    return ca.vertcat(
        a1*a2 - b1*b2 - c1*c2 - d1*d2,  # qw
        a1*b2 + b1*a2 + c1*d2 - d1*c2,  # qx
        a1*c2 - b1*d2 + c1*a2 + d1*b2,  # qy
        a1*d2 + b1*c2 - c1*b2 + d1*a2   # qz
    )


class SysDyn:
    def __init__(self):
        self.n_drones = DRONE_COUNT

    def SetupOde(self):
        """
        Returns:
          zeta_f      : CasADi MX state vector
          dyn_f       : CasADi MX expression for ẋ
          u           : CasADi MX control vector
          proj_constr : (unused here) for path constraints
          dyn_fun     : ca.Function('f', [zeta_f, u], [dyn_f])
        """

        # 2) Payload gravity and damping
        payload_force = ca.vertcat(0, 0, -m_payload * g0)
        payload_torque = ca.DM.zeros(3,1)

        # 3) Unpack payload state
        p_pos   = payload_pos
        p_quat  = payload_quat
        p_vel   = payload_vel
        p_ω     = payload_ω

        # 4) Loop over drones, accumulate payload wrench and build drone derivatives
        xdot_drones = []
        for i in range(self.n_drones):
            # indices into zeta_f & u
            base_x = 13 + i*11
            base_u = 4  * i

            # Drone state slices
            dq = ca.vertcat(*drone_syms[base_x-13:base_x-9])   # quaternion
            ω  = ca.vertcat(*drone_syms[base_x-9: base_x-6])   # angular vel
            θ   = drone_syms[base_x-6]
            φ   = drone_syms[base_x-5]
            θd  = drone_syms[base_x-4]
            φd  = drone_syms[base_x-3]

            # Controls
            T  = u_syms[base_u]
            τ  = ca.vertcat(*u_syms[base_u+1:base_u+4])

            # Rotation matrix from drone quaternion
            R_d = quat_to_rot(dq)  # or use your helper

            # Thrust direction & net force on drone
            thrust_dir = R_d @ ca.vertcat(0, 0, 1)
            F_d = T*thrust_dir + m_drone*ca.vertcat(0,0,-g0)

            # Cable directions
            cableDir = ca.vertcat(ca.sin(θ)*ca.cos(φ),
                                  -ca.cos(θ),
                                  ca.sin(θ)*ca.sin(φ))
            thetaDir = ca.vertcat(ca.cos(θ)*ca.cos(φ),
                                  ca.sin(θ),
                                  ca.cos(θ)*ca.sin(φ))
            phiDir   = ca.vertcat(-ca.sin(φ),
                                   0,
                                   ca.cos(φ))

            # Project forces
            forceOnCable = (F_d.T @ cableDir)[0]*cableDir
            fθ = (F_d.T @ thetaDir)[0]
            fφ = (F_d.T @ phiDir)[0]

            θdd = (fθ / (l*m_drone)) + φd**2 * ca.sin(θ) * ca.cos(θ)
            φdd = ((fφ / (l*m_drone)) - 2 * φd * θd * ca.cos(θ)) / ca.sin(θ)

            # Drone quaternion derivative
            ω_quat = ca.vertcat(0, ω[0], ω[1], ω[2])
            qdot_d = 0.5 * quat_mul(ω_quat, dq)

            # Drone angular acceleration
            α_d = droneInertiaInv @ (τ - droneBeta * ω)

            # Accumulate into payload wrench
            if self.n_drones == 1:
                corner = quat_to_rot(p_quat) @ ca.DM([0, 0, payload_height / 2])
            else:
                # Define corner points for 4 drones
                w = payload_width
                d = payload_depth
                h = payload_height
                corners_local = [
                    ca.DM([-w / 2, -d / 2, h / 2]),
                    ca.DM([ w / 2, -d / 2, h / 2]),
                    ca.DM([ w / 2,  d / 2, h / 2]),
                    ca.DM([-w / 2,  d / 2, h / 2])
                ]
                corner_local = corners_local[i]  # i is the drone index
                corner = quat_to_rot(p_quat) @ corner_local

            payload_force  += forceOnCable
            payload_torque += ca.cross(corner, forceOnCable)

            # Pack this drone’s xdot
            xdot_drones += [
                qdot_d,     # 4
                α_d,        # 3
                θd,         # 1
                φd,         # 1
                θdd,        # 1
                φdd         # 1 → 11 total
            ]

        # 5) Payload linear & angular acceleration
        a_p = (payload_force - payloadBeta*p_vel)/m_payload
        α_p = payloadInertiaInv @ payload_torque

        # 6) Payload quaternion rate
        ωp_quat = ca.vertcat(0, p_ω[0], p_ω[1], p_ω[2])
        qdot_p  = 0.5 * quat_mul(ωp_quat, p_quat)

        # 7) Assemble full ẋ
        dyn_f = T_f_sym * ca.vertcat(
            p_vel,       # 3
            qdot_p,      # 4
            a_p,         # 3
            α_p,         # 3 → 13
            *xdot_drones # 11×DRONE_COUNT
        )

        proj_constr = []  # (no additional constraints here)
        dyn_fun = ca.Function('f', [zeta_f, u, T_f_sym], [dyn_f])

        return zeta_f, dyn_f, u, proj_constr, dyn_fun, a_p, T_f_sym
    

        # # 7) Assemble full ẋ
        # dyn_f = ca.vertcat(
        #     p_vel,       # 3
        #     qdot_p,      # 4
        #     a_p,         # 3
        #     α_p,         # 3 → 13
        #     *xdot_drones # 11×DRONE_COUNT
        # )

        # proj_constr = []  # (no additional constraints here)
        # dyn_fun = ca.Function('f', [zeta_f, u], [dyn_f])

        # return zeta_f, dyn_f, u, proj_constr, dyn_fun, a_p