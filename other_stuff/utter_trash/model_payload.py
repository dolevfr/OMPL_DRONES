# ── model_payload.py ───────────────────────────────────────────
import casadi as ca
import numpy as np
from acados_template.builders import CMakeBuilder
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
import sys
from pathlib import Path
import scipy.linalg

# ------------------------------------------------------------------
# 0)  CONFIGURATION ── set number of drones once                     ──
# ------------------------------------------------------------------
NUM_DRONES = 1          # ← change this and everything scales up

# ------------------------------------------------------------------
# 1)  DIMENSIONS
# ------------------------------------------------------------------
NX_PAYLOAD   = 13                       # [pos(3), quat(4), vel(3), ω(3)]
NX_DRONE     = 11                       # [quat(4), ω(3), θ, φ, θ̇, φ̇]
NU_PER_DRONE = 4                        # [thrust, τx, τy, τz]

n_x = NX_PAYLOAD + NUM_DRONES * NX_DRONE
n_u = NUM_DRONES * NU_PER_DRONE
n_p = 3                                 # desired linear accel for payload

# ------------------------------------------------------------------
# 2)  SYMBOLS
# ------------------------------------------------------------------
x = ca.SX.sym('x', n_x)
u = ca.SX.sym('u', n_u)
p = ca.SX.sym('p', n_p)                 # a_des = [a_dx, a_dy, a_dz]'

# ------------------------------------------------------------------
# 3)  CONSTANTS
# ------------------------------------------------------------------
m_p, m_d = 2.0, 0.25            # payload mass, single-drone mass          [kg]
g        = 9.81                 # gravity                                   [m/s²]
l        = 2.0                  # cable length                              [m]
h        = 1.0                  # payload full height (corner uses h/2)     [m]
I_p = ca.DM([[0.833333333, 0,            0],
             [0,            0.833333333, 0],
             [0,            0,           1.333333333]])
I_d = ca.DM([[0.002508333, 0,          0],
             [0,           0.002508333,0],
             [0,           0,          0.005      ]])

beta_p, beta_d = 0.10, 0.01     # linear damping on payload / torque damping on drone

# ------------------------------------------------------------------
# 4)  HELPERS
# ------------------------------------------------------------------
def quat_mul(q, r):
    """Hamilton product (w,x,y,z)."""
    w1,x1,y1,z1 = q[0], q[1], q[2], q[3]
    w2,x2,y2,z2 = r[0], r[1], r[2], r[3]
    return ca.vertcat(
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    )

def quat_to_rotmat(q):
    """3×3 rotation matrix from (w,x,y,z)."""
    w,x,y,z = q[0],q[1],q[2],q[3]
    R = ca.SX.zeros(3,3)
    R[0,0] = 1-2*(y*y+z*z); R[0,1] = 2*(x*y-z*w);   R[0,2] = 2*(x*z+y*w)
    R[1,0] = 2*(x*y+z*w);   R[1,1] = 1-2*(x*x+z*z); R[1,2] = 2*(y*z-x*w)
    R[2,0] = 2*(x*z-y*w);   R[2,1] = 2*(y*z+x*w);   R[2,2] = 1-2*(x*x+y*y)
    return R

# ------------------------------------------------------------------
# 5)  UNPACK PAYLOAD STATE
# ------------------------------------------------------------------
pos_p       = x[0:3]
quat_p      = x[3:7]
vel_p       = x[7:10]
omega_p     = x[10:13]

omega_quat_p = ca.vertcat(0, omega_p)
qdot_p       = 0.5 * quat_mul(omega_quat_p, quat_p)

# force/torque accumulators on the payload
payload_force  = m_p * ca.DM([0,0,-g])
payload_torque = ca.DM.zeros(3,1)

# ------------------------------------------------------------------
# 6)  LOOP OVER DRONES
# ------------------------------------------------------------------
xdot_drone_list = []          # collect drone-specific derivatives
for i in range(NUM_DRONES):
    base  = NX_PAYLOAD + i * NX_DRONE
    ui    = i * NU_PER_DRONE

    # --- drone state slice -------------------------------------------------
    quat_d   = x[base : base+4]
    omega_d  = x[base+4 : base+7]
    theta    = x[base+7]
    phi      = x[base+8]
    thetaDot = x[base+9]
    phiDot   = x[base+10]

    # --- controls ----------------------------------------------------------
    thrust_mag = u[ui]
    tau_d      = u[ui+1 : ui+4]

    # --- kinematics --------------------------------------------------------
    R_d        = quat_to_rotmat(quat_d)
    thrust_dir = R_d @ ca.DM([0,0,1])

    F_d = thrust_mag*thrust_dir + m_d*ca.DM([0,0,-g])

    # cable geometry
    cableDir = ca.vertcat(ca.sin(theta)*ca.cos(phi),
                          ca.sin(theta)*ca.sin(phi),
                          ca.cos(theta))
    thetaDir = ca.vertcat(ca.cos(theta)*ca.cos(phi),
                          ca.cos(theta)*ca.sin(phi),
                         -ca.sin(theta))
    phiDir   = ca.vertcat(-ca.sin(phi),
                           ca.cos(phi),
                           0)

    force_on_cable = (F_d.T @ cableDir)[0]*cableDir
    forceTheta     = (F_d.T @ thetaDir)[0]
    forcePhi       = (F_d.T @ phiDir)[0]

    # --- drone rotational dynamics ----------------------------------------
    alpha_d = ca.solve(I_d, tau_d - beta_d*omega_d)
    omega_quat_d = ca.vertcat(0, omega_d)
    qdot_d = 0.5 * quat_mul(omega_quat_d, quat_d)

    # --- cable angle accelerations ----------------------------------------
    theta_ddot = forceTheta / (l*m_d)
    phi_ddot   = forcePhi   / (l*m_d)

    # --- pack derivatives for this drone ----------------------------------
    xdot_drone_list += [
        qdot_d,             # 4
        alpha_d,            # 3
        thetaDot,           # 1
        phiDot,             # 1
        theta_ddot,         # 1
        phi_ddot            # 1     → 11 in total
    ]

    # --- accumulate payload wrench ----------------------------------------
    corner          = quat_to_rotmat(quat_p) @ ca.DM([0,0,h/2])
    payload_force  += force_on_cable
    payload_torque += ca.cross(corner, force_on_cable)

# ------------------------------------------------------------------
# 7)  PAYLOAD LINEAR & ANGULAR ACCELERATION
# ------------------------------------------------------------------
a_p      = (payload_force - beta_p*vel_p) / m_p
alpha_p  = ca.solve(I_p, payload_torque)

# ------------------------------------------------------------------
# 8)  BUILD ẋ
# ------------------------------------------------------------------
xdot = ca.vertcat(
    vel_p,          #  3  payload position derivative
    qdot_p,         #  4  payload quaternion derivative
    a_p,            #  3  payload linear accel
    alpha_p,        #  3  payload angular accel
    *xdot_drone_list  # 11 × NUM_DRONES
)

# ------------------------------------------------------------------
# 9)  ACADOS MODEL
# ------------------------------------------------------------------
model = AcadosModel()
model.name        = f'payload_{NUM_DRONES}drones'
model.x           = x
model.u           = u
model.p           = p
model.f_expl_expr = xdot
model.f_impl_expr = xdot          # explicit form is fine here

# ------------------------------------------------------------------
# 10)  CREATE AND EXPORT AN OCP SOLVER
# ------------------------------------------------------------------

def export_model():
    print("Starting export_model()...")
    ocp = AcadosOcp()
    ocp.model = model

    # discretization & solver options
    ocp.solver_options.N_horizon   = 20
    ocp.solver_options.qp_solver   = 'FULL_CONDENSING_QPOASES'
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
    ocp.solver_options.tf          = 1.0

    ocp.parameter_values = np.zeros(n_p)
    ocp.code_export_directory = 'c_generated_code'

    # 1) state bounds for stages 1..N
    idxbx = np.arange(n_x)
    lbx   = np.full(n_x, -1e3)
    ubx   = np.full(n_x,  1e3)
    ocp.constraints.idxbx = idxbx
    ocp.constraints.lbx   = lbx
    ocp.constraints.ubx   = ubx

    # 2) initial‐state box‐bounds template
    ocp.constraints.idxbx_0 = idxbx
    ocp.constraints.lbx_0   = lbx.copy()
    ocp.constraints.ubx_0   = ubx.copy()

    # <<< THIS IS THE CRUCIAL LINE >>>
    ocp.constraints.x0  = np.zeros(n_x)     # enable stage-0 box bounds

    # 3) control bounds
    ocp.constraints.idxbu = np.arange(n_u)
    ocp.constraints.lbu   = np.array([0.0, -1.0, -1.0, -1.0] * NUM_DRONES)
    ocp.constraints.ubu   = np.array([10.0,  1.0,  1.0,  1.0] * NUM_DRONES)

    # 4) create & generate solver
    ocp_solver = AcadosOcpSolver(ocp, json_file='acados_ocp.json')
    ocp_solver.generate(ocp, 'acados_ocp.json')
    return ocp_solver






if __name__ == "__main__":
    export_model()
