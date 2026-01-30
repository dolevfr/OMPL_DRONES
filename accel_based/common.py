import numpy as np
import casadi as ca
import os
from pathlib import Path
from typing import Union


import json




def getTrack():
    track_file = os.path.join(os.path.dirname(__file__), "../src/python/geometric_path.txt")
    # track_file = os.path.join(os.path.dirname(__file__), "tracks/scaled_helix_track.txt")

    # load all the data rows (skip only the header)
    array = np.loadtxt(track_file, skiprows=1)
    # keep every data row, including the very first one at s=0
    xref = array[500:,0]
    yref = array[500:,1]
    zref = array[500:,2]

    return xref, yref, zref

# [x_ref, y_ref, z_ref] = getTrack()

# ── 3) MASSES & GEOMETRY ───────────────────────────────────────────────

m_payload = 2.0    # [kg] mass of the payload
m_drone   = 0.25   # [kg] mass of each drone

# Payload box (rectangular prism)
payload_width  = 2.0   # [m] x-size
payload_depth  = 2.0   # [m] y-size
payload_height = 1.0   # [m] z-size

# Cable
l = 2.0  # [m] cable length

# Drone body (for inertia)
drone_height      = 0.02  # [m]
drone_rotor_radius = 0.2  # [m]

# ── 4) INERTIA TENSORS ─────────────────────────────────────────────────

# rectangular prism about its center
payloadInertia = ca.DM([
    [ (1/12)*m_payload*(payload_height**2 + payload_depth**2), 0, 0 ],
    [ 0, (1/12)*m_payload*(payload_width**2 + payload_height**2), 0 ],
    [ 0, 0, (1/12)*m_payload*(payload_width**2 + payload_depth**2) ]
])

# cylinder (drone) about its center
droneInertia = ca.DM([
    [ (1/12)*m_drone*(3*drone_rotor_radius**2 + drone_height**2), 0, 0 ],
    [ 0, (1/12)*m_drone*(3*drone_rotor_radius**2 + drone_height**2), 0 ],
    [ 0, 0, (1/2)*m_drone*(drone_rotor_radius**2) ]
])

payloadInertiaInv = ca.DM([
    [12/(m_payload*(payload_height**2 + payload_depth**2)), 0, 0],
    [0, 12/(m_payload*(payload_width**2 + payload_height**2)), 0],
    [0, 0, 12/(m_payload*(payload_width**2 + payload_depth**2))]
])

droneInertiaInv = ca.DM([
    [12/(m_drone*(3*drone_rotor_radius**2 + drone_height**2)), 0, 0],
    [0, 12/(m_drone*(3*drone_rotor_radius**2 + drone_height**2)), 0],
    [0, 0, 2/(m_drone*(drone_rotor_radius**2))]
])

# ── 5) PHYSICAL COEFFICIENTS ───────────────────────────────────────────

g0          = 9.80665   # [m/s²] gravity
droneBeta   = 0.01      # angular damping on each drone
payloadBeta = 0.1      # linear damping on payload

# ── 6) CONTROL LIMITS ─────────────────────────────────────────────────

minThrust          = 0.0
maxThrust          = 1000.0        # [N]
maxTorquePitchRoll = 100.0      # [N·m]
maxTorqueYaw       = 100.0       # [N·m]

# ── 7) STATE & ANGLE LIMITS ────────────────────────────────────────────

maxDroneAngle    = np.deg2rad(70.0)  # [rad] max tilt
maxDroneVel      = 10.0              # [m/s]
maxPayloadAngle  = np.deg2rad(50.0)  # [rad] payload tilt
maxPayloadVel    = 10.0              # [m/s]
maxPayloadAngVel = 10.0              # [rad/s]

maxCableAngle    = np.deg2rad(70.0)  # [rad] cable deflection
maxCableAngleVel = 10.0              # [rad/s]

# ── 8) DRONE COUNT ─────────────────────────────────────────────────────

DRONE_COUNT = 4  # Set to your number of drones

payload_pos_w    = 0
payload_vel_w    = 0

# points_per_segment = 20

exp_difference = 3

payload_accel_w  = 1
payload_quat_w   = [0, 0.37386935089487, 0.0743824193038527, 0.0213865816155907]
drone_quat_w     = [0.5, 0.5, 0.371479020281623, 0.267581190023016]
payload_angvel_w = 0
drone_angvel_w   = 0.239510792463738
cable_angles_w   = 0.5
N = 15  # shooting nodes / horizon
N_lookahead = 2
velocity_ref = 0.580313249589266
r_T   = 0.298039734488618
r_tau = 0.0044932662034903
W_Tf = 0.012692394838726
T_f_initial = 1.24733969664191

# Load accel_traj from acc_ref.txt


# num_targets = 500
# delta_max = 10 ** (-exp_difference)

# desired_accel_list = [np.zeros(3)]  # start at [0,0,0]
# for _ in range(1, num_targets):
#     # Small random delta in each direction
#     delta = np.random.uniform(-delta_max, delta_max, size=3)
#     new_target = desired_accel_list[-1] + delta
#     desired_accel_list.append(new_target)

# # 2. Interpolate between each pair with linspace (exclude endpoint of each except last to avoid duplicates)
# accel_traj = []
# for i in range(len(desired_accel_list) - 1):
#     start = desired_accel_list[i]
#     end = desired_accel_list[i + 1]
#     # 20 points between each pair (including start, excluding end)
#     segment = np.linspace(start, end, points_per_segment, endpoint=False)
#     accel_traj.extend(segment)
# # Add the final target to complete the trajectory (to have exactly points_per_segment * num_targets)
# accel_traj.append(desired_accel_list[-1])

# accel_traj = np.array(accel_traj)  # shape: (points_per_segment*num_targets, 3)

# print("First 10 lines of accel_traj:")
# print(accel_traj[:10])


# # Best values for weights so far

# N = 17  # shooting nodes / horizon
# N_lookahead = 3  # lookahead for path following

# velocity_ref = 0  # [m/s] reference velocity for path following

# payload_pos_w    = 0.0
# payload_vel_w    = 0.1
# payload_accel_w  = 1.38956903223368

# payload_quat_w   = [0.0414797037996611, 0.0, 0.00378934658500981, 0.0211278100033569]
# drone_quat_w     = [0.0271782107729833, 0.0326801277276814, 0.0416161166555375, 0.0644482607258137]

# payload_angvel_w = 0.0167786354737607
# drone_angvel_w   = 0.0716680002077413
# cable_angles_w   = 0.0646608490444732

# r_T   = 0.069508635140028
# r_tau = 0.0766541347284516

# W_Tf = 0.0942173037976629
# T_f_initial = 1.06378896321604

# CONFIG_PATH = "config.json"
# def _try_override_from_config(obj):
#     """Override globals in this module using config.json if present."""
#     if os.path.exists(CONFIG_PATH):
#         with open(CONFIG_PATH, "r") as f:
#             config = json.load(f)
#         for k, v in config.items():
#             # Only update if this variable already exists (optional, but safer)
#             if k in obj:
#                 obj[k] = v

# _try_override_from_config(globals())


# timing parameters (you already have these)
T_del   = 0.01            # [s] control step  
T_f     = N * T_del       # = 0.5 s horizon length  
T_sim   = 0.2      # total simulation time [s]  
N_sim   = int(T_sim / T_del)

# your track length
# length     = len(x_ref)
length = 1

# State and control dimensions
n_states = 13 + 11 * DRONE_COUNT
n_controls = 4 * DRONE_COUNT

X_REF = np.zeros(n_states)

X_REF[3] = 1.0  # payload quaternion w = 1

# each drone has 11 states; their quaternion w is at offset 13 + i*11
for i in range(DRONE_COUNT):
    base = 13 + i*11
    X_REF[base] = 1.0
    X_REF[base + 7] = np.pi / 2    # theta
    X_REF[base + 8] = np.pi / 2    # phi

init_zeta = np.copy(X_REF)
# init_zeta[0] = x_ref[0]  # payload x position
# init_zeta[1] = y_ref[0]  # payload y position
# init_zeta[2] = z_ref[0]  # payload z position

# Hover-like thrust for each drone; no torque
hover_force = (m_drone + m_payload / DRONE_COUNT) * g0
U_REF = np.array([hover_force, 0, 0, 0] * DRONE_COUNT)

Q_weights = (
    [payload_pos_w] * 3
    + payload_quat_w
    + [payload_vel_w] * 3
    + [payload_angvel_w] * 3
    + drone_quat_w  * DRONE_COUNT
    + [drone_angvel_w] * 3 * DRONE_COUNT
    + [cable_angles_w] * 4 * DRONE_COUNT
)

assert len(Q_weights) == n_states

Q  = np.diag(Q_weights)

r_block = [r_T, r_tau, r_tau, r_tau] * DRONE_COUNT
R       = np.diag(r_block)

S = np.eye(3) * payload_accel_w

'''Global variables'''

# track="scaled_helix_track.txt"

# def getTrack():
#     track_file = os.path.join(str(Path(__file__).parent), "tracks/", track)
#     # load all the data rows (skip only the header)
#     array = np.loadtxt(track_file, skiprows=1)
#     # keep every data row, including the very first one at s=0
#     sref = array[:,0]
#     xref = array[:,1]
#     yref = array[:,2]
#     zref = array[:,3]

#     return sref, xref, yref, zref

# [s_ref, x_ref, y_ref, z_ref] = getTrack()