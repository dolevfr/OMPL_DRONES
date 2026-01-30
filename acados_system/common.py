import numpy as np
import casadi as ca
import os
from pathlib import Path
from typing import Union


import json




def getTrack():
    track_file = os.path.join(os.path.dirname(__file__), "pos_ref.txt")
    # load all the data rows (skip only the header)
    array = np.loadtxt(track_file, skiprows=1)
    # keep every data row, including the very first one at s=0
    xref = array[:,0]
    yref = array[:,1]
    zref = array[:,2]

    return xref, yref, zref

[x_ref, y_ref, z_ref] = getTrack()

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


payload_pos_w    = 9.81753931330093
payload_vel_w    = 0.191324202558108
payload_accel_w  = 0.101040322577169
payload_quat_w   = [
    0.0906526699893534,
    0.231870121357868,
    0.246607324703919,
    0.0201462697714508
]
drone_quat_w     = [
    0.272467757523852,
    0.279022962377033,
    0.0379100888996108,
    0.291848075042411
]
payload_angvel_w = 0.263800935539326
drone_angvel_w   = 0.242434653206998
cable_angles_w   = 0.239028910167166
N = 11  # shooting nodes / horizon
velocity_ref = 4.48465183171528
r_T   = 0.095294066664569
r_tau = 0.132363467070303
W_Tf = 0.124419682092534
T_f_initial = 1.58763406393806
N_lookahead = 5

# # Best values for weights so far

# payload_pos_w    = 9.81753931330093
# payload_vel_w    = 0.191324202558108
# payload_accel_w  = 0.101040322577169
# payload_quat_w   = [
#     0.0906526699893534,
#     0.231870121357868,
#     0.246607324703919,
#     0.0201462697714508
# ]
# drone_quat_w     = [
#     0.272467757523852,
#     0.279022962377033,
#     0.0379100888996108,
#     0.291848075042411
# ]
# payload_angvel_w = 0.263800935539326
# drone_angvel_w   = 0.242434653206998
# cable_angles_w   = 0.239028910167166
# N = 11  # shooting nodes / horizon
# velocity_ref = 4.48465183171528
# r_T   = 0.095294066664569
# r_tau = 0.132363467070303
# W_Tf = 0.124419682092534
# T_f_initial = 1.58763406393806
# N_lookahead = 5


CONFIG_PATH = "config.json"
def _try_override_from_config(obj):
    """Override globals in this module using config.json if present."""
    if os.path.exists(CONFIG_PATH):
        with open(CONFIG_PATH, "r") as f:
            config = json.load(f)
        for k, v in config.items():
            # Only update if this variable already exists (optional, but safer)
            if k in obj:
                obj[k] = v

_try_override_from_config(globals())


# timing parameters (you already have these)
T_del   = 0.01            # [s] control step  
T_f     = N * T_del       # = 0.5 s horizon length  
T_sim   = 60*2      # total simulation time [s]  
N_sim   = int(T_sim / T_del)

# your track length
length     = len(x_ref)

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
init_zeta[0] = x_ref[0]  # payload x position
init_zeta[1] = y_ref[0]  # payload y position
init_zeta[2] = z_ref[0]  # payload z position

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

Q_pos = np.eye(3) * payload_pos_w   # 3x3 for x, y, z of payload


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