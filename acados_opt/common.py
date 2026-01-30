import numpy as np
import casadi as ca
import os
import json

def getTrack():
    track_file = os.path.join(os.path.dirname(__file__), "../src/python/geometric_path.txt")
    # load all the data rows (skip only the header)
    array = np.loadtxt(track_file, skiprows=1)
    # keep every data row, including the very first one at s=0
    xref = array[500:,0]
    yref = array[500:,1]
    zref = array[500:,2]

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

payload_pos_w    = 6.34685267659066
payload_vel_w    = 0.0390341561882263
payload_accel_w  = 0.197161047305031
payload_quat_w   = [
    0.104009669155625,
    0.106087686647178,
    0.15682089407128,
    0.109785529567479
]
drone_quat_w     = [
    0.000190891892549588,
    0.242566271740635,
    0.131496869899277,
    0.00115235814899176
]
payload_angvel_w = 0.186050612026738
drone_angvel_w   = 0.119719093074459
cable_angles_w   = 0.170348557039704
N = 6  # shooting nodes / horizon
N_lookahead = 4
r_T   = 0.0457042499390717
r_tau = 0.0583292553636932
W_Tf = 0.213778255907559
T_f_initial = 1.91164102647951
velocity_ref = 1.87433732803992

# # Best values for weights so far
# payload_pos_w    = 0.717772658441723
# payload_vel_w    = 0.1
# payload_accel_w  = 0.0187917205764987
# payload_quat_w   = [0.0331155008882317, 0.1, 0.07329845579838, 0.0108886275568025]
# drone_quat_w     = [0.0625989314601602, 0.0695436339210184, 0.0394924473090709, 0.0281401674579522]
# payload_angvel_w = 0.0607238685856127
# drone_angvel_w   = 0.0282264616868958
# cable_angles_w   = 0.0428453232747861
# N = 12  # shooting nodes / horizon
# N_lookahead = 2
# velocity_ref = 7.6310767728206
# r_T   = 0.0754469120902419
# r_tau = 0.0484885762410102
# W_Tf = 0.0901042538465612
# T_f_initial = 1.1366754275882


def _try_override_from_config(obj):
    CONFIG_PATH = os.environ.get("CONFIG_PATH", "config.json")
    """Override globals in this module using config.json if present."""
    print(f"[common.py] Loading config from: {CONFIG_PATH}")
    if os.path.exists(CONFIG_PATH):
        with open(CONFIG_PATH, "r") as f:
            config = json.load(f)
        print("[common.py] Loaded config values:")
        # from pprint import pprint; pprint(config)
        for k, v in config.items():
            if k in obj:
                obj[k] = v

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

r_block = [r_T, r_tau, r_tau, r_tau] * DRONE_COUNT
R       = np.diag(r_block)

S = np.eye(3) * payload_accel_w