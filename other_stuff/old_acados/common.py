import os
from pathlib import Path
import numpy as np
import casadi as ca
from typing import Union
# ── 1) TRACK LOADING (unchanged) ──────────────────────────────────────

def getTrack(track_filename):
    """Load s–x–y–z reference track from `tracks/<filename>`."""
    track_file = Path(__file__).parent / "tracks" / track_filename
    data = np.loadtxt(track_file, skiprows=1)
    sref = data[1:, 0]
    xref = data[1:, 1]
    yref = data[1:, 2]
    zref = data[1:, 3]
    return sref, xref, yref, zref


# Example usage (override `TRACK_FILE` at runtime if desired)
TRACK_FILE = "trefoil_track.txt"
s_ref, x_ref, y_ref, z_ref = getTrack(TRACK_FILE)
path_length = s_ref[-1]
num_points  = len(s_ref)

# ── 2) TIMING ───────────────────────────────────────────────────────────

# integration / MPC timing
timeStep = 0.01    # [s] control & integration step
N        = 200     # MPC horizon length
Nsim     = 10      # number of MPC-solve + simulate cycles per run
Tf = N * timeStep

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

# ── 5) PHYSICAL COEFFICIENTS ───────────────────────────────────────────

g0          = 9.80665   # [m/s²] gravity
droneBeta   = 0.01      # angular damping on each drone
payloadBeta = 0.10      # linear damping on payload

# ── 6) CONTROL LIMITS ─────────────────────────────────────────────────

minThrust          = 0.0
maxThrust          = 90.0        # [N]
maxTorquePitchRoll = 0.01        # [N·m]
maxTorqueYaw       = 0.005       # [N·m]

# ── 7) STATE & ANGLE LIMITS ────────────────────────────────────────────

maxDroneAngle    = np.deg2rad(70.0)  # [rad] max tilt
maxDroneVel      = 20.0              # [m/s]
maxPayloadAngle  = np.deg2rad(30.0)  # [rad] payload tilt
maxPayloadVel    = 10.0              # [m/s]
maxPayloadAngVel = 0.5               # [rad/s]

maxCableTheta    = np.deg2rad(10.0)  # [rad] cable deflection
maxCableThetaVel = 2.0               # [rad/s]

# ── 8) DRONE COUNT ─────────────────────────────────────────────────────

DRONE_COUNT = 1  # Set to your number of drones

# ── 9) REFERENCE & WEIGHTS (for acceleration‐tracking MPC) ────────────

# weight matrix on accel error (||a - a_ref||²)
W_acc = np.diag([1.0, 1.0, 1.0])

# ── 10) INITIAL ────────────────────────────────────────────────────────

def build_init_state():
    """
    Constructs initial state vector for the payload + N drones.
    Payload state = [pos(3), quat(4), vel(3), omega(3)] = 13
    Each drone adds: [quat(4), omega(3), theta(1), phi(1), theta_dot(1), phi_dot(1)] = 11
    Total = 13 + 11 * DRONE_COUNT
    """
    # Payload starts at first point on reference path
    init_pos   = np.array([x_ref[0], y_ref[0], z_ref[0]])
    init_quat  = np.array([1, 0, 0, 0])        # no rotation
    init_vel   = np.zeros(3)
    init_omega = np.zeros(3)

    # Each drone: 11 state values initialized to zero
    init_drone = np.zeros(11 * DRONE_COUNT)

    init_zeta = np.concatenate([
        init_pos,
        init_quat,
        init_vel,
        init_omega,
        init_drone
    ])
    return init_zeta

# This will be imported elsewhere:
init_zeta = build_init_state()


# ── 11) HELPERS ────────────────────────────────────────────────────────

def DM2Arr(dm: ca.DM) -> np.ndarray:
    """Convert a CasADi DM to a NumPy array."""
    return np.array(dm.full()).squeeze()

def InterpolLuT(s: Union[ca.MX, float]):
    """B-spline interpolation of the loaded track at curvilinear coord s."""
    x_fun = ca.interpolant("x_ref", "bspline", [s_ref], x_ref)
    y_fun = ca.interpolant("y_ref", "bspline", [s_ref], y_ref)
    z_fun = ca.interpolant("z_ref", "bspline", [s_ref], z_ref)
    return x_fun(s), y_fun(s), z_fun(s)

def quat2rpy(q: ca.MX):
    """Convert quaternion [qw,qx,qy,qz] → roll, pitch, yaw (deg) via CasADi."""
    qw, qx, qy, qz = q[0], q[1], q[2], q[3]
    # formulas from Crazyflie firmware
    roll  = ca.atan2(2*(qw*qx + qy*qz), 1 - 2*(qx**2 + qy**2))
    pitch = ca.asin(2*(qw*qy - qx*qz))
    yaw   = ca.atan2(2*(qw*qz + qx*qy), 1 - 2*(qy**2 + qz**2))
    deg   = 180/np.pi
    return ca.vertcat(roll*deg, pitch*deg, yaw*deg)

def quat_from_vec_to_vec(a, b):
    """
    Returns a quaternion (w, x, y, z) that rotates vector a to vector b.
    Assumes a and b are 3D vectors (can be list, np.array, or shape (3,1) etc).
    """
    a = np.asarray(a).flatten()
    b = np.asarray(b).flatten()

    a = a / (np.linalg.norm(a) + 1e-8)
    b = b / (np.linalg.norm(b) + 1e-8)

    cross_prod = np.cross(a, b)
    dot_prod = np.dot(a, b)
    s = np.sqrt((1 + dot_prod) * 2)

    if s < 1e-8:  # a and b are opposite
        axis = np.array([1.0, 0.0, 0.0]) if abs(a[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
        cross_prod = np.cross(a, axis)
        cross_prod /= np.linalg.norm(cross_prod) + 1e-8
        return np.array([0.0, *cross_prod])  # 180° rotation

    q = np.zeros(4)
    q[0] = 0.5 * s
    q[1:] = cross_prod / s
    return q

def derivative_of_track(s, delta=0.01):
    """
    Estimate the derivative of the track at arc-length s using central differences.
    Returns a unit vector (dx, dy, dz).
    """
    s_prev = max(s - delta, s_ref[0])
    s_next = min(s + delta, s_ref[-1])
    
    x1, y1, z1 = InterpolLuT(s_prev)
    x2, y2, z2 = InterpolLuT(s_next)

    dx, dy, dz = x2 - x1, y2 - y1, z2 - z1
    norm = np.linalg.norm([dx, dy, dz]) + 1e-8
    return dx / norm, dy / norm, dz / norm



