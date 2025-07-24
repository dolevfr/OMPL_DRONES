import os
import numpy as np
from matplotlib import pyplot as plt, animation

from common import (
    s_ref, x_ref, y_ref, z_ref,
)

# ── LOCAL ANIMATION SETTINGS ───────────────────────────────────────────
f_plot       = 10      # subsample factor for faster animation
refresh_ms   = 10      # delay between frames, in milliseconds
rob_rad      = 0.04    # payload “radius” for scatter sizing
sphere_scale = 20000   # scale factor for scatter marker size

def animOptVars(misc_steps: np.ndarray, traj_ST: np.ndarray, traj_U: np.ndarray):
    """
    Animate the payload position over time in 3D.

    Inputs:
      misc_steps  2×M array (we ignore it here, but kept for signature consistency)
      traj_ST    nx×(M+1) state history; payload pos = traj_ST[0:3, :]
      traj_U      control history (unused here)
    """
    # subsample for speed
    xs = traj_ST[0,   ::f_plot]
    ys = traj_ST[1,   ::f_plot]
    zs = traj_ST[2,   ::f_plot]
    T  = xs.size

    # set up figure + 3D axis
    fig = plt.figure(figsize=(8,6))
    ax  = fig.add_subplot(111, projection='3d')
    ax.set_title("Payload Trajectory")
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")
    ax.plot(x_ref, y_ref, z_ref,
            linestyle='--', marker='x', alpha=0.4,
            label="Reference Path")

    # initial scatter (empty)
    payload_scatter = ax.scatter([], [], [], 
                                 s=np.pi*rob_rad**2 * sphere_scale, 
                                 c='red', alpha=0.6,
                                 label="Payload")

    ax.legend()

    def init():
        payload_scatter._offsets3d = ([], [], [])
        return (payload_scatter,)

    def animate(i):
        xi, yi, zi = xs[i], ys[i], zs[i]
        # update payload position
        payload_scatter._offsets3d = ([xi], [yi], [zi])
        return (payload_scatter,)

    anim = animation.FuncAnimation(
        fig, animate, init_func=init,
        frames=T, interval=refresh_ms, blit=False, repeat=True
    )

    # only show if not in a CI environment
    if os.environ.get("ACADOS_ON_CI") is None:
        plt.show()
