import os
import numpy as np
from matplotlib import pyplot, animation

from common import f_plot, refresh_ms, rob_rad, sphere_scale  # subsample & sphere size
from sys_dynamics import SysDyn
# from acados_template import latexify_plot
# latexify_plot()

def animOptVars(misc_steps, traj_ST, traj_U):
    '''Plot data as animate matplotlib graph'''

    # subsample for speed
    misc = misc_steps[:, ::f_plot]
    ST   = traj_ST[:, :, ::f_plot]
    U    = traj_U[:, ::f_plot]
    times = misc[0, :]  # time vector

    # extract payload state
    px, py, pz       = ST[0, :, :],  ST[1, :, :],  ST[2, :, :]
    vx, vy, vz       = ST[7, :, :],  ST[8, :, :],  ST[9, :, :]
    qw, qx, qy, qz   = ST[3, :, :],  ST[4, :, :],  ST[5, :, :], ST[6, :, :]

    # controls for drone 0
    ctrl0 = U[0:4, :]

    # figure setup
    fig = pyplot.figure(figsize=(15, 10))

    # 1) Payload position vs time
    ax1 = fig.add_subplot(3, 3, 1)
    ax1.set_xlabel('time (s)');  ax1.set_ylabel('position (m)')
    line_px, = ax1.plot([], [], label='x'); line_py, = ax1.plot([], [], label='y')
    line_pz, = ax1.plot([], [], label='z')
    ax1.grid(); ax1.legend(loc='upper right')

    # 4) Payload velocity vs time
    ax4 = fig.add_subplot(3, 3, 4)
    ax4.set_xlabel('time (s)');  ax4.set_ylabel('velocity (m/s)')
    line_vx, = ax4.plot([], [], label='vx'); line_vy, = ax4.plot([], [], label='vy')
    line_vz, = ax4.plot([], [], label='vz')
    ax4.grid(); ax4.legend(loc='upper right')

    # 2) Drone-0 controls vs time
    ax2 = fig.add_subplot(3, 3, 2)
    ax2.set_xlabel('time (s)');  ax2.set_ylabel('control')
    ctrl_lines = []
    for lbl in ['T', 'τx', 'τy', 'τz']:
        ln, = ax2.plot([], [], label=lbl)
        ctrl_lines.append(ln)
    ax2.grid(); ax2.legend(ncol=2, loc='upper right')

    # 3) Payload quaternion vs time
    ax3 = fig.add_subplot(3, 3, 3)
    ax3.set_xlabel('time (s)');  ax3.set_ylabel('quaternion')
    q_lines = []
    for lbl in ['qw','qx','qy','qz']:
        ln, = ax3.plot([], [], label=lbl)
        q_lines.append(ln)
    ax3.grid(); ax3.legend(loc='upper right')

    # 3D path + horizon
    ax3d = fig.add_subplot(3, 3, (5, 9), projection='3d')
    ax3d.set_xlabel('X (m)');  ax3d.set_ylabel('Y (m)');  ax3d.set_zlabel('Z (m)')
    path_line,   = ax3d.plot([], [], [], 'b', alpha=0.5, linewidth=0.5)
    horizon_line,= ax3d.plot([], [], [], 'x-g', alpha=0.5)
    drone_dot    = ax3d.scatter([], [], [], s=np.pi*rob_rad**2*sphere_scale, c='lightcoral', alpha=0.45)

    # time text
    time_text = ax3d.text2D(0.02, 0.95, '', transform=ax3d.transAxes)

    def init():
        for ln in [line_px, line_py, line_pz,
                   line_vx, line_vy, line_vz,
                   *ctrl_lines, *q_lines]:
            ln.set_data([], [])
        path_line.set_data([], []); path_line.set_3d_properties([])
        horizon_line.set_data([], []); horizon_line.set_3d_properties([])
        drone_dot._offsets3d = ([],[],[])
        time_text.set_text('')
        return [line_px, line_py, line_pz,
                line_vx, line_vy, line_vz,
                *ctrl_lines, *q_lines,
                path_line, horizon_line, drone_dot, time_text]

    def animate(i):
        t = times[i]

        # update position plots
        line_px.set_data(times[:i+1], px[0,:i+1])
        line_py.set_data(times[:i+1], py[0,:i+1])
        line_pz.set_data(times[:i+1], pz[0,:i+1])

        # velocity
        line_vx.set_data(times[:i+1], vx[0,:i+1])
        line_vy.set_data(times[:i+1], vy[0,:i+1])
        line_vz.set_data(times[:i+1], vz[0,:i+1])

        # controls drone 0
        for j, ln in enumerate(ctrl_lines):
            ln.set_data(times[:i+1], ctrl0[j, :i+1])

        # quaternion
        for j, ql in enumerate(q_lines):
            ql.set_data(times[:i+1], [qw, qx, qy, qz][j][0,:i+1])

        # 3D path
        path_line.set_data(px[0,:i+1], py[0,:i+1])
        path_line.set_3d_properties(pz[0,:i+1])

        # horizon = next-step predicted positions
        # ST shape: (nx, N+1, iter)
        # current horizon predicted from i: ST[0:3,1:,i]
        horizon_line.set_data(ST[0,1:,i], ST[1,1:,i])
        horizon_line.set_3d_properties(ST[2,1:,i])

        # drone current dot
        drone_dot._offsets3d = ([px[0,i]], [py[0,i]], [pz[0,i]])

        # time text
        time_text.set_text(f'time = {t:.2f} s')

        return [line_px, line_py, line_pz,
                line_vx, line_vy, line_vz,
                *ctrl_lines, *q_lines,
                path_line, horizon_line, drone_dot, time_text]

    anim = animation.FuncAnimation(
        fig, animate, init_func=init,
        frames=times.shape[0], interval=refresh_ms, blit=False
    )

    fig.tight_layout()
    if os.environ.get("ACADOS_ON_CI") is None:
        pyplot.show()
