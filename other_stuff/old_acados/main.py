import numpy as np
from time import time
from common import Nsim, N, s_ref, InterpolLuT, quat_from_vec_to_vec, derivative_of_track
from acados_settings import AcadosCustomOcp
from visualize_mpl import animOptVars

def plan_ocp(ocp_wrapper):
    nx, nu = ocp_wrapper.nx, ocp_wrapper.nu
    zeta_cur = ocp_wrapper.zeta_0.copy()
    times, costs = [], []
    ST = zeta_cur.reshape(nx,1)
    U  = np.zeros((nu,0))

    s_end = s_ref[-1]
    ds    = s_end / float(Nsim)
    s_cur = 0.0
    h     = ds     # finite-difference step

    for k in range(Nsim):
        print(f"\n=== Iter {k+1} | s_cur = {s_cur:.3f}/{s_end:.3f} ===")
        if s_cur >= s_end:
            print("Reached end of track.")
            break

        # --- STAGE ACCELERATION REFS (ny = 3) ---
        for j in range(N+1):
            s_j    = min(s_cur + j*ds, s_end)
            s_prev = max(s_j - h,     0.0)
            s_next = min(s_j + h, s_end)

            p_prev = np.array(InterpolLuT(s_prev))
            p_mid  = np.array(InterpolLuT(s_j))
            p_next = np.array(InterpolLuT(s_next))

            # central-difference accel
            acc_j  = (p_next - 2*p_mid + p_prev) / (h*h)

            if j == 0:
                print("  setting stage acc ref shape:", acc_j.shape,
                      "| ocp.dims.ny =", ocp_wrapper.ocp.dims.ny)

            ocp_wrapper.solver.set(j, "yref", acc_j.flatten())

        # --- SOLVE + SIMULATE ONE STEP ---
        t0            = time()
        u0, zeta_next = ocp_wrapper.step(zeta_cur)
        t_solve       = time() - t0

        print("  u0 =", np.round(u0,3), 
              "| next pos ≃", np.round(zeta_next[0:3],3))

        # record
        times.append(t_solve)
        costs.append(ocp_wrapper.get_cost())
        ST = np.hstack([ST, zeta_next.reshape(nx,1)])
        U  = np.hstack([U,   u0.reshape(nu,1)])

        # advance
        zeta_cur = zeta_next
        s_cur    = min(s_cur + ds, s_end)

    misc = np.vstack([times, costs])
    return misc, ST, U




if __name__ == '__main__':
    # 1) build & configure OCP
    ocp = AcadosCustomOcp()
    assert ocp.setup_acados_ocp()

    # 2) run closed‐loop MPC
    misc, ST, U = plan_ocp(ocp)

    # 3) animate results
    animOptVars(misc, ST, U)
