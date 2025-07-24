import casadi as ca
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver, AcadosSimSolver, ACADOS_INFTY
import scipy.linalg

from common import *
from sys_dynamics import SysDyn

class AcadosCustomOcp:

    def __init__(self):
        self.nx = 0
        self.nu = 0
        self.ns = 0

        self.ocp = None,
        self.solver = None,
        self.integrator = None
        self.sysModel = None

        self.zeta_0 = None
        self.zeta_N = None
        self.u_N = None
        self.last_yref = None
        self.first_ref = True
        self.last_nn_idx = 0
        self.i0 = 1

        


    def setup_acados_ocp(self):
        '''Formulate acados OCP'''

        # create casadi symbolic expressions
        sysModel = SysDyn()
        self.sysModel = sysModel

        zeta_f, dyn_f, u, proj_constr, dyn_fn, a_p, T_f_sym = sysModel.SetupOde()
        self.zeta_0 = np.copy(init_zeta)

        # create Acados model
        ocp = AcadosOcp()
        model_ac = AcadosModel()
        model_ac.f_expl_expr = dyn_f
        model_ac.x = zeta_f
        model_ac.u = u
        model_ac.p = ca.vertcat(T_f_sym)
        model_ac.name = "drone_FrenSer"
        ocp.model = model_ac

        ocp.parameter_values = np.array([T_f_initial])

        # set dimensions
        ocp.solver_options.N_horizon = N
        self.nx = model_ac.x.size()[0]
        self.nu = model_ac.u.size()[0]

        self.zeta_N = ca.repmat(np.reshape(self.zeta_0, (self.nx,1)), 1, N+1)
        self.u_N = ca.repmat(U_REF.reshape((self.nu,1)), 1, N)

        # continuity constraints
        ocp.constraints.x0 = self.zeta_0


        # ── full-state + input sliding-window cost ─────────────────────────────
        # cost on the entire state x (length nx) and input u (length nu)

        ocp.model.cost_y_expr = ca.vertcat(   # [ x; u ]
            model_ac.x,
            model_ac.u,
            a_p,
            T_f_sym
        )
        ocp.cost.cost_type   = "NONLINEAR_LS"

        # just concatenate them once:
        ocp.cost.yref = np.concatenate([X_REF, U_REF, np.zeros(3), np.array([0.0])])

        # block-diag weights: Q on the state portion, R on the input portion
        ocp.cost.W = scipy.linalg.block_diag(Q, R, S, np.array([[W_Tf]]))


        # ── terminal cost on state only ────────────────────────────────────────
        ocp.cost.cost_type_e    = "NONLINEAR_LS"
        ocp.model.cost_y_expr_e = model_ac.x    # only x at the last node

        # reference at terminal: zero state except identity quaternions
        yref_e = np.zeros(self.nx)
        yref_e[3] = 1.0
        for i in range(DRONE_COUNT):
            yref_e[13 + i*11] = 1.0
        ocp.cost.yref_e = yref_e

        # terminal weight on the state
        ocp.cost.W_e = Q

        # constrain AGV dynamics : acceleration, angular velocity (convex ?, Non-linear)
        dyn_constr_eqn = []
        dyn_constr_eqn = ca.vertcat(dyn_constr_eqn , proj_constr)

        ineq_constr_eqn = []
        ineq_constr_eqn = ca.vertcat(ineq_constr_eqn, dyn_constr_eqn)

        model_ac.con_h_expr = ineq_constr_eqn
        model_ac.con_h_expr_e = ineq_constr_eqn

        # enforce payload quaternion unit‐norm
        q_pl = model_ac.x[3:7]                          # payload quaternion
        model_ac.con_h_expr = ca.vertcat(
            model_ac.con_h_expr,
            ca.dot(q_pl, q_pl) - 1.0
        )
        model_ac.con_h_expr_e = ca.vertcat(
            model_ac.con_h_expr_e,
            ca.dot(q_pl, q_pl) - 1.0
        )

        # inequality bounds
        nh = model_ac.con_h_expr.shape[0]

        # Bounds on path constraints (inequality)
        lh = np.zeros(nh);        uh = np.zeros(nh)
        lh[:] = -ACADOS_INFTY;     uh[:] = 1

        ocp.constraints.lh   = lh
        ocp.constraints.uh   = uh
        ocp.constraints.lh_e = lh
        ocp.constraints.uh_e = uh

        # ---------------------------------
        # 1) per-control bounds on [T_i, τx_i, τy_i, τz_i]:
        lbu = np.zeros(self.nu)
        ubu = np.zeros(self.nu)
        for i in range(DRONE_COUNT):
            u0 = 4*i
            lbu[u0]   = minThrust
            ubu[u0]   = maxThrust
            lbu[u0+1:u0+4] = -maxTorquePitchRoll
            ubu[u0+1:u0+4] = maxTorquePitchRoll
        ocp.constraints.idxbu = np.arange(self.nu)
        ocp.constraints.lbu   = lbu
        ocp.constraints.ubu   = ubu

        # # 2) per-state (box) bounds on cable-angle θ_i ∈ [−40°, +40°]
        # idxbx, lbx, ubx = [], [], []
        # for i in range(DRONE_COUNT):
        #     theta_idx = 13 + i*11 + 7
        #     phi_idx = 13 + i*11 + 8
        #     idxbx.append(theta_idx)
        #     lbx.append(np.pi/2 - maxCableAngle)
        #     ubx.append(np.pi/2 + maxCableAngle)
        #     idxbx.append(phi_idx)
        #     lbx.append(np.pi - maxCableAngle)
        #     ubx.append(np.pi + maxCableAngle)

        # # === Add essential lower bound on T_f ===
        # T_f_idx = self.nx - 1   # Assuming T_f is the last state variable
        # Tf_min = 1e-8            # Lower bound for T_f (must be > 0, e.g. 0.1 sec)
        # Tf_max = 1e8    # Or any upper bound you prefer

        # idxbx.append(T_f_idx)
        # lbx.append(Tf_min)
        # ubx.append(Tf_max)

        # ocp.constraints.idxbx = np.array(idxbx, dtype=int)
        # ocp.constraints.lbx   = np.array(lbx)
        # ocp.constraints.ubx   = np.array(ubx)


        # ── Integrator setup ─────────────────────────────────────────
        ocp.solver_options.integrator_type              = "ERK"
        ocp.solver_options.tf                           = T_f  # free-final-time parameter
        ocp.solver_options.sim_method_num_stages        = 4
        ocp.solver_options.sim_method_num_steps         = 1

        # ── QP / NLP solver configuration ────────────────────────────
        ocp.solver_options.qp_solver                    = "FULL_CONDENSING_HPIPM"
        ocp.solver_options.hessian_approx               = "GAUSS_NEWTON"
        ocp.solver_options.nlp_solver_type              = "SQP"
        ocp.solver_options.search_direction_mode        = "BYRD_OMOJOKUN"
        ocp.solver_options.allow_direction_mode_switch_to_nominal = False

        # ── Regularization & globalization ───────────────────────────
        ocp.solver_options.levenberg_marquardt                    = 1e-2
        ocp.solver_options.regularize_method                      = 'GERSHGORIN_LEVENBERG_MARQUARDT'
        ocp.solver_options.adaptive_levenberg_marquardt_mu0       = 1e-2
        ocp.solver_options.globalization                          = "MERIT_BACKTRACKING"
        ocp.solver_options.globalization_alpha_min                = 0.1
        ocp.solver_options.globalization_alpha_reduction          = 0.5
        ocp.solver_options.step_size                              = 0.5

        # ── QP tolerances (looser for robustness) ──────────────────
        ocp.solver_options.qp_tol_stat    = 1e-4
        ocp.solver_options.qp_tol_eq      = 1e-4
        ocp.solver_options.qp_tol_ineq    = 1e-4
        ocp.solver_options.qp_tol_comp    = 1e-4

        # ── NLP tolerances and iterations ───────────────────────────
        ocp.solver_options.nlp_solver_tol_stat   = 1e-6
        ocp.solver_options.nlp_solver_tol_eq     = 1e-6
        ocp.solver_options.nlp_solver_tol_ineq   = 1e-6
        ocp.solver_options.nlp_solver_tol_comp   = 1e-6
        ocp.solver_options.nlp_solver_max_iter   = 200

        # ── QP solver iteration limits ──────────────────────────────
        ocp.solver_options.qp_solver_iter_max    = 200

        # ── Warm start for QP (improves convergence in large/hard problems) ──
        ocp.solver_options.qp_solver_warm_start              = 1
        ocp.solver_options.nlp_solver_warm_start_first_qp    = True

        # ── Diagnostic and scaling options ───────────────────────────
        ocp.solver_options.nlp_solver_ext_qp_res    = 0
        ocp.solver_options.print_level              = 0
        ocp.solver_options.qp_print_level           = 0

        # ── Other: use nominal QP direction only if necessary ───────
        ocp.solver_options.allow_direction_mode_switch_to_nominal = True


        # create solver
        self.ocp = ocp
        self.solver = AcadosOcpSolver(ocp, json_file = "planner_ocp.json")
        self.integrator = AcadosSimSolver(ocp)

        print("init_zeta =", init_zeta)
        print("lbx =", ocp.constraints.lbx)
        print("ubx =", ocp.constraints.ubx)
        # print("Is in bounds:", np.all(init_zeta >= ocp.constraints.lbx), np.all(init_zeta <= ocp.constraints.ubx))
        print("T_f_initial =", T_f_initial)


        return True


    def solve_and_sim(self, T_f_value):
        """
        Solve the OCP with multiple shooting, forward simulate with RK4,
        and handle time scaling via the parameter T_f_value.
        """

        # 1) Set T_f parameter for all shooting nodes
        for j in range(N+1):
            self.solver.set(j, "p", np.array([T_f_value]))
            self.integrator.set("p", np.array([T_f_value]))  # If integrator needs the param too

        # 2) Compute optimal control at current state
        try:
            u_0 = self.solver.solve_for_x0(self.zeta_0)
        except RuntimeError as e:
            print(f"[solve_and_sim] QP failed on solve_for_x0: {e}")
            raise  # or handle fallback here

        # 3) One integration step (simulate forward)
        self.zeta_0 = self.integrator.simulate(x=self.zeta_0, u=u_0, p=np.array([T_f_value]))

        # Normalize payload quaternion
        q = self.zeta_0[3:7]
        self.zeta_0[3:7] = q / np.linalg.norm(q)
        # Normalize each drone quaternion
        for i in range(DRONE_COUNT):
            base = 13 + i*11
            qd = self.zeta_0[base:base+4]
            self.zeta_0[base:base+4] = qd / np.linalg.norm(qd)

        # 4) Recompute u_0 at the new state and store trajectories
        for j in range(N+1):
            self.solver.set(j, "p", np.array([T_f_value]))
        try:
            # print("[solve_and_sim] Before second solve_for_x0()", flush=True)
            u_0 = self.solver.solve_for_x0(self.zeta_0)
            # print("[solve_and_sim] After second solver call, u_0=", np.round(u_0, 4))
        except RuntimeError as e:
            print(f"[solve_and_sim] QP failed on second solve_for_x0: {e}")
            raise

        # 5) Collect predicted state trajectory
        self.zeta_N = np.reshape(self.solver.get(0, "x"), (self.nx, 1))
        for j in range(1, N+1):
            zeta_j = np.reshape(self.solver.get(j, "x"), (self.nx, 1))
            self.zeta_N = np.concatenate((self.zeta_N, zeta_j), axis=1)

        # 6) Save first control for plotting
        self.u_N[:, 0] = u_0


    def advance_track_index(self, x0, y0, z0, x_ref, y_ref, z_ref, N_lookahead, window_size=100):
        n = len(x_ref)
        # Use self.i0 if last_nn_idx is not set yet
        if not hasattr(self, "last_nn_idx"):
            self.last_nn_idx = self.i0

        # Always build the window starting from last_nn_idx
        window_start = self.last_nn_idx
        window_end = min(window_start + window_size, n)
        print(f"last_nn_idx = {self.last_nn_idx}, window: [{window_start}, {window_end})")
        xs = x_ref[window_start:window_end]
        ys = y_ref[window_start:window_end]
        zs = z_ref[window_start:window_end]

        dists = []
        for j in range(window_end - window_start):
            dx = xs[j] - x0
            dy = ys[j] - y0
            dz = zs[j] - z0
            dist2 = dx*dx + dy*dy + dz*dz
            dists.append(dist2)

        min_idx_local = int(np.argmin(dists))
        new_nn_idx = window_start + min_idx_local

        advanced_idx = min(new_nn_idx + N_lookahead, n - 1)
        self.i0 = advanced_idx
        self.last_nn_idx = new_nn_idx  # Update for next time

        # print("_________________________________________________________________")
        # print(f"advance_track_index: Closest point at new_nn_idx={new_nn_idx}: ({x_ref[new_nn_idx]:.3f}, {y_ref[new_nn_idx]:.3f}, {z_ref[new_nn_idx]:.3f})")
        # print(f"advance_track_index: Updated self.i0={self.i0}: ({x_ref[self.i0]:.3f}, {y_ref[self.i0]:.3f}, {z_ref[self.i0]:.3f})")
        # print("_________________________________________________________________")



    def cost_update_ref(self, zeta_0, _):
        """
        Reference window includes both position and *velocity*.
        The velocity reference at each point is velocity_ref * (direction to next point).
        """
        x0, y0, z0 = float(zeta_0[0]), float(zeta_0[1]), float(zeta_0[2])

        if self.i0 >= len(x_ref) - 1:
            return True

        # Advance index using your improved window
        self.advance_track_index(x0, y0, z0, x_ref, y_ref, z_ref, N_lookahead)

        print(f"[cost_update_ref] i0={self.i0}, pos_ref={x_ref[self.i0]:.3f},{y_ref[self.i0]:.3f},{z_ref[self.i0]:.3f}")

        for j in range(N):

            idx = min(self.i0 + j, len(x_ref) - 2)  # -2 to allow next_idx below
            next_idx = idx + 1

            xj, yj, zj = x_ref[idx], y_ref[idx], z_ref[idx]
            xj_next, yj_next, zj_next = x_ref[next_idx], y_ref[next_idx], z_ref[next_idx]

            # Velocity direction from idx to next_idx
            vec = np.array([xj_next - xj, yj_next - yj, zj_next - zj])
            norm = np.linalg.norm(vec)
            if norm > 1e-6:
                v_dir = vec / norm
            else:
                v_dir = np.zeros(3)
            v_ref = velocity_ref * v_dir

            # Assemble reference
            full_ref = np.concatenate([X_REF, U_REF, np.zeros(3), np.array([T_f_initial])])
            full_ref[0:3] = [xj, yj, zj]
            # full_ref[7:10] = v_ref

            # print(f"[cost_update_ref] stage {j}: idx={idx}, pos_ref=({xj:.3f}, {yj:.3f}, {zj:.3f})")
            
            # print("==== Reference vector for stage", j, "====")
            # zeta_real = full_ref.copy()
            # payload_pos = zeta_real[0:3]
            # payload_quat = zeta_real[3:7]
            # payload_vel = zeta_real[7:10]
            # payload_angvel = zeta_real[10:13]
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
            #     drone_theta_dot = zeta_real[offset+9]
            #     drone_phi_dot   = zeta_real[offset+10]
            #     print(f"Drone {i+1} quat:      {np.round(drone_quat, 4)}")
            #     print(f"Drone {i+1} angvel:    {np.round(drone_angvel, 4)}")
            #     print(f"Drone {i+1} θ, φ:       ({drone_theta:.4f}, {drone_phi:.4f})")
            #     print(f"Drone {i+1} θ̇, φ̇:      ({drone_theta_dot:.4f}, {drone_phi_dot:.4f})")
            # print("T_f (final time var):", zeta_real[-1])

            # print("====================================")

            self.last_yref = full_ref.copy()
            self.solver.set(j, "yref", full_ref)

        # Terminal node
        idx_T = min(self.i0 + N, len(x_ref)-1)
        xT, yT, zT = x_ref[idx_T], y_ref[idx_T], z_ref[idx_T]
        yref_e = X_REF.copy()
        yref_e[0:3] = [xT, yT, zT]
        self.solver.set(N, "yref", yref_e)

        return False

    def get_cost(self):
        cost = self.solver.get_cost()
        return cost

