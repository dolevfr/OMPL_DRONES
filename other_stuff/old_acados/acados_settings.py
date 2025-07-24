import numpy as np
import scipy.linalg
import casadi as ca
import os
import json
from acados_template import (
    AcadosModel, AcadosOcp, AcadosOcpSolver, AcadosSimSolver, AcadosSim, ACADOS_INFTY
)
from common import N, timeStep, Tf, init_zeta, W_acc, DRONE_COUNT, s_ref, x_ref, y_ref, z_ref
from sys_dynamics import SysDyn

class AcadosCustomOcp:
    def __init__(self):
        # placeholders
        self.nx = None
        self.nu = None
        self.ny = None
        self.ocp = None
        self.solver = None
        self.integrator = None
        self.zeta_0 = init_zeta.copy()

    def setup_acados_ocp(self):
        # 1) build your system model
        sysModel = SysDyn()
        zeta_f, dyn_f, u, proj_constr, dyn_fun = sysModel.SetupOde()

        # 2) create AcadosModel
        model = AcadosModel()
        model.name        = "payload_accel"
        model.x           = zeta_f
        model.u           = u
        model.f_expl_expr = dyn_f

        # 3) OCP + dimensions
        ocp = AcadosOcp()
        ocp.model = model
        ocp.solver_options.N_horizon = N
        self.nx = model.x.size()[0]
        self.nu = model.u.size()[0]

        # 4) STAGE COST: track payload linear acceleration
        ocp.cost.cost_type    = "NONLINEAR_LS"
        ocp.model.cost_y_expr = dyn_f[7:10]         # a_px, a_py, a_pz
        ocp.cost.yref         = np.zeros(3)         # overwritten each step
        ocp.cost.W            = np.eye(3) * 10.0    # tune weight

        # 5) TERMINAL COST: on payload velocity (state-only, no u), zero‐weighted
        ocp.cost.cost_type_e    = "NONLINEAR_LS"
        ocp.model.cost_y_expr_e = zeta_f[7:10]      # vx, vy, vz
        ocp.cost.yref_e         = np.zeros(3)
        ocp.cost.W_e            = np.zeros((3,3))   # zero weight → effectively no terminal cost

        # 6) initial condition constraint
        ocp.constraints.x0 = self.zeta_0

        # 7) solver options for the MPC
        ocp.solver_options.tf                    = Tf
        ocp.solver_options.integrator_type       = "ERK"
        ocp.solver_options.sim_method_num_stages = 4
        ocp.solver_options.sim_method_num_steps  = 1
        ocp.solver_options.qp_solver             = "PARTIAL_CONDENSING_HPIPM"
        ocp.solver_options.hessian_approx        = "GAUSS_NEWTON"
        ocp.solver_options.nlp_solver_type       = "SQP_RTI"
        ocp.solver_options.tol                   = 1e-3

        # 8) build the MPC solver (fresh JSON + rebuild)
        self.ocp    = ocp
        self.solver = AcadosOcpSolver(
            ocp,
            json_file="planner_accel.json",
            build=True,
            generate=True,
        )

        # 9) build a separate one‐step integrator at dt = timeStep
        sim = AcadosSim()
        sim.model = self.ocp.model

        # ← HERE: must set solver_options.T
        sim.solver_options.T                   = timeStep
        sim.solver_options.integrator_type     = "ERK"
        sim.solver_options.sim_method_num_stages = 4
        sim.solver_options.sim_method_num_steps  = 1

        self.integrator = AcadosSimSolver(sim)

        return True


    def step(self, zeta_current):
        # 1) set initial state
        self.solver.set(0, "lbx", zeta_current)
        self.solver.set(0, "ubx", zeta_current)

        # 2) solve
        status = self.solver.solve()
        if status != 0:
            raise RuntimeError(f"Acados returned status {status}")

        # 3) get first control
        u0 = self.solver.get(0, "u")

        # 4) forward‐simulate **one** dt
        zeta_next = self.integrator.simulate(zeta_current, u0)

        return u0, zeta_next



    def get_cost(self):
        return self.solver.get_cost()
