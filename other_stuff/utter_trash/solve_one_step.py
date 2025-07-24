# solve_one_step.py (in src/python)

from model_payload import export_model
import numpy as np
import sys

class PythonSolver:
    def __init__(self):
        self.solver = export_model()     # build Acados solver once
        self.N      = self.solver.N      # prediction-horizon length (public)

    def solve(self, x0: np.ndarray, a_des: np.ndarray):
        import sys
        print(f"[PythonSolver] x0[:5] = {x0[:5]}", file=sys.stderr)
        print(f"[PythonSolver] a_des  = {a_des}",  file=sys.stderr)

        # 1) Validate x0
        if x0.shape != (24,) or not np.isfinite(x0).all():
            raise ValueError(f"x0 must be a finite (24,) array, got {x0.shape}")

        # 2) Stage-0 equality box (lbx == ubx == x0)
        lbx = x0.copy()
        ubx = x0.copy()
        self.solver.constraints_set(0, "lbx", lbx)
        self.solver.constraints_set(0, "ubx", ubx)

        # 3) Parameter 'p' at each stage
        for k in range(self.N + 1):
            self.solver.set(k, "p", a_des)

        # 4) Solve OCP
        status = self.solver.solve()
        if status != 0:
            raise RuntimeError(f"acados solver returned status {status}")

        u0 = self.solver.get(0, "u")
        print(f"[PythonSolver] u[0:5] = {u0[:5]}", file=sys.stderr)
        return u0




_bridge = None

def get_solver():
    global _bridge
    if _bridge is None:
        _bridge = PythonSolver()
    return _bridge
