import numpy as np
import casadi as ca

def cable_opt(F_req, corners, mu=0.1, lam=0.5):
    # F_req: list or np.array of size (3,)
    # corners: list of 4 corner vectors (each size 3)

    t = ca.SX.sym("t", 4)
    theta = ca.SX.sym("theta", 4)
    phi   = ca.SX.sym("phi", 4)
    x = ca.vertcat(t, theta, phi)

    def s(theta, phi):
        return ca.vertcat(ca.sin(theta)*ca.cos(phi),
                          -ca.cos(theta),
                          ca.sin(theta)*ca.sin(phi))

    F = sum([t[i] * s(theta[i], phi[i]) for i in range(4)])
    constraints = [F[j] - F_req[j] for j in range(3)]
    Tau = sum([ca.cross(corners[i], t[i]*s(theta[i], phi[i])) for i in range(4)])
    cost = mu*ca.dot(Tau,Tau) + lam*ca.dot(t,t)

    nlp = {"x": x, "f": cost, "g": ca.vertcat(*constraints)}
    solver = ca.nlpsol("solver", "ipopt", nlp, {
        "ipopt": {
            "print_level": 0,             # 0 = silence, 5 = full log
            "sb": "yes"                   # suppress banner
        },
        "print_time": 0,                  # disable timing prints
    })


    x0 = np.zeros(12)
    x0[:4] = 5.0
    x0[4:8] = np.pi/2
    x0[8:12] = np.pi/2

    sol = solver(x0=x0, lbg=0, ubg=0)
    x_opt = np.array(sol["x"]).flatten()

    t_opt = x_opt[:4].tolist()
    theta_opt = x_opt[4:8].tolist()
    phi_opt   = x_opt[8:12].tolist()
    return t_opt, theta_opt, phi_opt


# F_req = np.array([20,20,19.62])
# corners = [np.array([-1, -1, 0.5]),
#            np.array([ 1, -1, 0.5]),
#            np.array([ 1,  1, 0.5]),
#            np.array([-1,  1, 0.5])]

# t_opt, theta_opt, phi_opt = cable_opt(F_req, corners)
# print("Tensions:", t_opt)
# print("Theta:", theta_opt)
# print("Phi:", phi_opt)
