#include "AcadosOneDrone.h"

// ────────────────────────────────────────────────────────────────
// convenient aliases coming from the generated header
#define NX_PAYLOAD  PAYLOAD_1DRONES_NX   // 13 + 11·1  = 24
#define NU_PAYLOAD  PAYLOAD_1DRONES_NU   // 4 · 1      = 4
#define N_HORIZON   PAYLOAD_1DRONES_N    // shooting intervals
// ────────────────────────────────────────────────────────────────

/* ctor */
AcadosMPC::AcadosMPC(bool /*verbose*/)
    : capsule_(nullptr, &payload_1drones_acados_free),
      NX_(NX_PAYLOAD),
      NU_(NU_PAYLOAD),
      N_(N_HORIZON)
{
    auto *cap = payload_1drones_acados_create_capsule();
    if (!cap)
        throw std::runtime_error("acados: create_capsule failed");
    capsule_.reset(cap);

    if (payload_1drones_acados_create(capsule_.get()))
        throw std::runtime_error("acados: create_solver failed");
}

/* dtor */
AcadosMPC::~AcadosMPC() = default;

/* no‐op: initial state & params are pushed in the Python helper */
void AcadosMPC::reset(const Eigen::VectorXd &x0_in, const Eigen::Vector3d &a_des)
{
    std::cout << "[AcadosMPC] reset() called with x0 = " << x0_in.transpose()
              << ", a_des = " << a_des.transpose() << std::endl;

    // Explicitly initialize x0 if it contains uninitialized values
    Eigen::VectorXd x0 = x0_in;

    // // Check if x0 contains uninitialized values and reset
    // if (x0.isZero()) {
    //     std::cout << "[AcadosMPC] Warning: x0 contains uninitialized values, resetting." << std::endl;
    //     x0.setZero();  // Zero-initialize all entries
    // }

    // Initialize x0 with valid starting values (e.g., position and orientation)
    if (x0.size() == NX_) {
        x0.setZero();  // Zero-initialize
        x0[0] = 0.0;   // Position, for example (set to 0)
        x0[3] = 1.0;   // Quaternion (identity rotation)
        // Add additional initializations for other states (e.g., velocities)
    } else {
        throw std::runtime_error("Mismatch between x0 dimensions and expected size");
    }

    // Convert x0 to std::vector for Acados interface
    std::vector<double> x0vec(x0.data(), x0.data() + NX_);
}




/* solve and return the first control via Python subprocess */
Eigen::VectorXd AcadosMPC::nextControl(const Eigen::VectorXd &x0,
                                       const Eigen::Vector3d &a_des)
{
    // delegate to our embedded Python solver
    static AcadosPythonBridge pySolver;
    return pySolver.solve(x0, a_des);
}

