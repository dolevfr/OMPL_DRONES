#ifndef ACADOS_PYTHON_BRIDGE_H
#define ACADOS_PYTHON_BRIDGE_H

#include "AcadosOneDrone.h"

namespace py = pybind11;

/**
 * @brief A simple C++ wrapper that embeds the Python interpreter
 *        and calls your Acados OCP solver logic defined in Python.
 */
class AcadosPythonBridge
{
public:
    /**
     * @brief Initializes the Python interpreter and loads your solver.
     *        Expects solve_one_step.py (or equivalent) on Python path.
     */
    AcadosPythonBridge();

    ~AcadosPythonBridge();

    /**
     * @brief Solve for the first control u0, given state x0 and accel a_des.
     *
     * @param x0     Current state vector (length NX_)
     * @param a_des  Desired acceleration vector (length 3)
     * @return       Control vector u0 (length NU_)
     */
    Eigen::VectorXd solve(const Eigen::VectorXd &x0,
                          const Eigen::Vector3d &a_des);

private:
    // The scoped_interpreter keeps Python alive for the lifetime of the program.
    static py::scoped_interpreter guard_;

    // The Python solver object, returned by your get_solver() function.
    py::object solver_;
};

#endif // ACADOS_PYTHON_BRIDGE_H
