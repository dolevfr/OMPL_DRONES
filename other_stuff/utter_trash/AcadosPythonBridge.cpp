#include "AcadosOneDrone.h"

pybind11::scoped_interpreter AcadosPythonBridge::guard_;  // start Python

AcadosPythonBridge::AcadosPythonBridge()
{
    // add your Python path so solve_one_step.py can be found
    // when running from build/, the modules live at ../src/python
    py::module::import("sys")
        .attr("path")
        .attr("append")("../src/python");
    // import the factory function
    solver_ = py::module::import("solve_one_step")
                  .attr("get_solver")();
}

AcadosPythonBridge::~AcadosPythonBridge() = default;

Eigen::VectorXd AcadosPythonBridge::solve(const Eigen::VectorXd &x0,
                                          const Eigen::Vector3d &a_des)
{
    // build a 1-D numpy array for x0
    py::array_t<double> x0_py(x0.size(), x0.data());

    // build a 1-D numpy array for a_des
    double a_arr[3] = { a_des.x(), a_des.y(), a_des.z() };
    py::array_t<double> a_py(3, a_arr);

    // call Python: solver.solve(x0, a_des)
    py::array_t<double> u0_py = solver_.attr("solve")(x0_py, a_py);

    // copy back to Eigen
    auto buf = u0_py.request();
    double *ptr = static_cast<double*>(buf.ptr);
    Eigen::VectorXd u0(buf.size);
    std::memcpy(u0.data(), ptr, sizeof(double) * buf.size);
    return u0;
}
