#include "stateTransition.h"
#include <pybind11/numpy.h>
#include <iostream>
#include <Eigen/Dense>

Eigen::VectorXd state_transition_copy(const Eigen::VectorXd &x_k,
                                 const Eigen::VectorXd &u_k,
                                 py::object &droneModel,
                                 double dt) {

    // Create the combined state and control vector
    Eigen::VectorXd state_and_control(x_k.size() + u_k.size());
    state_and_control.head(x_k.size()) = x_k;
    state_and_control.tail(u_k.size()) = u_k;

    // Convert to py::array
    py::array_t<double> state_and_control_py(state_and_control.size(), state_and_control.data());

    // Pass to get_A and get_B
    py::array_t<double> A_py = droneModel.attr("get_A")(state_and_control_py);
    py::array_t<double> B_py = droneModel.attr("get_B")(state_and_control_py);

    // Convert A and B matrices to Eigen
    Eigen::MatrixXd A = Eigen::Map<Eigen::MatrixXd>(A_py.mutable_data(), A_py.shape(0), A_py.shape(1));
    Eigen::VectorXd B = Eigen::Map<Eigen::VectorXd>(B_py.mutable_data(), B_py.size());

    // Compute least-squares solution for accelerations (second derivatives)
    Eigen::VectorXd second_derivatives = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B);

    // Update the new state vector
    Eigen::VectorXd new_state(x_k.size());

    // First half: Positions updated with velocities
    new_state.head(x_k.size() / 2) = x_k.head(x_k.size() / 2) + x_k.tail(x_k.size() / 2) * dt;

    // Normalize the quaternions at indices 3-6, 7-10, 13-16
    for (int i : {3, 7, 13}) {
        Eigen::Vector4d quaternion = new_state.segment<4>(i);
        quaternion.normalize();
        new_state.segment<4>(i) = quaternion;
    }

    // Second half: Velocities updated with accelerations (second derivatives)
    new_state.tail(x_k.size() / 2) = x_k.tail(x_k.size() / 2) + second_derivatives * dt;

    return new_state;
}

