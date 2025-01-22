#include "stateTransition.h"
#include <pybind11/numpy.h>
#include <iostream>
#include <Eigen/Dense>

Eigen::VectorXd state_transition(const Eigen::VectorXd &x_k,
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

    // std::cout << "Payload position accelerations: " << second_derivatives.segment<3>(0).transpose() << std::endl;
    // std::cout << "Payload angular accelerations: " << second_derivatives.segment<3>(3).transpose() << std::endl;
    // std::cout << "Drone 1 angular accelerations: " << second_derivatives.segment<3>(6).transpose() << std::endl;
    // std::cout << "Cable 1 angle accelerations: " << second_derivatives.segment<2>(9).transpose() << std::endl;
    // std::cout << "Drone 2 angular accelerations: " << second_derivatives.segment<3>(11).transpose() << std::endl;
    // std::cout << "Cable 2 angle accelerations: " << second_derivatives.segment<2>(14).transpose() << std::endl;
    // std::cout << "----------------------------------------" << std::endl;
    // std::cout << "----------------------------------------" << std::endl;

    // Initialize new state vector
    Eigen::VectorXd new_state(x_k.size());

    // Update the last 16 values linearly using second derivatives
    new_state.tail(16) = x_k.tail(16) + second_derivatives * dt;

    new_state[0] = x_k[0] + x_k[0 + 19] * dt;
    new_state[1] = x_k[1] + x_k[1 + 19] * dt;
    new_state[2] = x_k[2] + x_k[2 + 19] * dt;
    new_state[11] = x_k[11] + x_k[11 + 17] * dt;
    new_state[12] = x_k[12] + x_k[12 + 17] * dt;
    new_state[17] = x_k[17] + x_k[17 + 16] * dt;
    new_state[18] = x_k[18] + x_k[18 + 16] * dt;

    // Update quaternion states: 3-6, 7-10, 13-16
    std::vector<int> quaternion_indices = {3, 7, 13};
    int counter = 0; // Counter for the number of quaternions passed
    for (int i : quaternion_indices) {
        Eigen::Vector4d q_old = x_k.segment<4>(i);  // Extract quaternion from x_k
        Eigen::Vector3d omega = x_k.segment<3>(i + 19 - counter);  // Corresponding angular velocity

        // Normalize the old quaternion
        q_old.normalize();

        // Compute the E matrix
        Eigen::MatrixXd E(3, 4);
        E << q_old[0], -q_old[3], q_old[2], -q_old[1],
             q_old[3], q_old[0], -q_old[1], -q_old[2],
            -q_old[2], q_old[1], q_old[0], -q_old[3];

        // Compute q_dot = 0.5 * E^T * omega
        Eigen::Vector4d q_dot = 0.5 * E.transpose() * omega; // [q_dot_w, q_dot_x, q_dot_y, q_dot_z]

        // Update quaternion
        Eigen::Vector4d q_new = q_old + q_dot * dt;

        // Normalize the updated quaternion
        q_new.normalize();

        // Assign back to the state
        new_state.segment<4>(i) = q_new;

        // Increment the counter
        counter++;
    }

    return new_state;
}
