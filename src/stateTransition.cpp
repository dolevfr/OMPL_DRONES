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

    // Extract second derivatives for positions and quaternions
    Eigen::Vector3d pos_ddot = second_derivatives.segment<3>(0); // x_p_ddot, y_p_ddot, z_p_ddot
    Eigen::VectorXd quaternions_ddot = second_derivatives.segment(3, 20); // Second derivatives of quaternions

    std::cout << "pos_ddot: " << pos_ddot.transpose() << std::endl;

    // Extract current velocities from x_k
    Eigen::Vector3d pos_dot = x_k.segment<3>(23); // x_p_dot, y_p_dot, z_p_dot
    Eigen::VectorXd quaternions_dot = x_k.segment(26, 20); // Derivatives of quaternions

    std::cout << "pos_dot: " << pos_dot.transpose() << std::endl;


    // Update linear velocities
    Eigen::Vector3d pos_dot_new = pos_dot + pos_ddot * dt;

    // Update linear positions
    Eigen::Vector3d pos_new = x_k.segment<3>(0) + pos_dot_new * dt;

    // Update quaternions and their derivatives for all components
    Eigen::VectorXd quaternions_new(20);
    Eigen::VectorXd quaternions_dot_new(20);
    for (int i = 0; i < 20; i += 4) { // Update each quaternion (payload, drones, cables)
        // Extract current quaternion and its derivative
        Eigen::Vector4d q_current = x_k.segment<4>(3 + i);         // Current quaternion
        Eigen::Vector4d q_dot_current = quaternions_dot.segment<4>(i); // Current quaternion derivative
        Eigen::Vector4d q_ddot = quaternions_ddot.segment<4>(i);   // Second derivative of quaternion

        // Update quaternion derivative
        Eigen::Vector4d q_dot_new = q_dot_current + q_ddot * dt;

        // Update quaternion
        Eigen::Vector4d q_new = q_current + q_dot_new * dt;

        // Normalize quaternion to ensure it remains a valid unit quaternion
        Eigen::Quaterniond q_updated(q_new(0), q_new(1), q_new(2), q_new(3));
        q_updated.normalize();

        // Store updated quaternion and derivative
        quaternions_new.segment<4>(i) = q_updated.coeffs(); // Updated quaternion
        quaternions_dot_new.segment<4>(i) = q_dot_new;      // Updated quaternion derivative
    }

    // Construct the new state vector
    Eigen::VectorXd new_state(x_k.size());
    new_state.segment<3>(0) = pos_new;                       // Updated position
    new_state.segment(3, 20) = quaternions_new;              // Updated quaternions
    new_state.segment<3>(23) = pos_dot_new;                  // Updated linear velocity
    new_state.segment(26, 20) = quaternions_dot_new;         // Updated quaternion derivatives

    return new_state;
}

