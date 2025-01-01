#include <pybind11/embed.h>
#include <pybind11/pybind11.h>
#include <experimental/filesystem>
#include <iostream>
#include <Eigen/Dense>
#include "stateTransition.h"  // Include the new state_transition header

namespace py = pybind11;
namespace fs = std::experimental::filesystem;

int main() {
    // Initialize Python interpreter
    py::scoped_interpreter guard{};

    try {
        // Get the current working directory and append the relative path to the Python module
        std::string python_path = (fs::current_path().parent_path() / "src" / "python").string();
        py::module::import("sys").attr("path").attr("append")(python_path);

        // Import the Python drone model
        py::object droneModel = py::module::import("system_model").attr("DronePayloadModel")();

        // Define the state vector (x_k)
        Eigen::VectorXd x_k(46);  // Updated state vector: payload + drones + cables + derivatives
        x_k << 0.0, 0.0, 0.0,  // Payload position (x_p, y_p, z_p)
               1.0, 0.0, 0.0, 0.0,  // Payload orientation quaternion (q_w, q_x, q_y, q_z)
               1.0, 0.0, 0.0, 0.0,  // Drone 1 quaternion (q_d1_w, q_d1_x, q_d1_y, q_d1_z)
               1.0, 0.0, 0.0, 0.0,  // Drone 2 quaternion (q_d2_w, q_d2_x, q_d2_y, q_d2_z)
               1.0, 0.0, 0.0, 0.0,  // Cable 1 quaternion (q_c1_w, q_c1_x, q_c1_y, q_c1_z)
               1.0, 0.0, 0.0, 0.0,  // Cable 2 quaternion (q_c2_w, q_c2_x, q_c2_y, q_c2_z)
               0.0, 0.0, 0.0,  // Payload position derivatives (dx_p, dy_p, dz_p)
               0.0, 0.0, 0.0, 0.0,  // Payload orientation quaternion derivatives (dq_w, dq_x, dq_y, dq_z)
               0.0, 0.0, 0.0, 0.0,  // Drone 1 quaternion derivatives (dq_d1_w, dq_d1_x, dq_d1_y, dq_d1_z)
               0.0, 0.0, 0.0, 0.0,  // Drone 2 quaternion derivatives (dq_d2_w, dq_d2_x, dq_d2_y, dq_d2_z)
               0.0, 0.0, 0.0, 0.0,  // Cable 1 quaternion derivatives (dq_c1_w, dq_c1_x, dq_c1_y, dq_c1_z)
               0.0, 0.0, 0.0, 0.0;  // Cable 2 quaternion derivatives (dq_c2_w, dq_c2_x, dq_c2_y, dq_c2_z)

        // Define the control input vector (u_k)
        Eigen::VectorXd u_k(8);  // Control vector: thrusts + torques
        u_k << 100.0, 100.0,  // Thrusts for drones 1 and 2 (T_d1, T_d2)
               0.0, 0.0, 1.0,  // Torques for drone 1 (tau_d1_x, tau_d1_y, tau_d1_z)
               0.0, 0.0, 1.0;  // Torques for drone 2 (tau_d2_x, tau_d2_y, tau_d2_z)

        // Define the current time
        // double t = 0.0;

        // Combine state and control into a single vector
        Eigen::VectorXd state_and_control(x_k.size() + u_k.size());  // State + control + time
        state_and_control << x_k, u_k;

        // Define the timestep dt
        double dt = 0.01;

        // Call state_transition
        Eigen::VectorXd new_state = state_transition(x_k, u_k, droneModel, dt);

        // Output the result
        std::cout << "New state:" << std::endl;
        std::cout << "Payload position: " << new_state.segment<3>(0).transpose() << std::endl;
        std::cout << "Payload orientation quaternion: " << new_state.segment<4>(3).transpose() << std::endl;
        std::cout << "Drone 1 quaternion: " << new_state.segment<4>(7).transpose() << std::endl;
        std::cout << "Drone 2 quaternion: " << new_state.segment<4>(11).transpose() << std::endl;
        std::cout << "Cable 1 quaternion: " << new_state.segment<4>(15).transpose() << std::endl;
        std::cout << "Cable 2 quaternion: " << new_state.segment<4>(19).transpose() << std::endl;
        std::cout << "Payload position derivatives: " << new_state.segment<3>(23).transpose() << std::endl;
        std::cout << "Payload orientation quaternion derivatives: " << new_state.segment<4>(26).transpose() << std::endl;
        std::cout << "Drone 1 quaternion derivatives: " << new_state.segment<4>(30).transpose() << std::endl;
        std::cout << "Drone 2 quaternion derivatives: " << new_state.segment<4>(34).transpose() << std::endl;
        std::cout << "Cable 1 quaternion derivatives: " << new_state.segment<4>(38).transpose() << std::endl;
        std::cout << "Cable 2 quaternion derivatives: " << new_state.segment<4>(42).transpose() << std::endl;

    } catch (const py::error_already_set &e) {
        std::cerr << "Python error: " << e.what() << std::endl;
    }

    return 0;
}
