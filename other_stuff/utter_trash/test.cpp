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

        std::cout << "Python drone model imported successfully!" << std::endl;

        // Define the state vector (x_k)
        Eigen::VectorXd x_k(35);  // Updated state vector: payload + drones + cables + derivatives
        x_k << 0.0, 0.0, 0.0,  // Payload position (x_p, y_p, z_p)
               0.0, 0.0, 0.0, 1.0,  // Payload orientation quaternion (q_w, q_x, q_y, q_z)
               0.0, 0.0, 0.0, 1.0,  // Drone 1 quaternion (q_d1_w, q_d1_x, q_d1_y, q_d1_z)
               0.0, 0.0, // Cable 1 angles (theta_c1, phi_c1)
               0.0, 0.0, 0.0, 1.0,  // Drone 2 quaternion (q_d2_w, q_d2_x, q_d2_y, q_d2_z)
               0.0, 0.0, // Cable 2 angles (theta_c2, phi_c2)
               0.0, 0.0, 0.0,  // Payload linear velocity (dx_p, dy_p, dz_p)
               0.0, 0.0, 0.0,  // Payload angular velocity (omega_x, omega_y, omega_z)
               0.0, 0.0, 0.0,  // Drone 1 angular velocity (omega_d1_x, omega_d1_y, omega_d1_z)
               0.0, 0.0, // Cable 1 angle derivatives (dtheta_c1, dphi_c1)
               0.0, 0.0, 0.0,  // Drone 2 angular velocity (omega_d2_x, omega_d2_y, omega_d2_z)
               0.0, 0.0; // Cable 2 angle derivatives (dtheta_c2, dphi_c2)

        // Define the control input vector (u_k)
        Eigen::VectorXd u_k(8);  // Control vector: thrusts + torques
        u_k << 100.0, 0.0, 0.0, 0.0,  // Drone 1 thrust and torques (T_d1, tau_d1_x, tau_d1_y, tau_d1_z)
               100.0, 0.0, 0.0, 0.0;  // Drone 2 thrust and torques (T_d2, tau_d2_x, tau_d2_y, tau_d2_z)

        // Define the timestep dt
        double dt = 0.01;

        std::cout << "Starting state propagation..." << std::endl;

        // Propagate the state for 10 steps
        for (int i = 0; i < 10; ++i) {

            // Output the current state
            std::cout << "Step " << i + 1 << ":" << std::endl;
            std::cout << "Payload position: " << x_k.segment<3>(0).transpose() << std::endl;
            std::cout << "Payload orientation quaternion: " << x_k.segment<4>(3).transpose() << std::endl;
            std::cout << "Drone 1 quaternion: " << x_k.segment<4>(7).transpose() << std::endl;
            std::cout << "Cable 1 angles: " << x_k.segment<2>(11).transpose() << std::endl;
            std::cout << "Drone 2 quaternion: " << x_k.segment<4>(13).transpose() << std::endl;
            std::cout << "Cable 2 angles: " << x_k.segment<2>(17).transpose() << std::endl;
            std::cout << "----------------------------------------" << std::endl;
            // std::cout << "Payload position derivatives: " << x_k.segment<3>(19).transpose() << std::endl;
            // std::cout << "Payload angular velocity: " << x_k.segment<3>(22).transpose() << std::endl;
            // std::cout << "Drone 1 angular velocity: " << x_k.segment<3>(25).transpose() << std::endl;
            // std::cout << "Cable 1 angle derivatives: " << x_k.segment<2>(28).transpose() << std::endl;
            // std::cout << "Drone 2 angular velocity: " << x_k.segment<3>(30).transpose() << std::endl;
            // std::cout << "Cable 2 angle derivatives: " << x_k.segment<2>(33).transpose() << std::endl;
            // std::cout << "----------------------------------------" << std::endl;

            // Call state_transition
            x_k = state_transition(x_k, u_k, droneModel, dt);
        }
    } catch (const py::error_already_set &e) {
        std::cerr << "Python error: " << e.what() << std::endl;
    }

    return 0;
}
