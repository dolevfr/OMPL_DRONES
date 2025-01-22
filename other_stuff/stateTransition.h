#ifndef STATE_TRANSITION_H
#define STATE_TRANSITION_H

#include <pybind11/embed.h>
#include <Eigen/Dense>

namespace py = pybind11;

// Define state_transition as a standalone function
Eigen::VectorXd state_transition(const Eigen::VectorXd &x_k,
                                 const Eigen::VectorXd &u_k,
                                 py::object &droneModel,
                                 double dt);

#endif // STATE_TRANSITION_H
