/*********************************************************************
* Rice University Software Distribution License
*
* Copyright (c) 2010, Rice University
* All Rights Reserved.
*
* For a full description see the file named LICENSE.
*
*********************************************************************/

/* Author: Mark Moll (adapted for 4 drones) */

#include "PayloadTwoDrones.h"
#include <boost/numeric/odeint.hpp>
#include <boost/numeric/odeint/stepper/runge_kutta_cash_karp54.hpp>
#include <boost/numeric/odeint/stepper/controlled_runge_kutta.hpp>
#include <Eigen/Dense>
#include <vector>

namespace py = pybind11;
namespace fs = std::experimental::filesystem;


unsigned int ompl::app::PayloadSystem::droneCount_ = 2; // Initialize the static member

using StepperType = boost::numeric::odeint::runge_kutta_cash_karp54<std::vector<double>>;

ompl::app::PayloadSystem::PayloadSystem()
    : AppBase<AppType::CONTROL>(constructControlSpace(), Motion_3D), iterationNumber_(0)
{
    py::initialize_interpreter();
    try
    {
        std::string python_path = (fs::current_path().parent_path() / "src" / "python").string();
        py::module::import("sys").attr("path").attr("append")(python_path);
        droneModel_ = py::module::import("system_model").attr("DronePayloadModel")();
    }
    catch (py::error_already_set &e)
    {
        std::cerr << "Python error: " << e.what() << std::endl;
        throw;
    }

    name_ = "PayloadSystem";

    // Verify the state space structure
    auto stateSpace = getStateSpace();
    if (!stateSpace)
    {
        std::cerr << "Error: StateSpace is null!\n";
        throw ompl::Exception("StateSpace initialization failed.");
    }

    auto *compoundSpace = stateSpace->as<base::CompoundStateSpace>();

    // Lock the state space
    compoundSpace->lock();

    // Apply default bounds
    setDefaultBounds();

    // Set up the ODE solver
    odeSolver = std::make_shared<ompl::control::ODEAdaptiveSolver<StepperType>>(
        si_,
        [this](const ompl::control::ODESolver::StateType &q, const ompl::control::Control *ctrl, ompl::control::ODESolver::StateType &qdot) {
            ode(q, ctrl, qdot);
        },
        timeStep_
    );

    si_->setPropagationStepSize(timeStep_);
    si_->setMinMaxControlDuration(1, 10);

    si_->setStatePropagator(control::ODESolver::getStatePropagator(odeSolver));

    setEnvironmentMesh("/usr/local/share/ompl/resources/3D/Twistycool_env.dae");
    setRobotMesh("/usr/local/share/ompl/resources/3D/quadrotor.dae");
}

const ompl::base::State* ompl::app::PayloadSystem::getGeometricComponentStateInternal(const ompl::base::State* state, unsigned int index) const
{
    const auto* compoundState = state->as<ompl::base::CompoundState>();
    if (compoundState == nullptr)
    {
        throw ompl::Exception("State is not a compound state.");
    }
    return compoundState->components[index];
}



ompl::base::ScopedState<> ompl::app::PayloadSystem::getFullStateFromGeometricComponent(
    const base::ScopedState<> &state) const
{
    base::ScopedState<> fullState(getStateSpace());  // Create a full state
    std::vector<double> reals = state.reals();       // Extract geometric component

    // Ensure the full state is zero-initialized
    fullState = 0.0;

    // Map the geometric component to the full state
    for (size_t i = 0; i < reals.size(); ++i)
    {
        fullState[i] = reals[i];  // Copy geometric component to full state
    }

    // Return the reconstructed full state
    return fullState;
}

// void ompl::app::PayloadSystem::inferProblemDefinitionBounds() {}

ompl::base::StateSpacePtr ompl::app::PayloadSystem::constructStateSpace()
{
    auto stateSpace = std::make_shared<ompl::base::CompoundStateSpace>();

    // Payload position and quaternion
    stateSpace->addSubspace(std::make_shared<ompl::base::SE3StateSpace>(), 1.0);  // Position and orientation

    // Drones and cables (quaternions only)
    for (unsigned int i = 0; i < droneCount_; ++i) {
        stateSpace->addSubspace(std::make_shared<ompl::base::SO3StateSpace>(), 1.0);  // Drone i quaternion
        stateSpace->addSubspace(std::make_shared<ompl::base::SO3StateSpace>(), 1.0);  // Cable i quaternion
    }

    // Payload velocities
    stateSpace->addSubspace(std::make_shared<ompl::base::RealVectorStateSpace>(3), 1.0);  // Velocity
    stateSpace->addSubspace(std::make_shared<ompl::base::RealVectorStateSpace>(4), 1.0);  // Quaternion derivatives

    // Drones and cables quaternion derivatives
    for (unsigned int i = 0; i < droneCount_; ++i) {
        stateSpace->addSubspace(std::make_shared<ompl::base::RealVectorStateSpace>(4), 1.0);  // Drone i quaternion deriv
        stateSpace->addSubspace(std::make_shared<ompl::base::RealVectorStateSpace>(4), 1.0);  // Cable i quaternion deriv
    }

    stateSpace->lock(); 
    return stateSpace;
}




ompl::control::ControlSpacePtr ompl::app::PayloadSystem::constructControlSpace()
{
    // Construct a control space with dimensions equal to 4 times the number of drones
    auto controlSpace = std::make_shared<ompl::control::RealVectorControlSpace>(constructStateSpace(), droneCount_ * 4);

    // Define control bounds
    ompl::base::RealVectorBounds controlBounds(droneCount_ * 4); // 4 controls per drone

    // Set bounds for each drone
    for (size_t i = 0; i < droneCount_; ++i)
    {
        // Thrust
        controlBounds.setLow(4 * i, 20);    // Thrust lower bound
        controlBounds.setHigh(4 * i, 20); // Thrust upper bound

        double maxTorque = 1; // Maximum torque

        // Roll torque
        controlBounds.setLow(4 * i + 1, -maxTorque); // Roll torque lower bound
        controlBounds.setHigh(4 * i + 1, maxTorque); // Roll torque upper bound

        // Pitch torque
        controlBounds.setLow(4 * i + 2, -maxTorque); // Pitch torque lower bound
        controlBounds.setHigh(4 * i + 2, maxTorque); // Pitch torque upper bound

        // Yaw torque
        controlBounds.setLow(4 * i + 3, -maxTorque); // Yaw torque lower bound
        controlBounds.setHigh(4 * i + 3, maxTorque); // Yaw torque upper bound
    }

    // Apply bounds to the control space
    controlSpace->as<ompl::control::RealVectorControlSpace>()->setBounds(controlBounds);

    return controlSpace;
}


void ompl::app::PayloadSystem::setDefaultBounds()
{
    std::cout << "Setting default bounds..." << std::endl;
    // Bounds for the payload position
    base::RealVectorBounds payloadBounds(3);

    // Set the bounds for the payload location
    payloadBounds.setLow(-300);  // Minimum position for X, Y, Z
    payloadBounds.setHigh(600);  // Maximum position for X, Y, Z

    // Apply the bounds to the payload's SE3StateSpace position
    getStateSpace()
        ->as<base::CompoundStateSpace>()
        ->as<base::SE3StateSpace>(0)  // Index 0 corresponds to the payload
        ->setBounds(payloadBounds);

    std::cout << "Bounds applied successfully to the payload position." << std::endl;
}







void ompl::app::PayloadSystem::ode(const control::ODESolver::StateType &q,
                                   const control::Control *ctrl,
                                   control::ODESolver::StateType &qdot)
{
    std::cout << "ODE function called." << std::endl;

    // Extract control inputs
    const double *u = ctrl->as<control::RealVectorControlSpace::ControlType>()->values;

    // Zero out qdot
    qdot.resize(q.size(), 0);

    // Map q and u to Eigen for calculations
    Eigen::VectorXd x_k = Eigen::Map<const Eigen::VectorXd>(q.data(), q.size());
    Eigen::VectorXd u_k = Eigen::Map<const Eigen::VectorXd>(u, 4 * droneCount_);

    // Combine state and control into one vector
    Eigen::VectorXd state_and_control(x_k.size() + u_k.size());
    state_and_control.head(x_k.size()) = x_k;
    state_and_control.tail(u_k.size()) = u_k;

    // Pass state and control to Python model
    py::array_t<double> state_and_control_py(state_and_control.size(), state_and_control.data());
    py::array_t<double> A_py = droneModel_.attr("get_A")(state_and_control_py);
    py::array_t<double> B_py = droneModel_.attr("get_B")(state_and_control_py);

    // Convert Python outputs to Eigen
    Eigen::MatrixXd A = Eigen::Map<Eigen::MatrixXd>(A_py.mutable_data(), A_py.shape(0), A_py.shape(1));
    Eigen::VectorXd B = Eigen::Map<Eigen::VectorXd>(B_py.mutable_data(), B_py.size());

    // Solve A * q_ddot = B for q_ddot
    Eigen::VectorXd second_derivatives = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B);

    std::cout << "Second derivatives: " << second_derivatives.transpose() << std::endl;

    // Populate position derivatives (velocity)
    qdot[0] = x_k[23]; // x velocity
    qdot[1] = x_k[24]; // y velocity
    qdot[2] = x_k[25]; // z velocity

    // Populate position second derivatives (acceleration)
    qdot[23] = second_derivatives[0]; // x acceleration
    qdot[24] = second_derivatives[1]; // y acceleration
    qdot[25] = second_derivatives[2]; // z acceleration

    // Handle quaternion derivatives and second derivatives
    for (int i = 0; i < 20; i += 4) { // Loop over payload, drones, and cables
        // Extract current quaternion and its derivative
        Eigen::Vector4d q_current = x_k.segment<4>(3 + i);         // Current quaternion
        Eigen::Vector4d q_dot_current = x_k.segment<4>(10 + 8 * droneCount_ + i);    // Quaternion derivative
        Eigen::Vector4d q_ddot = second_derivatives.segment<4>(3 + i); // Quaternion second derivative

        // Update quaternion derivative
        Eigen::Vector4d q_dot_new = q_dot_current + q_ddot;

        // Populate qdot with updated quaternion derivatives
        qdot[3 + i] = q_dot_new[0];
        qdot[4 + i] = q_dot_new[1];
        qdot[5 + i] = q_dot_new[2];
        qdot[6 + i] = q_dot_new[3];
    }
}

void ompl::app::PayloadSystem::postPropagate(const base::State* /*state*/, const control::Control* /*control*/, const double /*duration*/, base::State* result)
{
    const base::CompoundStateSpace* cs = getStateSpace()->as<base::CompoundStateSpace>();
    const base::SO3StateSpace* SO3 = cs->as<base::SE3StateSpace>(0)->as<base::SO3StateSpace>(1);
    base::CompoundStateSpace::StateType& csState = *result->as<base::CompoundStateSpace::StateType>();
    base::SO3StateSpace::StateType& so3State = csState.as<base::SE3StateSpace::StateType>(0)->rotation();

    // Normalize the quaternion representation for the payload system
    SO3->enforceBounds(&so3State);
    // Enforce velocity bounds
    cs->getSubspace(1)->enforceBounds(csState[1]);
}
