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
    // Initialize the Python interpreter and load the drone model
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

    // Set the name of the system
    name_ = "PayloadSystem";

    // Verify the state space structure
    auto stateSpace = getStateSpace();
    if (!stateSpace)
    {
        std::cerr << "Error: StateSpace is null!\n";
        throw ompl::Exception("StateSpace initialization failed.");
    }

    // Apply default bounds
    setDefaultBounds();

    // Use ODEBasicSolver for fixed-step integration
    odeSolver = std::make_shared<ompl::control::ODEBasicSolver<>>(si_, [this](
        const ompl::control::ODESolver::StateType &q,
        const ompl::control::Control *ctrl,
        ompl::control::ODESolver::StateType &qdot) {
        ode(q, ctrl, qdot);
    });

    // Configure propagation step size and control duration
    si_->setPropagationStepSize(0.1);       // Fixed small step size
    si_->setMinMaxControlDuration(1, 3);   // Shorter control duration for efficiency

    // Set the state propagator with a post-propagate callback
    si_->setStatePropagator(control::ODESolver::getStatePropagator(odeSolver,
        [this](const base::State *state, const control::Control *control, const double duration, base::State *result) {
            postPropagate(state, control, duration, result);
        }));

    // Set environment and robot meshes
    setEnvironmentMesh("/usr/local/share/ompl/resources/3D/Twistycool_env.dae");
    setRobotMesh("/usr/local/share/ompl/resources/3D/quadrotor.dae");
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

ompl::base::StateSpacePtr ompl::app::PayloadSystem::constructStateSpace()
{
    auto stateSpace = std::make_shared<ompl::base::CompoundStateSpace>();

    // Payload position and quaternion
    stateSpace->addSubspace(std::make_shared<ompl::base::SE3StateSpace>(), 1.0);

    // Drones and cables (quaternions only)
    for (unsigned int i = 0; i < droneCount_; ++i) {
        stateSpace->addSubspace(std::make_shared<ompl::base::RealVectorStateSpace>(4), 1.0);  // Drone i quaternion
        stateSpace->addSubspace(std::make_shared<ompl::base::RealVectorStateSpace>(4), 1.0);  // Cable i quaternion
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
    std::cout << "Setting default bounds for PayloadSystem." << std::endl;

    // Define bounds
    base::RealVectorBounds posBounds(3); // Bounds for payload position
    posBounds.setLow(-300);
    posBounds.setHigh(600);

    base::RealVectorBounds quatBounds(4); // Bounds for quaternions
    quatBounds.setLow(-1);
    quatBounds.setHigh(1);

    base::RealVectorBounds velBounds(3); // Bounds for velocities
    velBounds.setLow(-1);
    velBounds.setHigh(1);

    base::RealVectorBounds quatDerivBounds(4); // Bounds for quaternion derivatives
    quatDerivBounds.setLow(-1);
    quatDerivBounds.setHigh(1);

    // Access compound state space
    auto *compoundSpace = getStateSpace()->as<base::CompoundStateSpace>();
    if (!compoundSpace)
    {
        std::cerr << "Error: CompoundStateSpace is null!" << std::endl;
        return;
    }

    unsigned int index = 0;

    try
    {
        // Set bounds for payload position and quaternion (SE3StateSpace)
        compoundSpace->getSubspace(index++)->as<base::SE3StateSpace>()->setBounds(posBounds);


        // Set bounds for drones and cables (quaternions only)
        for (unsigned int i = 0; i < droneCount_; ++i)
        {
            compoundSpace->getSubspace(index++)->as<base::RealVectorStateSpace>()->setBounds(quatBounds); // Drone quaternion
            compoundSpace->getSubspace(index++)->as<base::RealVectorStateSpace>()->setBounds(quatBounds); // Cable quaternion
        }

        // Set bounds for payload velocity
        compoundSpace->getSubspace(index++)->as<base::RealVectorStateSpace>()->setBounds(velBounds);

        // Set bounds for payload quaternion derivatives
        compoundSpace->getSubspace(index++)->as<base::RealVectorStateSpace>()->setBounds(quatDerivBounds);

        // Set bounds for drones and cables quaternion derivatives
        for (unsigned int i = 0; i < droneCount_; ++i)
        {
            compoundSpace->getSubspace(index++)->as<base::RealVectorStateSpace>()->setBounds(quatDerivBounds); // Drone quaternion derivative
            compoundSpace->getSubspace(index++)->as<base::RealVectorStateSpace>()->setBounds(quatDerivBounds); // Cable quaternion derivative
        }

        std::cout << "Bounds applied successfully." << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
    }
}


// void ompl::app::PayloadSystem::setDefaultBounds()
// {
//     std::cout << "Skipping bounds setup for debugging." << std::endl;

//     // Access the compound state space
//     auto *compoundSpace = getStateSpace()->as<base::CompoundStateSpace>();
//     if (!compoundSpace)
//     {
//         std::cerr << "Error: CompoundStateSpace is null!" << std::endl;
//         return;
//     }

//     // Iterate over all subspaces and disable bounds
//     for (unsigned int i = 0; i < compoundSpace->getSubspaceCount(); ++i)
//     {
//         auto *subspace = compoundSpace->getSubspace(i)->as<base::RealVectorStateSpace>();
//         if (subspace)
//         {
//             std::cout << "Subspace " << i << " has dimension: " << subspace->getDimension() << std::endl;
//             subspace->setBounds(base::RealVectorBounds(subspace->getDimension()));
//         }
//     }

//     std::cout << "All bounds skipped for debugging." << std::endl;
// }


void ompl::app::PayloadSystem::ode(const control::ODESolver::StateType &q,
                                   const control::Control *ctrl,
                                   control::ODESolver::StateType &qdot)
{

    // Extract control inputs
    const double *u = ctrl->as<control::RealVectorControlSpace::ControlType>()->values;

    // // Debugging: Enforce thrust magnitude of exactly 10 and set other controls to 0
    // for (unsigned int i = 0; i < droneCount_; ++i)
    // {
    //     // Ensure the thrust control (u[i * 4]) is set to 10
    //     double& thrustControl = const_cast<double&>(ctrl->as<ompl::control::RealVectorControlSpace::ControlType>()->values[i * 4]);
    //     thrustControl = 10.0;

    //     // Set roll, pitch, and yaw torques to 0
    //     for (unsigned int j = 1; j < 4; ++j)
    //     {
    //         double& torqueControl = const_cast<double&>(ctrl->as<ompl::control::RealVectorControlSpace::ControlType>()->values[i * 4 + j]);
    //         torqueControl = 0.0;
    //     }
    // }

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

    // Populate qdot: first half with the second half of x_k, and second half with second_derivatives
    unsigned int state_size = x_k.size() / 2;

    for (unsigned int i = 0; i < state_size; ++i)
    {
        qdot[i] = x_k[state_size + i];         // First half of qdot: velocities or quaternion derivatives
        qdot[state_size + i] = second_derivatives[i]; // Second half of qdot: accelerations or quaternion second derivatives
    }

    // // Print the state (q) and its derivative (qdot)
    // std::cout << "State (q): ";
    // for (const auto &val : q)
    // {
    //     std::cout << val << " ";
    // }
    // std::cout << std::endl;

    // std::cout << "State derivative (qdot): ";
    // for (const auto &val : qdot)
    // {
    //     std::cout << val << " ";
    // }
    // std::cout << std::endl;
}


void ompl::app::PayloadSystem::postPropagate(const base::State* /*state*/, const control::Control* /*control*/, const double /*duration*/, base::State* result)
{
    const auto *compoundSpace = getStateSpace()->as<base::CompoundStateSpace>();
    auto &compoundState = *result->as<base::CompoundStateSpace::StateType>();

    // Enforce bounds for payload SE3
    auto *payloadSE3Space = compoundSpace->getSubspace(0)->as<base::SE3StateSpace>();
    auto &payloadSE3State = *compoundState.as<base::SE3StateSpace::StateType>(0);
    payloadSE3Space->enforceBounds(&payloadSE3State);

    unsigned int index = 1; // Start after payload SE3

    // Normalize drone and cable quaternions
    for (unsigned int i = 0; i < droneCount_; ++i)
    {
        auto *droneQuat = compoundSpace->getSubspace(index++)->as<base::RealVectorStateSpace>()->getValueAddressAtIndex(&compoundState, 0);
        Eigen::Map<Eigen::Vector4d>(droneQuat).normalize();

        auto *cableQuat = compoundSpace->getSubspace(index++)->as<base::RealVectorStateSpace>()->getValueAddressAtIndex(&compoundState, 0);
        Eigen::Map<Eigen::Vector4d>(cableQuat).normalize();
    }

    // Print the state
    std::ostringstream stateStream;

    // // Payload position and quaternion
    // stateStream << "Payload Position: ["
    //             << payloadSE3State.getX() << ", " << payloadSE3State.getY() << ", " << payloadSE3State.getZ() << "] ";
    // stateStream << "Quaternion: ["
    //             << payloadSE3State.rotation().x << ", " << payloadSE3State.rotation().y << ", "
    //             << payloadSE3State.rotation().z << ", " << payloadSE3State.rotation().w << "]\n";

    // // Drone and cable quaternions
    // for (unsigned int i = 0; i < droneCount_; ++i)
    // {
    //     auto *droneQuat = compoundState.as<ompl::base::RealVectorStateSpace::StateType>(index++);
    //     auto *cableQuat = compoundState.as<ompl::base::RealVectorStateSpace::StateType>(index++);
    //     stateStream << "Drone " << i << " Quaternion: ["
    //                 << droneQuat->values[0] << ", " << droneQuat->values[1] << ", "
    //                 << droneQuat->values[2] << ", " << droneQuat->values[3] << "] ";
    //     stateStream << "Cable " << i << " Quaternion: ["
    //                 << cableQuat->values[0] << ", " << cableQuat->values[1] << ", "
    //                 << cableQuat->values[2] << ", " << cableQuat->values[3] << "]\n";
    // }

    // std::cout << "Post-Propagation State:\n" << stateStream.str();
}






const ompl::base::State *ompl::app::PayloadSystem::getGeometricComponentStateInternal(const ompl::base::State *state, unsigned int index) const
{
    // Ensure the provided index is valid
    if (index >= getStateSpace()->as<ompl::base::CompoundStateSpace>()->getSubspaceCount())
    {
        throw std::out_of_range("Index out of bounds in getGeometricComponentStateInternal");
    }

    // Cast the state to a CompoundState
    const auto *compoundState = state->as<ompl::base::CompoundState>();
    
    // Return the substate corresponding to the given index
    return compoundState->components[index];
}