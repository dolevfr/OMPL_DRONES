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
    std::cout << "Setting default bounds for PayloadSystem (velocities and quaternion derivatives)." << std::endl;

    // Bounds for payload velocities (3 dimensions)
    base::RealVectorBounds velBounds(3);
    velBounds.setLow(-1);
    velBounds.setHigh(1);

    // Bounds for quaternion derivatives (4 dimensions)
    base::RealVectorBounds quatDerivBounds(4);
    quatDerivBounds.setLow(-10);
    quatDerivBounds.setHigh(10);

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
        // Skip payload position (SE3StateSpace): no bounds set for position and orientation
        index++; 

        // Skip drone and cable orientations (SO3StateSpace)
        for (unsigned int i = 0; i < droneCount_; ++i)
        {
            index++; // Drone quaternion
            index++; // Cable quaternion
        }

        // Set bounds for payload velocity
        compoundSpace->getSubspace(index++)->as<base::RealVectorStateSpace>()->setBounds(velBounds);

        // Set bounds for payload quaternion derivatives
        compoundSpace->getSubspace(index++)->as<base::RealVectorStateSpace>()->setBounds(quatDerivBounds);

        // Set bounds for drone and cable quaternion derivatives
        for (unsigned int i = 0; i < droneCount_; ++i)
        {
            compoundSpace->getSubspace(index++)->as<base::RealVectorStateSpace>()->setBounds(quatDerivBounds); // Drone deriv
            compoundSpace->getSubspace(index++)->as<base::RealVectorStateSpace>()->setBounds(quatDerivBounds); // Cable deriv
        }

        std::cout << "Bounds applied successfully to velocities and quaternion derivatives." << std::endl;
    }
    catch (const ompl::Exception &e)
    {
        std::cerr << "OMPL Exception: " << e.what() << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Standard Exception: " << e.what() << std::endl;
    }
}





void ompl::app::PayloadSystem::ode(const control::ODESolver::StateType &q,
                                   const control::Control *ctrl,
                                   control::ODESolver::StateType &qdot)
{

    // Extract control inputs
    const double *u = ctrl->as<control::RealVectorControlSpace::ControlType>()->values;

    // Debugging: Enforce thrust magnitude of exactly 10
    for (unsigned int i = 0; i < droneCount_; ++i)
    {
        // Ensure the thrust control (u[i * 4]) is set to 10
        double& thrustControl = const_cast<double&>(ctrl->as<ompl::control::RealVectorControlSpace::ControlType>()->values[i * 4]);
        thrustControl = 10.0;

        // Optionally, print the adjusted thrust for debugging
        // std::cout << "Drone " << i << " thrust set to: " << thrustControl << std::endl;
    }


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
    const auto* compoundStateSpace = getStateSpace()->as<base::CompoundStateSpace>();
    if (!compoundStateSpace)
    {
        std::cerr << "Error: CompoundStateSpace is null.\n";
        return;
    }

    auto* compoundState = result->as<base::CompoundState>();
    if (!compoundState)
    {
        std::cerr << "Error: CompoundState is null.\n";
        return;
    }

    // Normalize the payload quaternion
    auto* payloadSE3State = compoundState->as<base::SE3StateSpace::StateType>(0);
    if (!payloadSE3State)
    {
        std::cerr << "Error: Payload SE3StateSpace is null.\n";
        return;
    }
    Eigen::Quaterniond payloadRot(
        payloadSE3State->rotation().w,
        payloadSE3State->rotation().x,
        payloadSE3State->rotation().y,
        payloadSE3State->rotation().z
    );
    payloadRot.normalize();
    payloadSE3State->rotation().w = payloadRot.w();
    payloadSE3State->rotation().x = payloadRot.x();
    payloadSE3State->rotation().y = payloadRot.y();
    payloadSE3State->rotation().z = payloadRot.z();

    // Enforce bounds on payload velocity
    auto* payloadVelocity = compoundState->as<base::RealVectorStateSpace::StateType>(1 + 2 * droneCount_);
    if (!payloadVelocity)
    {
        std::cerr << "Error: Payload velocity state is null.\n";
        return;
    }
    compoundStateSpace->getSubspace(1 + 2 * droneCount_)->enforceBounds(payloadVelocity);

    // Enforce bounds on payload quaternion derivatives
    auto* payloadQuatDeriv = compoundState->as<base::RealVectorStateSpace::StateType>(2 + 2 * droneCount_);
    if (!payloadQuatDeriv)
    {
        std::cerr << "Error: Payload quaternion derivative state is null.\n";
        return;
    }
    compoundStateSpace->getSubspace(2 + 2 * droneCount_)->enforceBounds(payloadQuatDeriv);

    // Loop through drones and cables
    for (unsigned int i = 0; i < droneCount_; ++i)
    {
        // Normalize drone quaternion
        auto* droneSO3State = compoundState->as<base::SO3StateSpace::StateType>(1 + i * 2);
        if (!droneSO3State)
        {
            std::cerr << "Error: Drone quaternion state is null at index " << i << ".\n";
            return;
        }
        Eigen::Quaterniond droneRot(
            droneSO3State->w,
            droneSO3State->x,
            droneSO3State->y,
            droneSO3State->z
        );
        droneRot.normalize();
        droneSO3State->w = droneRot.w();
        droneSO3State->x = droneRot.x();
        droneSO3State->y = droneRot.y();
        droneSO3State->z = droneRot.z();

        // Normalize cable quaternion
        auto* cableSO3State = compoundState->as<base::SO3StateSpace::StateType>(2 + i * 2);
        if (!cableSO3State)
        {
            std::cerr << "Error: Cable quaternion state is null at index " << i << ".\n";
            return;
        }
        Eigen::Quaterniond cableRot(
            cableSO3State->w,
            cableSO3State->x,
            cableSO3State->y,
            cableSO3State->z
        );
        cableRot.normalize();
        cableSO3State->w = cableRot.w();
        cableSO3State->x = cableRot.x();
        cableSO3State->y = cableRot.y();
        cableSO3State->z = cableRot.z();

        // Enforce bounds on drone quaternion derivatives
        auto* droneQuatDeriv = compoundState->as<base::RealVectorStateSpace::StateType>(3 + 2 * droneCount_ + i * 2);
        if (!droneQuatDeriv)
        {
            std::cerr << "Error: Drone quaternion derivative state is null at index " << i << ".\n";
            return;
        }
        compoundStateSpace->getSubspace(3 + 2 * droneCount_ + i * 2)->enforceBounds(droneQuatDeriv);

        // Enforce bounds on cable quaternion derivatives
        auto* cableQuatDeriv = compoundState->as<base::RealVectorStateSpace::StateType>(4 + 2 * droneCount_ + i * 2);
        if (!cableQuatDeriv)
        {
            std::cerr << "Error: Cable quaternion derivative state is null at index " << i << ".\n";
            return;
        }
        compoundStateSpace->getSubspace(4 + 2 * droneCount_ + i * 2)->enforceBounds(cableQuatDeriv);
    }
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