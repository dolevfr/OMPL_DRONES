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

    // Print state space structure for debugging
    std::cout << "StateSpace Subspaces:\n";
    auto *compoundSpace = stateSpace->as<base::CompoundStateSpace>();
    for (size_t i = 0; i < compoundSpace->getSubspaceCount(); ++i)
    {
        std::cout << "Subspace " << i << ": Dimension = " 
                  << compoundSpace->getSubspace(i)->getDimension() << "\n";
    }

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


ompl::base::ScopedState<> ompl::app::PayloadSystem::getFullStateFromGeometricComponent(
    const base::ScopedState<> &state) const
{
    base::ScopedState<> fullState(getStateSpace());
    std::vector<double> reals = state.reals();

    fullState = 0.0;
    for (size_t i = 0; i < reals.size(); ++i)
    {
        fullState[i] = reals[i];  // Copy values to the full state
    }
    return fullState;
}

std::vector<double> convertOmplToPythonModel(const std::vector<double> &omplState,
                                             const Eigen::Vector3d &payloadDimensions)
{
    constexpr unsigned int droneCount_ = 2; // Fixed number of drones
    std::vector<double> pythonModelState;

    // Extract payload position and quaternion
    Eigen::Vector3d payloadPos(omplState[0], omplState[1], omplState[2]);
    Eigen::Quaterniond payloadQuat(omplState[3], omplState[4], omplState[5], omplState[6]);
    payloadQuat.normalize();

    // Add payload position and quaternion to Python state
    pythonModelState.insert(pythonModelState.end(), {payloadPos.x(), payloadPos.y(), payloadPos.z()});
    pythonModelState.insert(pythonModelState.end(), {payloadQuat.w(), payloadQuat.x(), payloadQuat.y(), payloadQuat.z()});

    // Calculate payload endpoints
    Eigen::Vector3d offset = (payloadDimensions.x() / 2) *
                             payloadQuat.toRotationMatrix() * Eigen::Vector3d(1, 0, 0);
    Eigen::Vector3d r1 = payloadPos + offset;
    Eigen::Vector3d r2 = payloadPos - offset;

    // Loop over drones to extract positions, quaternions, and compute cable quaternions
    unsigned int baseIndex = 7; // Start of drone data in OMPL state
    std::vector<Eigen::Quaterniond> droneQuats(droneCount_);
    std::vector<Eigen::Quaterniond> cableQuats(droneCount_);
    std::vector<Eigen::Vector3d> angularVelocities(droneCount_);

    for (unsigned int i = 0; i < droneCount_; ++i)
    {
        unsigned int droneIndex = baseIndex + i * 13;

        // Extract drone position and quaternion
        Eigen::Vector3d dronePos(omplState[droneIndex + 0],
                                 omplState[droneIndex + 1],
                                 omplState[droneIndex + 2]);

        Eigen::Quaterniond droneQuat(omplState[droneIndex + 3],
                                     omplState[droneIndex + 4],
                                     omplState[droneIndex + 5],
                                     omplState[droneIndex + 6]);
        droneQuat.normalize();

        // Calculate cable quaternion based on endpoints
        Eigen::Vector3d cableVec = (i == 0 ? r1 : r2) - dronePos;
        Eigen::Quaterniond cableQuat(0, cableVec.x(), cableVec.y(), cableVec.z());

        // Extract angular velocity
        Eigen::Vector3d angularVel(omplState[droneIndex + 10],
                                   omplState[droneIndex + 11],
                                   omplState[droneIndex + 12]);

        // Store values
        droneQuats[i] = droneQuat;
        cableQuats[i] = cableQuat;
        angularVelocities[i] = angularVel;

        // Add drone and cable quaternions to Python state
        pythonModelState.insert(pythonModelState.end(), {droneQuat.w(), droneQuat.x(), droneQuat.y(), droneQuat.z()});
        pythonModelState.insert(pythonModelState.end(), {cableQuat.w(), cableQuat.x(), cableQuat.y(), cableQuat.z()});
    }    

    // Helper function for quaternion derivatives
    auto calculateQuaternionDerivative = [](const Eigen::Quaterniond &quat, const Eigen::Vector3d &angularVel) {
        Eigen::Matrix4d E;
        E << -quat.x(), quat.w(), -quat.z(), quat.y(),
             -quat.y(), quat.z(), quat.w(), -quat.x(),
             -quat.z(), -quat.y(), quat.x(), quat.w();
        return 0.5 * E.transpose() * Eigen::Vector4d(angularVel.x(), angularVel.y(), angularVel.z(), 0.0);
    };

    // Insert payload linear velocity after drone and cable quaternions
    Eigen::Vector3d payloadLinearVel(omplState[baseIndex + droneCount_ * 13 + 0],
                                     omplState[baseIndex + droneCount_ * 13 + 1],
                                     omplState[baseIndex + droneCount_ * 13 + 2]);
    pythonModelState.insert(pythonModelState.end(), {payloadLinearVel.x(), payloadLinearVel.y(), payloadLinearVel.z()});

    // Base index for payload angular velocities
    unsigned int payloadAngularVelIndex = baseIndex + droneCount_ * 13 + 3;

    // Payload quaternion derivative
    Eigen::Vector4d payloadQuatDot = calculateQuaternionDerivative(
        payloadQuat,
        Eigen::Vector3d(omplState[payloadAngularVelIndex + 0],
                        omplState[payloadAngularVelIndex + 1],
                        omplState[payloadAngularVelIndex + 2]));
    pythonModelState.insert(pythonModelState.end(), payloadQuatDot.data(), payloadQuatDot.data() + 4);

    // Drone and cable quaternion derivatives
    for (unsigned int i = 0; i < droneCount_; ++i)
    {
        Eigen::Vector4d droneQuatDot = calculateQuaternionDerivative(droneQuats[i], angularVelocities[i]);

        pythonModelState.insert(pythonModelState.end(), droneQuatDot.data(), droneQuatDot.data() + 4);

        // Linear velocity of the payload
        Eigen::Vector3d payloadLinearVel(
            omplState[payloadAngularVelIndex + 0],
            omplState[payloadAngularVelIndex + 1],
            omplState[payloadAngularVelIndex + 2]);

        // Angular velocity of the payload
        Eigen::Vector3d payloadAngularVel(
            omplState[payloadAngularVelIndex + 3],
            omplState[payloadAngularVelIndex + 4],
            omplState[payloadAngularVelIndex + 5]);

        // Distance vector from payload center to r1 or r2
        Eigen::Vector3d distanceToEndpoint = (i == 0 ? r1 : r2) - payloadPos;

        // Linear velocity of the payload endpoint (r1 or r2)
        Eigen::Vector3d endpointLinearVel = payloadLinearVel + payloadAngularVel.cross(distanceToEndpoint);

        // Linear velocity of the drone
        unsigned int droneVelIndex = baseIndex + i * 13 + 7; // Drone velocity starts here
        Eigen::Vector3d droneLinearVel(
            omplState[droneVelIndex + 0],
            omplState[droneVelIndex + 1],
            omplState[droneVelIndex + 2]);

        // Angular velocity of the cable (difference in linear velocities divided by distance)
        Eigen::Vector3d cableAngularVel = (droneLinearVel - endpointLinearVel).cross(cableQuats[i].vec());

        // Calculate quaternion derivative for the cable
        Eigen::Vector4d cableQuatDot = calculateQuaternionDerivative(cableQuats[i], cableAngularVel);

        // Insert the cable quaternion derivative into the Python state
        pythonModelState.insert(pythonModelState.end(), cableQuatDot.data(), cableQuatDot.data() + 4);
    }

    return pythonModelState;
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





const ompl::base::State* ompl::app::PayloadSystem::getGeometricComponentStateInternal(const base::State* state, unsigned int index) const
{
    const base::CompoundState* compoundState = state->as<base::CompoundState>();
    return compoundState->components[index * 2];
}


ompl::base::StateSpacePtr ompl::app::PayloadSystem::constructStateSpace()
{
    auto stateSpace = std::make_shared<base::CompoundStateSpace>();

    // Add SE3 state space for the payload (position and orientation)
    stateSpace->addSubspace(std::make_shared<base::SE3StateSpace>(), 1.0);

    // Add RealVector state space for the payload's velocity (6 dimensions)
    stateSpace->addSubspace(std::make_shared<base::RealVectorStateSpace>(6), 0.3);

    for (unsigned int i = 0; i < droneCount_; ++i)
    {
        // Add SE3 state space for position and orientation
        stateSpace->addSubspace(std::make_shared<base::SE3StateSpace>(), 1.0);

        // Add RealVector state space for velocity (6 dimensions)
        stateSpace->addSubspace(std::make_shared<base::RealVectorStateSpace>(6), 0.3);
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
        controlBounds.setLow(4 * i, 0);    // Thrust lower bound
        controlBounds.setHigh(4 * i, 20); // Thrust upper bound

        double maxTorque = 5; // Maximum torque

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
    base::RealVectorBounds velBounds(6);
    velBounds.setLow(-1);  // Set valid lower bounds
    velBounds.setHigh(1);  // Set valid upper bounds

    base::RealVectorBounds posBounds(3);
    posBounds.setLow(-200); // Set lower bounds for position
    posBounds.setHigh(500); // Set upper bounds for position

    for (unsigned int i = 0; i < droneCount_; ++i)
    {
        // Set bounds for SE3 subspace (position)
        getStateSpace()->as<base::CompoundStateSpace>()
            ->as<base::SE3StateSpace>(i * 2)
            ->setBounds(posBounds);

        // Set bounds for RealVector subspace (velocity)
        getStateSpace()->as<base::CompoundStateSpace>()
            ->as<base::RealVectorStateSpace>(i * 2 + 1)
            ->setBounds(velBounds);
    }

    // Set bounds for payload SE3 subspace (position)
    getStateSpace()->as<base::CompoundStateSpace>()
        ->as<base::SE3StateSpace>(droneCount_ * 2)
        ->setBounds(posBounds);

    // Set bounds for payload RealVector subspace (velocity)
    getStateSpace()->as<base::CompoundStateSpace>()
        ->as<base::RealVectorStateSpace>(droneCount_ * 2 + 1)
        ->setBounds(velBounds);
}
