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

#include "PayloadTwoDrones_old.h"
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






// void ompl::app::PayloadSystem::setPositionState(base::State* state, double x, double y, double z, unsigned int componentIndex)
// {
//     auto* compoundState = state->as<base::CompoundState>();
//     if (!compoundState)
//     {
//         std::cerr << "Error: State is not a CompoundState!" << std::endl;
//         return;
//     }

//     if (componentIndex >= getStateSpace()->as<base::CompoundStateSpace>()->getSubspaceCount())
//     {
//         std::cerr << "Error: componentIndex " << componentIndex << " is out of bounds!" << std::endl;
//         return;
//     }

//     if (auto* positionState = compoundState->components[componentIndex]->as<base::SE3StateSpace::StateType>())
//     {
//         positionState->setX(x);
//         positionState->setY(y);
//         positionState->setZ(z);
//     }
//     else
//     {
//         std::cerr << "Error: Subspace at index " << componentIndex << " is not SE3StateSpace!" << std::endl;
//     }
// }




// void ompl::app::PayloadSystem::setQuaternionState(base::State* state, const Eigen::Quaterniond& rotation, unsigned int componentIndex)
// {
//     auto *compoundState = state->as<base::CompoundState>();
//     if (!compoundState)
//     {
//         std::cerr << "Error: State is not a CompoundState!" << std::endl;
//         return;
//     }

//     // First, try treating it as SO3StateSpace
//     auto *so3State = compoundState->as<base::SO3StateSpace::StateType>(componentIndex);
//     if (so3State)
//     {
//         so3State->x = rotation.x();
//         so3State->y = rotation.y();
//         so3State->z = rotation.z();
//         so3State->w = rotation.w();
//         return;
//     }

//     // If not SO3StateSpace, check RealVectorStateSpace (for quaternion derivatives)
//     auto *quaternionState = compoundState->as<base::RealVectorStateSpace::StateType>(componentIndex);
//     if (quaternionState)
//     {
//         quaternionState->values[0] = rotation.w();
//         quaternionState->values[1] = rotation.x();
//         quaternionState->values[2] = rotation.y();
//         quaternionState->values[3] = rotation.z();
//         return;
//     }

//     std::cerr << "Error: Subspace at index " << componentIndex << " is neither SO3StateSpace nor RealVectorStateSpace!" << std::endl;
// }



// void ompl::app::PayloadSystem::setLinearVelocityState(base::State* state, double vx, double vy, double vz, unsigned int componentIndex)
// {
//     auto* compoundState = state->as<base::CompoundState>();
//     auto* velocityState = compoundState->components[componentIndex]->as<base::RealVectorStateSpace::StateType>();
//     velocityState->values[0] = vx;
//     velocityState->values[1] = vy;
//     velocityState->values[2] = vz;
// }

// void ompl::app::PayloadSystem::setQuaternionDerivativeState(base::State* state, const Eigen::Quaterniond& derivative, unsigned int componentIndex)
// {
//     auto* compoundState = state->as<base::CompoundState>();
//     auto* quaternionDerivativeState = compoundState->components[componentIndex]->as<base::RealVectorStateSpace::StateType>();
//     quaternionDerivativeState->values[0] = derivative.w();
//     quaternionDerivativeState->values[1] = derivative.x();
//     quaternionDerivativeState->values[2] = derivative.y();
//     quaternionDerivativeState->values[3] = derivative.z();
// }

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






// void ompl::app::PayloadSystem::setPayloadState(base::State* state, double x, double y, double z, const Eigen::Quaterniond& rotation)
// {
//     auto *compoundState = state->as<base::CompoundState>();
//     auto *payloadState = compoundState->components[2 * getRobotCount()]->as<base::SE3StateSpace::StateType>();
//     payloadState->setX(x);
//     payloadState->setY(y);
//     payloadState->setZ(z);
//     payloadState->rotation().x = rotation.x();
//     payloadState->rotation().y = rotation.y();
//     payloadState->rotation().z = rotation.z();
//     payloadState->rotation().w = rotation.w();
// }

// void ompl::app::PayloadSystem::setDroneState(base::State* state, unsigned int droneIndex, double x, double y, double z, const Eigen::Quaterniond& rotation)
// {
//     auto *compoundState = state->as<base::CompoundState>();
//     auto *droneState = compoundState->components[2 * droneIndex]->as<base::SE3StateSpace::StateType>();
//     droneState->setX(x);
//     droneState->setY(y);
//     droneState->setZ(z);
//     droneState->rotation().x = rotation.x();
//     droneState->rotation().y = rotation.y();
//     droneState->rotation().z = rotation.z();
//     droneState->rotation().w = rotation.w();
// }

// void ompl::app::PayloadSystem::setPayloadVelocity(base::State *state, double vx, double vy, double vz, double wx, double wy, double wz)
// {
//     auto *compoundState = state->as<base::CompoundState>();
//     auto *payloadVelocity = compoundState->components[2 * droneCount_ + 1]->as<base::RealVectorStateSpace::StateType>();
//     payloadVelocity->values[0] = vx;
//     payloadVelocity->values[1] = vy;
//     payloadVelocity->values[2] = vz;
//     payloadVelocity->values[3] = wx;
//     payloadVelocity->values[4] = wy;
//     payloadVelocity->values[5] = wz;
// }

// void ompl::app::PayloadSystem::setDroneVelocity(base::State *state, unsigned int droneIndex, double vx, double vy, double vz, double wx, double wy, double wz)
// {
//     auto *compoundState = state->as<base::CompoundState>();
//     auto *droneVelocity = compoundState->components[2 * droneIndex + 1]->as<base::RealVectorStateSpace::StateType>();
//     droneVelocity->values[0] = vx;
//     droneVelocity->values[1] = vy;
//     droneVelocity->values[2] = vz;
//     droneVelocity->values[3] = wx;
//     droneVelocity->values[4] = wy;
//     droneVelocity->values[5] = wz;
// }



// Eigen::Vector3d ompl::app::PayloadSystem::getPayloadCorner(unsigned int i) const
// {
//     double a = payloadDimension; // Payload dimension (a)

//     switch (i)
//     {
//         case 0: return Eigen::Vector3d(-a / 2, 0, 0); // Left endpoint
//         case 1: return Eigen::Vector3d(a / 2, 0, 0);  // Right endpoint
//         default: throw std::out_of_range("Invalid corner index");
//     }
// }

// double ompl::app::PayloadSystem::calculateCableTension(const Eigen::Vector3d &cableVec) const
// {
//     double cableLen = cableVec.norm();
//     double restLength = l; // Rest length of the cable

//     // Enforce the inextensibility constraint
//     if (cableLen < restLength)
//     {
//         return 0.0; // No tension when the cable is slack
//     }

//     // Compute the Lagrange multiplier (force to maintain constraint)
//     double tension = cableStiffness_ * (cableLen - restLength);

//     return tension;
// }

// void ompl::app::PayloadSystem::ode(const control::ODESolver::StateType &q, const control::Control *ctrl, control::ODESolver::StateType &qdot)
// {
//     // Initialize qdot
//     qdot.resize(q.size(), 0);

//     Eigen::Matrix3d inertia_ = Eigen::Matrix3d::Zero(); // Initialize to zero
//     inertia_(0, 0) = rodInertia * 0.001;
//     inertia_(1, 1) = rodInertia;
//     inertia_(2, 2) = rodInertia;


//     // Access the control values
//     const double *u = ctrl->as<ompl::control::RealVectorControlSpace::ControlType>()->values;

//     // std::cout << "Iteration " << iterationNumber_++ << std::endl;
//     // for (unsigned int i = 0; i < droneCount_; ++i)
//     // {
//     //     std::cout << "Drone " << i + 1 << " control inputs: ";
//     //     for (unsigned int j = 0; j < 4; ++j)
//     //     {
//     //         std::cout << u[i * 4 + j] << " ";
//     //     }
//     //     std::cout << std::endl;
//     // }

//     unsigned int droneStateSize = 13; // State size per drone
//     unsigned int payloadIndex = droneCount_ * droneStateSize; // Start index for payload

//     // Payload position and orientation
//     Eigen::Vector3d payloadPos(q[payloadIndex + 0], q[payloadIndex + 1], q[payloadIndex + 2]);
//     Eigen::Quaterniond payloadRot(q[payloadIndex + 6], q[payloadIndex + 3], q[payloadIndex + 4], q[payloadIndex + 5]);

//     // Normalize payload quaternion
//     double norm = payloadRot.norm();
//     if (std::abs(norm - 1.0) > 1e-6) // Normalize only if norm deviates significantly
//     {
//         payloadRot.normalize();
//     }
//     else if (norm < 1e-3 || norm > 1e3) // Invalid norms outside practical limits
//     {
//         OMPL_WARN("Payload quaternion norm is invalid: %f. Resetting to identity.", norm);
//         payloadRot = Eigen::Quaterniond::Identity();
//     }


//     // Initialize payload force and torque
//     Eigen::Vector3d payloadForce(0, 0, -m_payload * 9.81); // Gravity force on the payload
//     Eigen::Vector3d payloadTorque(0, 0, 0); // Accumulated torque on the payload

//     for (unsigned int i = 0; i < droneCount_; ++i)
//     {
//         unsigned int baseIndex = i * droneStateSize; // Start index for drone i

//         // Drone position and velocity
//         qdot[baseIndex + 0] = q[baseIndex + 7]; // dx = vx
//         qdot[baseIndex + 1] = q[baseIndex + 8]; // dy = vy
//         qdot[baseIndex + 2] = q[baseIndex + 9]; // dz = vz

//         // Derivative of orientation
//         base::SO3StateSpace::StateType qomegaDrone;
//         qomegaDrone.w = 0;
//         qomegaDrone.x = .5 * q[baseIndex + 10];
//         qomegaDrone.y = .5 * q[baseIndex + 11];
//         qomegaDrone.z = .5 * q[baseIndex + 12];
//         double deltaDrone = q[baseIndex + 3] * qomegaDrone.x + 
//                             q[baseIndex + 4] * qomegaDrone.y + 
//                             q[baseIndex + 5] * qomegaDrone.z;
//         qdot[baseIndex + 3] = qomegaDrone.x - deltaDrone * q[baseIndex + 3];
//         qdot[baseIndex + 4] = qomegaDrone.y - deltaDrone * q[baseIndex + 4];
//         qdot[baseIndex + 5] = qomegaDrone.z - deltaDrone * q[baseIndex + 5];
//         qdot[baseIndex + 6] = qomegaDrone.w - deltaDrone * q[baseIndex + 6];

//         // Compute forces
//         Eigen::Vector3d thrustForce = u[i * 4] * Eigen::Vector3d(
//             2 * (q[baseIndex + 3] * q[baseIndex + 5] - q[baseIndex + 6] * q[baseIndex + 4]),
//             2 * (q[baseIndex + 4] * q[baseIndex + 5] + q[baseIndex + 6] * q[baseIndex + 3]),
//             q[baseIndex + 6] * q[baseIndex + 6] - q[baseIndex + 3] * q[baseIndex + 3] - q[baseIndex + 4] * q[baseIndex + 4] + q[baseIndex + 5] * q[baseIndex + 5]
//         );

//         Eigen::Vector3d dragForce = -beta_ * Eigen::Vector3d(q[baseIndex + 7], q[baseIndex + 8], q[baseIndex + 9]);

//         // Compute cable vector and tension
//         Eigen::Vector3d cableVec = (payloadPos + payloadRot * getPayloadCorner(i)) - Eigen::Vector3d(q[baseIndex + 0], q[baseIndex + 1], q[baseIndex + 2]);
//         // std::cout << "Drone " << i << " cable length: " << cableVec.norm() << std::endl;
//         Eigen::Vector3d tensionForce =  calculateCableTension(cableVec) * cableVec.normalized();

//         // Apply forces to drones
//         Eigen::Vector3d netForceDrone = thrustForce + dragForce - tensionForce;

//         // Update drone velocity derivatives
//         qdot[baseIndex + 7] = netForceDrone.x() / m_drone;
//         qdot[baseIndex + 8] = netForceDrone.y() / m_drone;
//         qdot[baseIndex + 9] = netForceDrone.z() / m_drone - 9.81;

//         // Angular velocity derivatives (apply torques from control inputs)
//         qdot[baseIndex + 10] = u[i * 4 + 1];
//         qdot[baseIndex + 11] = u[i * 4 + 2];
//         qdot[baseIndex + 12] = u[i * 4 + 3];

//         std::cout << "Drone " << i << " angular velocity: [" 
//                 << qdot[baseIndex + 10] << ", " << qdot[baseIndex + 11] << ", " << qdot[baseIndex + 12] << "]" << std::endl;
//         std::cout << "Drone " << i << " quaternion derivative: [" 
//                 << qdot[baseIndex + 3] << ", " << qdot[baseIndex + 4] << ", " << qdot[baseIndex + 5] << ", " << qdot[baseIndex + 6] << "]" << std::endl;


//         // Add tension force to payload
//         payloadForce += tensionForce;
//         payloadTorque += getPayloadCorner(i).cross(tensionForce);

//         // Log forces for debugging
//         std::cout << "Drone " << i << " thrust force: " << thrustForce.transpose() << std::endl;
//         std::cout << "Drone " << i << " tension force: " << tensionForce.transpose() << std::endl;
//         std::cout << "Drone " << i << " net force: " << netForceDrone.transpose() << std::endl;
//     }

//     // Clamp payload forces and torques
//     payloadForce = payloadForce.cwiseMax(-1e3).cwiseMin(1e3);
//     payloadTorque = payloadTorque.cwiseMax(-1e3).cwiseMin(1e3);

//     // Log payload dynamics for debugging
//     std::cout << "Payload total force: " << payloadForce.transpose() << std::endl;
//     std::cout << "Payload torque: " << payloadTorque.transpose() << std::endl;

//     // Payload state derivatives
//     qdot[payloadIndex + 0] = q[payloadIndex + 7]; // dx = vx
//     qdot[payloadIndex + 1] = q[payloadIndex + 8]; // dy = vy
//     qdot[payloadIndex + 2] = q[payloadIndex + 9]; // dz = vz

//     // Linear acceleration
//     qdot[payloadIndex + 7] = payloadForce.x() / m_payload;
//     qdot[payloadIndex + 8] = payloadForce.y() / m_payload;
//     qdot[payloadIndex + 9] = payloadForce.z() / m_payload;

//     // Angular acceleration
//     Eigen::Vector3d angularAccelPayload = inertia_.inverse() * payloadTorque;
//     qdot[payloadIndex + 10] = angularAccelPayload.x();
//     qdot[payloadIndex + 11] = angularAccelPayload.y();
//     qdot[payloadIndex + 12] = angularAccelPayload.z();

//     std::cout << "Payload position: [" 
//             << q[payloadIndex + 0] << ", " 
//             << q[payloadIndex + 1] << ", " 
//             << q[payloadIndex + 2] << "]\n";
//     std::cout << "Payload velocity: [" 
//             << q[payloadIndex + 7] << ", " 
//             << q[payloadIndex + 8] << ", " 
//             << q[payloadIndex + 9] << "]\n";
//     std::cout << "Payload acceleration: [" 
//             << qdot[payloadIndex + 7] << ", " 
//             << qdot[payloadIndex + 8] << ", " 
//             << qdot[payloadIndex + 9] << "]\n";

//     // Log angular acceleration
//     std::cout << "Payload angular acceleration: " << angularAccelPayload.transpose() << std::endl;
// }