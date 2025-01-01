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

#include "PayloadFourDrones.h"
#include <boost/numeric/odeint.hpp>
#include <boost/numeric/odeint/stepper/runge_kutta_cash_karp54.hpp>
#include <boost/numeric/odeint/stepper/controlled_runge_kutta.hpp>




unsigned int ompl::app::PayloadSystem::droneCount_ = 4; // Default number of drones


using StepperType = boost::numeric::odeint::runge_kutta_cash_karp54<std::vector<double>>;

ompl::app::PayloadSystem::PayloadSystem()
    : AppBase<AppType::CONTROL>(constructControlSpace(), Motion_3D), iterationNumber_(0)
{
    // Set the stepper for the ODE solver
    odeSolver = std::make_shared<ompl::control::ODEAdaptiveSolver<StepperType>>(
        si_,
        [this](const ompl::control::ODESolver::StateType &q, const ompl::control::Control *ctrl, ompl::control::ODESolver::StateType &qdot) {
            ode(q, ctrl, qdot);
        },
        1.0e-2 // Integration step size
    );

    name_ = std::string("PayloadSystem");
    setDefaultBounds();

    si_->setPropagationStepSize(0.01);
    si_->setMinMaxControlDuration(1, 10);

    si_->setStatePropagator(control::ODESolver::getStatePropagator(odeSolver,
        [this](const base::State* state, const control::Control* control, const double duration, base::State* result)
        {
            postPropagate(state, control, duration, result);
        }));

    // Environment and robot meshes
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

Eigen::Vector3d ompl::app::PayloadSystem::getPayloadCorner(unsigned int i) const
{
    double a = payloadDimensions.x();
    double b = payloadDimensions.y();

    switch (i)
    {
        case 0: return Eigen::Vector3d(-a / 2, -b / 2, 0); // Bottom-left corner
        case 1: return Eigen::Vector3d(a / 2, -b / 2, 0);  // Bottom-right corner
        case 2: return Eigen::Vector3d(a / 2, b / 2, 0);   // Top-right corner
        case 3: return Eigen::Vector3d(-a / 2, b / 2, 0);  // Top-left corner
        default: throw std::out_of_range("Invalid corner index");
    }
}

double ompl::app::PayloadSystem::calculateCableTension(const Eigen::Vector3d &cableVec) const
{
    double cableLen = cableVec.norm();
    double restLength = l; // Rest length of the cable

    // Enforce the inextensibility constraint
    if (cableLen <= restLength)
    {
        return 0.0; // No tension when the cable is slack
    }

    // Compute the Lagrange multiplier (force to maintain constraint)
    double tension = cableStiffness_ * (cableLen - restLength);

    // Clamp the tension to avoid instability
    tension = std::clamp(tension, 0.0, 1e4);

    return tension;
}



void ompl::app::PayloadSystem::ode(const control::ODESolver::StateType &q, const control::Control *ctrl, control::ODESolver::StateType &qdot)
{
    // Initialize qdot
    qdot.resize(q.size(), 0);

    // Access the control values
    const double *u = ctrl->as<ompl::control::RealVectorControlSpace::ControlType>()->values;

    unsigned int droneStateSize = 13; // State size per drone
    unsigned int payloadIndex = droneCount_ * droneStateSize; // Start index for payload

    // Payload position and orientation
    Eigen::Vector3d payloadPos(q[payloadIndex + 0], q[payloadIndex + 1], q[payloadIndex + 2]);
    Eigen::Quaterniond payloadRot(q[payloadIndex + 6], q[payloadIndex + 3], q[payloadIndex + 4], q[payloadIndex + 5]);

    // Normalize payload quaternion
    double norm = payloadRot.norm();
    if (norm > 1e-3 && norm < 1e3)
    {
        payloadRot.normalize();
    }
    else
    {
        OMPL_WARN("Payload quaternion norm is invalid: %f. Resetting to identity.", norm);
        payloadRot = Eigen::Quaterniond::Identity();
    }

    // Initialize payload force and torque
    Eigen::Vector3d payloadForce(0, 0, -m_payload * 9.81); // Gravity force on the payload
    Eigen::Vector3d payloadTorque(0, 0, 0); // Accumulated torque on the payload

    for (unsigned int i = 0; i < droneCount_; ++i)
    {
        unsigned int baseIndex = i * droneStateSize; // Start index for drone i

        // Drone position and velocity
        Eigen::Vector3d dronePos(q[baseIndex + 0], q[baseIndex + 1], q[baseIndex + 2]);
        Eigen::Vector3d droneVel(q[baseIndex + 7], q[baseIndex + 8], q[baseIndex + 9]);

        // Drone orientation and thrust direction
        Eigen::Quaterniond droneRot(q[baseIndex + 6], q[baseIndex + 3], q[baseIndex + 4], q[baseIndex + 5]);

        // Normalize drone quaternion
        norm = droneRot.norm();
        if (norm > 1e-3 && norm < 1e3)
        {
            droneRot.normalize();
        }
        else
        {
            OMPL_WARN("Drone %u quaternion norm is invalid: %f. Resetting to identity.", i, norm);
            droneRot = Eigen::Quaterniond::Identity();
        }

        Eigen::Vector3d thrustDir = droneRot * Eigen::Vector3d(0, 0, 1);
        Eigen::Vector3d thrustForce = std::clamp(u[i * 4], 0.0, 100.0) * thrustDir; // Clamp thrust magnitude

        // Drag force
        Eigen::Vector3d dragForce = -beta_ * droneVel;

        // Compute cable vector and tension
        Eigen::Vector3d cornerPos = payloadPos + payloadRot * getPayloadCorner(i);
        Eigen::Vector3d cableVec = cornerPos - dronePos;

        double cableVecNorm = cableVec.norm();
        if (cableVecNorm > 1e6 || std::isnan(cableVecNorm) || std::isinf(cableVecNorm))
        {
            OMPL_ERROR("Cable vector norm for Drone %u is invalid: %f. Skipping.", i, cableVecNorm);
            continue;
        }

        double tensionMagnitude = (cableVecNorm > 1e-6) ? calculateCableTension(cableVec) : 0.0;
        tensionMagnitude = std::clamp(tensionMagnitude, 0.0, 1e4); // Clamp tension magnitude
        Eigen::Vector3d tensionForce = tensionMagnitude * cableVec.normalized();

        // Add forces to payload
        payloadForce += tensionForce;
        payloadTorque += getPayloadCorner(i).cross(tensionForce);

        // Drone state derivatives
        qdot[baseIndex + 0] = droneVel.x(); // dx = vx
        qdot[baseIndex + 1] = droneVel.y(); // dy = vy
        qdot[baseIndex + 2] = droneVel.z(); // dz = vz

        // Velocity derivatives
        qdot[baseIndex + 7] = (thrustForce.x() + dragForce.x() - tensionForce.x()) / m_drone;
        qdot[baseIndex + 8] = (thrustForce.y() + dragForce.y() - tensionForce.y()) / m_drone;
        qdot[baseIndex + 9] = (thrustForce.z() + dragForce.z() - tensionForce.z()) / m_drone - 9.81;

        // Angular velocity derivatives
        qdot[baseIndex + 10] = std::clamp(u[i * 4 + 1], -10.0, 10.0);
        qdot[baseIndex + 11] = std::clamp(u[i * 4 + 2], -10.0, 10.0);
        qdot[baseIndex + 12] = std::clamp(u[i * 4 + 3], -10.0, 10.0);
    }

    // Clamp payload forces and torques
    payloadForce = payloadForce.cwiseMax(-1e5).cwiseMin(1e5);
    payloadTorque = payloadTorque.cwiseMax(-1e5).cwiseMin(1e5);

    // Payload state derivatives
    qdot[payloadIndex + 0] = q[payloadIndex + 7]; // dx = vx
    qdot[payloadIndex + 1] = q[payloadIndex + 8]; // dy = vy
    qdot[payloadIndex + 2] = q[payloadIndex + 9]; // dz = vz

    // Linear acceleration
    qdot[payloadIndex + 7] = payloadForce.x() / m_payload;
    qdot[payloadIndex + 8] = payloadForce.y() / m_payload;
    qdot[payloadIndex + 9] = payloadForce.z() / m_payload;

    // Angular acceleration
    Eigen::Vector3d angularAccel = inertia_.inverse() * payloadTorque;
    qdot[payloadIndex + 10] = angularAccel.x();
    qdot[payloadIndex + 11] = angularAccel.y();
    qdot[payloadIndex + 12] = angularAccel.z();
}




void ompl::app::PayloadSystem::postPropagate(const base::State* state, const control::Control* control, double duration, base::State* result)
{
    unsigned int payloadIndex = droneCount_ * 2; // SE3 subspace for payload

    auto *payloadState = result->as<base::CompoundState>()->as<base::SE3StateSpace::StateType>(payloadIndex);
    if (!payloadState)
    {
        throw ompl::Exception("Payload state is null or invalid.");
    }

    Eigen::Vector3d payloadPos(payloadState->getX(), payloadState->getY(), payloadState->getZ());
    Eigen::Quaterniond payloadRot(
        payloadState->rotation().w,
        payloadState->rotation().x,
        payloadState->rotation().y,
        payloadState->rotation().z
    );

    double norm = payloadRot.norm();
    if (norm > 1e-3 && norm < 1e3)
    {
        payloadRot.normalize();
    }
    else
    {
        OMPL_ERROR("Payload quaternion norm is invalid: %f. Resetting to identity.", norm);
        payloadRot = Eigen::Quaterniond::Identity();
    }

    for (unsigned int i = 0; i < droneCount_; ++i)
    {
        unsigned int droneIndex = i * 2;

        auto *droneState = result->as<base::CompoundState>()->as<base::SE3StateSpace::StateType>(droneIndex);
        if (!droneState)
        {
            throw ompl::Exception("Drone state is null or invalid.");
        }

        Eigen::Vector3d dronePos(droneState->getX(), droneState->getY(), droneState->getZ());
        Eigen::Quaterniond droneRot(
            droneState->rotation().w,
            droneState->rotation().x,
            droneState->rotation().y,
            droneState->rotation().z
        );

        norm = droneRot.norm();
        if (norm > 1e-3 && norm < 1e3)
        {
            droneRot.normalize();
        }
        else
        {
            OMPL_ERROR("Drone %u quaternion norm is invalid: %f. Resetting to identity.", i, norm);
            droneRot = Eigen::Quaterniond::Identity();
        }

        Eigen::Vector3d cornerPos = payloadPos + payloadRot * getPayloadCorner(i);
        if (cornerPos.norm() > 1e6 || std::isnan(cornerPos.norm()) || std::isinf(cornerPos.norm()))
        {
            OMPL_ERROR("Corner position for Drone %u is invalid. Skipping processing.", i);
            continue;
        }

        Eigen::Vector3d cableVec = cornerPos - dronePos;
        double cableLength = cableVec.norm();

        if (cableLength > 1e3 || std::isnan(cableLength) || std::isinf(cableLength))
        {
            OMPL_ERROR("Cable length for Drone %u is invalid: %f. Resetting to default.", i, cableLength);
            cableLength = l;
            continue;
        }

        OMPL_DEBUG("Drone %u Cable Vector: (%f, %f, %f), Cable Length: %f", 
                   i, cableVec.x(), cableVec.y(), cableVec.z(), cableLength);

        if (std::abs(cableLength - l) > 1e-4)
        {
            OMPL_WARN("Cable length constraint violated for Drone %u. Expected: %f, Actual: %f", i, l, cableLength);
        }
    }
}





const ompl::base::State* ompl::app::PayloadSystem::getGeometricComponentStateInternal(const base::State* state, unsigned int index) const
{
    const base::CompoundState* compoundState = state->as<base::CompoundState>();
    return compoundState->components[index * 2];
}


ompl::base::StateSpacePtr ompl::app::PayloadSystem::constructStateSpace()
{
    auto stateSpace = std::make_shared<base::CompoundStateSpace>();

    for (unsigned int i = 0; i < droneCount_; ++i)
    {
        // Add SE3 state space for position and orientation
        stateSpace->addSubspace(std::make_shared<base::SE3StateSpace>(), 1.0);

        // Add RealVector state space for velocity (6 dimensions)
        stateSpace->addSubspace(std::make_shared<base::RealVectorStateSpace>(6), 0.3);
    }

    // Add SE3 state space for the payload (position and orientation)
    stateSpace->addSubspace(std::make_shared<base::SE3StateSpace>(), 1.0);

    // Add RealVector state space for the payload's velocity (6 dimensions)
    stateSpace->addSubspace(std::make_shared<base::RealVectorStateSpace>(6), 0.3);

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
