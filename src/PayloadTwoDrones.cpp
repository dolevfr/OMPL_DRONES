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




unsigned int ompl::app::PayloadSystem::droneCount_ = 2; // Default number of drones


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
        // Add SO3 state space for  orientation
        stateSpace->addSubspace(std::make_shared<base::SO3StateSpace>(), 1.0);

        // Add RealVector state space for velocity (3 dimensions)
        stateSpace->addSubspace(std::make_shared<base::RealVectorStateSpace>(3), 0.3);

        // Add RealVector state space for cable angles and velocities (4 dimensions)
        stateSpace->addSubspace(std::make_shared<base::RealVectorStateSpace>(4), 0.1);
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

void ompl::app::PayloadSystem::ode(const control::ODESolver::StateType &q, const control::Control *ctrl, control::ODESolver::StateType &qdot)
{
    // Initialize qdot
    qdot.resize(q.size(), 0);

    // Access the control values
    const double *u = ctrl->as<ompl::control::RealVectorControlSpace::ControlType>()->values;

    unsigned int droneStateSize = 11; // State size per drone and cable
    unsigned int droneIndex = 13; // Start index for drones

    // Payload position and orientation
    Eigen::Vector3d payloadPos(q[0], q[1], q[2]);
    Eigen::Quaterniond payloadRot(q[4], q[5], q[6], q[3]);
    Eigen::Vector3d payloadVel(q[7], q[8], q[9]);
    Eigen::Vector3d payloadAngVel(q[10], q[11], q[12]);

    // Initialize payload force and torque
    Eigen::Vector3d payloadForce(0, 0, -m_payload * 9.81); // Gravity force on the payload
    Eigen::Vector3d payloadTorque(0, 0, 0); // Accumulated torque on the payload

    for (unsigned int i = 0; i < droneCount_; ++i)
    {
        unsigned int baseIndex = droneIndex + i * droneStateSize; // Start index for drone i

        // Drone orientation and angular velocity
        Eigen::Quaterniond droneRot(q[baseIndex + 3], q[baseIndex + 0], q[baseIndex + 1], q[baseIndex + 2]);
        Eigen::Vector3d omega(q[baseIndex + 4], q[baseIndex + 5], q[baseIndex + 6]);

        // Cables angles and velocities
        double theta = q[baseIndex + 7];
        double phi = q[baseIndex + 8];
        double thetaDot = q[baseIndex + 9];
        double phiDot = q[baseIndex + 10];

        // Trust direction
        Eigen::Vector3d thrustDir = droneRot * Eigen::Vector3d(0, 0, 1);

        // Retrieve thrust magnitude from control input
        double thrustMagnitude = u[i * 4];

        Eigen::Matrix3d spherical;
        spherical << sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta),
                     cos(theta) * cos(phi), cos(theta) * sin(phi), -sin(theta),
                     -sin(phi), cos(phi), 0;

        // Cable unit vectors
        Eigen::Vector3d cableDir = spherical * Eigen::Vector3d(1, 0, 0);
        Eigen::Vector3d thetaDir = spherical * Eigen::Vector3d(0, 1, 0);
        Eigen::Vector3d phiDir = spherical * Eigen::Vector3d(0, 0, 1);


        Eigen::Vector3d thrustOnCable = thrustMagnitude * cableDir;
        Eigen::Vector3d thrustTheta = thrustMagnitude * thetaDir;
        Eigen::Vector3d thrustPhi = thrustMagnitude * phiDir;

        // Construct Omega(omega) matrix
        Eigen::Matrix4d Omega;
        Omega <<  0,       -omega.x(), -omega.y(), -omega.z(),
                omega.x(),     0,       omega.z(), -omega.y(),
                omega.y(), -omega.z(),     0,      omega.x(),
                omega.z(),  omega.y(), -omega.x(),    0;

        // Compute quaternion derivative
        Eigen::Vector4d quatDot = 0.5 * Omega * droneRot;

        // Angular acceleration calculation
        Eigen::Vector3d torque(u[i * 4 + 1], u[i * 4 + 2], u[i * 4 + 3]); // Control inputs for torques
        Eigen::Vector3d angularAccel = droneInertia.inverse() * torque;

        // Update qdot for quaternion
        qdot[baseIndex + 0] = quatDot.x();
        qdot[baseIndex + 1] = quatDot.y();
        qdot[baseIndex + 2] = quatDot.z();
        qdot[baseIndex + 3] = quatDot.w();

        // Update qdot for angular accelerations
        qdot[baseIndex + 4] = angularAccel.x();
        qdot[baseIndex + 5] = angularAccel.y();
        qdot[baseIndex + 6] = angularAccel.z();

        // Update angular velocities of theta and phi
        qdot[baseIndex + 7] = thetaDot;
        qdot[baseIndex + 8] = phiDot;
        qdot[baseIndex + 9] = thrustTheta.norm() / l;
        qdot[baseIndex + 10] = thrustPhi.norm() / l;

        // Calculate the rotated x-direction of the payload
        Eigen::Vector3d payloadXDir = payloadRot * Eigen::Vector3d(1, 0, 0); // x-direction in the payload's local frame

        // Update forces and torques
        payloadForce += thrustOnCable; // Thrust force on the payload
        payloadTorque += (0.5 * payloadDimension * payloadXDir).cross(thrustOnCable);
    }

    Eigen::Matrix4d Omega;
    Omega <<  0,       -payloadAngVel.x(), -payloadAngVel.y(), -payloadAngVel.z(),
        payloadAngVel.x(),     0,       payloadAngVel.z(), -payloadAngVel.y(),
        payloadAngVel.y(), -payloadAngVel.z(),     0,      payloadAngVel.x(),
        payloadAngVel.z(),  payloadAngVel.y(), -payloadAngVel.x(),    0;

    // Compute quaternion derivative
    Eigen::Vector4d quatDot = 0.5 * Omega * payloadRot;

    // Payload linear and angular acceleration
    Eigen::Vector3d payloadAccel = payloadForce / m_payload;
    Eigen::Vector3d payloadAngAccel = payloadInertia.inverse() * payloadTorque;

    // Update qdot for payload indices
    qdot[0] = payloadVel.x();
    qdot[1] = payloadVel.y();
    qdot[2] = payloadVel.z();

    qdot[3] = quatDot.x();
    qdot[4] = quatDot.y();
    qdot[5] = quatDot.z();
    qdot[6] = quatDot.w();

    qdot[7] = payloadAccel.x();
    qdot[8] = payloadAccel.y();
    qdot[9] = payloadAccel.z();

    qdot[10] = payloadAngAccel.x();
    qdot[11] = payloadAngAccel.y();
    qdot[12] = payloadAngAccel.z();
}




void ompl::app::PayloadSystem::postPropagate(const base::State* /*state*/, const control::Control* /*control*/, const double /*duration*/, base::State* result)
{
    const base::CompoundStateSpace* cs = getStateSpace()->as<base::CompoundStateSpace>();

    // Normalize the SE3 quaternion for the payload
    const base::SE3StateSpace* SE3 = cs->as<base::SE3StateSpace>(0);
    base::SE3StateSpace::StateType& payloadSE3State = *result->as<base::CompoundStateSpace::StateType>()
                                                         ->as<base::SE3StateSpace::StateType>(0);
    SE3->enforceBounds(&payloadSE3State.rotation());

    // Enforce bounds for the payload's velocity
    cs->getSubspace(1)->enforceBounds(result->as<base::CompoundStateSpace::StateType>()->components[1]);

    // Process each drone
    for (unsigned int i = 0; i < droneCount_; ++i)
    {
        unsigned int droneBaseIndex = 2 + i * 3; // Offset: payload's SE3 (0) + velocity (1)

        // Normalize the SO3 quaternion for each drone
        const base::SO3StateSpace* SO3 = cs->as<base::SO3StateSpace>(droneBaseIndex);
        base::SO3StateSpace::StateType& droneSO3State = *result->as<base::CompoundStateSpace::StateType>()
                                                          ->components[droneBaseIndex]
                                                          ->as<base::SO3StateSpace::StateType>();
        SO3->enforceBounds(&droneSO3State);

        // Enforce velocity bounds for each drone
        cs->getSubspace(droneBaseIndex + 1)->enforceBounds(result->as<base::CompoundStateSpace::StateType>()->components[droneBaseIndex + 1]);

        // Enforce bounds for cable angles and velocities for each drone
        cs->getSubspace(droneBaseIndex + 2)->enforceBounds(result->as<base::CompoundStateSpace::StateType>()->components[droneBaseIndex + 2]);
    }
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
