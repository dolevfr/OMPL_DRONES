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
#include "PayloadClasses.h"


unsigned int ompl::app::PayloadSystem::droneCount_ = 4; // Default number of drones

using StepperType = boost::numeric::odeint::runge_kutta_cash_karp54<std::vector<double>>;

ompl::app::PayloadSystem::PayloadSystem()
    : AppBase<AppType::CONTROL>(constructControlSpace(), Motion_3D), 
      rigidBody_(ompl::app::MotionModel::Motion_3D, ompl::app::CollisionChecker::FCL)
{
    // Set the stepper for the ODE solver
    odeSolver = std::make_shared<ompl::control::ODEAdaptiveSolver<StepperType>>(
        si_,
        [this](const ompl::control::ODESolver::StateType &q, const ompl::control::Control *ctrl, ompl::control::ODESolver::StateType &qdot)
        {
            ode(q, ctrl, qdot);
        },
        timeStep_ // Integration step size
    );

    name_ = std::string("PayloadSystem");
    setDefaultBounds();

    si_->setPropagationStepSize(timeStep_);
    si_->setMinMaxControlDuration(1, 10);

    si_->setStatePropagator(control::ODESolver::getStatePropagator(odeSolver,
        [this](const base::State *state, const control::Control *control, const double duration, base::State *result)
        {
            postPropagate(state, control, duration, result);
        }));

    si_->setup();

    si_->setStateValidityCheckingResolution(0.01);

    auto validityChecker = std::make_shared<PayloadSystemValidityChecker>(si_, *this);
    si_->setStateValidityChecker(validityChecker);

    auto motionValidator = std::make_shared<ompl::base::DiscreteMotionValidator>(si_);
    si_->setMotionValidator(motionValidator);
}

ompl::base::ScopedState<> ompl::app::PayloadSystem::getDefaultStartState() const
{
    base::ScopedState<base::SE3StateSpace> s(getGeometricComponentStateSpace());

    s->setXYZ(PayloadSystem::startPosition_.x(), PayloadSystem::startPosition_.y(), PayloadSystem::startPosition_.z());
    s->rotation().setIdentity();
    return getFullStateFromGeometricComponent(s);
}


ompl::base::StateSpacePtr ompl::app::PayloadSystem::constructStateSpace()
{
    auto stateSpace = std::make_shared<base::CompoundStateSpace>();

    // Add SE3 state space for the payload (position and orientation)
    stateSpace->addSubspace(std::make_shared<base::SE3StateSpace>(), 1.0);

    // Add RealVector state space for the payload's velocity (6 dimensions)
    stateSpace->addSubspace(std::make_shared<base::RealVectorStateSpace>(6), 0.002);

    for (unsigned int i = 0; i < droneCount_; ++i)
    {
        // Add SO3 state space for orientation
        stateSpace->addSubspace(std::make_shared<base::SO3StateSpace>(), 0.002);

        // Add RealVector state space for velocity (3 dimensions)
        stateSpace->addSubspace(std::make_shared<base::RealVectorStateSpace>(3), 0.002);

        // Add RealVector state space for cable angles and velocities (4 dimensions)
        stateSpace->addSubspace(std::make_shared<base::RealVectorStateSpace>(4), 0.002);
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
        controlBounds.setLow(4 * i, minThrust);          // Thrust lower bound
        controlBounds.setHigh(4 * i, maxThrust); // Thrust upper bound

        // Roll torque
        controlBounds.setLow(4 * i + 1, -maxTorquePitchRoll); // Roll torque lower bound
        controlBounds.setHigh(4 * i + 1, maxTorquePitchRoll); // Roll torque upper bound

        // Pitch torque
        controlBounds.setLow(4 * i + 2, -maxTorquePitchRoll); // Pitch torque lower bound
        controlBounds.setHigh(4 * i + 2, maxTorquePitchRoll); // Pitch torque upper bound

        // Yaw torque
        controlBounds.setLow(4 * i + 3, -maxTorqueYaw); // Yaw torque lower bound
        controlBounds.setHigh(4 * i + 3, maxTorqueYaw); // Yaw torque upper bound
    }

    // Apply bounds to the control space
    controlSpace->as<ompl::control::RealVectorControlSpace>()->setBounds(controlBounds);

    return controlSpace;
}

void ompl::app::PayloadSystem::ode(const control::ODESolver::StateType &q, const control::Control *ctrl, control::ODESolver::StateType &qdot)
{

    // Initialize qdot
    qdot.resize(q.size(), 0);

    const double *u = ctrl->as<ompl::control::RealVectorControlSpace::ControlType>()->values;

    // // Access the control values
    // const double *u_copy = ctrl->as<ompl::control::RealVectorControlSpace::ControlType>()->values;

    // std::vector<double> u = {
    //     50, 0.5, 0, 0,
    //     50, 0.5, 0, 0,
    //     50, 0.5, 0, 0,
    //     50, 0.5, 0, 0
    // };

    unsigned int droneStateSize = 11; // State size per drone and cable
    unsigned int droneIndex = 13;     // Start index for drones

    // Payload position and orientation
    Eigen::Vector3d payloadPos(q[0], q[1], q[2]);
    Eigen::Quaterniond payloadRot(q[4], q[5], q[6], q[3]);
    Eigen::Vector3d payloadVel(q[7], q[8], q[9]);
    Eigen::Vector3d payloadAngVel(q[10], q[11], q[12]);

    // Initialize payload force and torque
    Eigen::Vector3d payloadForce(0, 0, -m_payload * 9.81); // Gravity force on the payload
    Eigen::Vector3d payloadTorque(0, 0, 0);                // Accumulated torque on the payload

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

        // Net force on the drone
        Eigen::Vector3d droneForce = thrustMagnitude * thrustDir + m_drone * Eigen::Vector3d(0, 0, -9.81);

        // Cable unit vectors
        Eigen::Vector3d cableDir = Eigen::Vector3d(sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta));
        Eigen::Vector3d thetaDir = Eigen::Vector3d(cos(theta) * cos(phi), cos(theta) * sin(phi), -sin(theta));
        Eigen::Vector3d phiDir = Eigen::Vector3d(-sin(phi), cos(phi), 0);

        Eigen::Vector3d forceOnCable = droneForce.dot(cableDir) * cableDir;
        double forceTheta = droneForce.dot(thetaDir);
        double forcePhi = droneForce.dot(phiDir);

        Eigen::Quaterniond omega_quat(0, omega.x(), omega.y(), omega.z());
        Eigen::Quaterniond q_dot = Eigen::Quaterniond(0.5 * omega_quat.coeffs()) * droneRot;

        // Angular acceleration calculation
        Eigen::Vector3d torque(u[i * 4 + 1], u[i * 4 + 2], u[i * 4 + 3]); // Control inputs for torques
        Eigen::Vector3d angularAccel = droneInertia.inverse() * (torque - droneBeta * omega);

        // Update qdot for quaternion
        qdot[baseIndex + 0] = q_dot.x();
        qdot[baseIndex + 1] = q_dot.y();
        qdot[baseIndex + 2] = q_dot.z();
        qdot[baseIndex + 3] = q_dot.w();

        // Update qdot for angular accelerations
        qdot[baseIndex + 4] = angularAccel.x();
        qdot[baseIndex + 5] = angularAccel.y();
        qdot[baseIndex + 6] = angularAccel.z();

        // Update angular velocities of theta and phi
        qdot[baseIndex + 7] = thetaDot;
        qdot[baseIndex + 8] = phiDot;
        qdot[baseIndex + 9] = forceTheta / (l * m_drone);
        qdot[baseIndex + 10] = forcePhi / (l * m_drone);

        // corner in payload local coordinates
        Eigen::Vector3d cornerLocal;
        switch (i)
        {
        case 0:
            cornerLocal = Eigen::Vector3d(-w / 2, -d / 2, h / 2);
            break;
        case 1:
            cornerLocal = Eigen::Vector3d(w / 2, -d / 2, h / 2);
            break;
        case 2:
            cornerLocal = Eigen::Vector3d(w / 2, d / 2, h / 2);
            break;
        case 3:
            cornerLocal = Eigen::Vector3d(-w / 2, d / 2, h / 2);
            break;
        default:
            throw std::runtime_error("Invalid drone index");
        }

        // Rotate corner to world frame using payload rotation
        Eigen::Vector3d cornerWorld = payloadRot * cornerLocal;

        // Update forces and torques correctly in world frame
        payloadForce += forceOnCable;
        payloadTorque += cornerWorld.cross(forceOnCable);
    }

    Eigen::Quaterniond omega_quat_p(0, payloadAngVel.x(), payloadAngVel.y(), payloadAngVel.z());
    Eigen::Quaterniond q_dot_p = Eigen::Quaterniond(0.5 * omega_quat_p.coeffs()) * payloadRot;

    // Payload linear and angular acceleration
    Eigen::Vector3d payloadAccel = (payloadForce - payloadBeta * payloadVel) / m_payload;
    Eigen::Vector3d payloadAngAccel = payloadInertia.inverse() * payloadTorque;

    // Update qdot for payload indices
    qdot[0] = payloadVel.x();
    qdot[1] = payloadVel.y();
    qdot[2] = payloadVel.z();

    qdot[3] = q_dot_p.x();
    qdot[4] = q_dot_p.y();
    qdot[5] = q_dot_p.z();
    qdot[6] = q_dot_p.w();

    qdot[7] = payloadAccel.x();
    qdot[8] = payloadAccel.y();
    qdot[9] = payloadAccel.z();

    qdot[10] = payloadAngAccel.x();
    qdot[11] = payloadAngAccel.y();
    qdot[12] = payloadAngAccel.z();
}

void ompl::app::PayloadSystem::postPropagate(const base::State * /*state*/, const control::Control *control, const double /*duration*/, base::State *result)
{   
    // Ensure the custom validity checker is always used
    si_->setStateValidityChecker(std::make_shared<PayloadSystemValidityChecker>(si_, *this));

    // Access the CompoundStateSpace and subspaces
    const base::CompoundStateSpace *cs = getStateSpace()->as<base::CompoundStateSpace>();

    // Cast the result to a CompoundStateSpace::StateType
    auto *compoundState = result->as<ompl::base::CompoundStateSpace::StateType>();

    // Normalize the SE3 quaternion for the payload
    const base::SE3StateSpace *SE3 = cs->as<base::SE3StateSpace>(0); // Assuming the payload is at index 0
    base::SE3StateSpace::StateType &payloadSE3State = *compoundState->as<base::SE3StateSpace::StateType>(0);

    // Finally, enforce bounds within SO3
    SE3->as<base::SO3StateSpace>(1)->enforceBounds(&payloadSE3State.rotation());

    // Enforce velocity bounds (assuming velocity is in subspace 1)
    cs->getSubspace(1)->enforceBounds(compoundState->as<base::RealVectorStateSpace::StateType>(1));

    // Process each drone
    for (unsigned int i = 0; i < droneCount_; ++i)
    {
        unsigned int droneBaseIndex = 2 + i * 3; // Offset: payload's SE3 (0) + velocity (1)

        // Normalize the SO3 quaternion for each drone
        const base::SO3StateSpace *SO3 = cs->as<base::SO3StateSpace>(droneBaseIndex);
        base::SO3StateSpace::StateType &droneSO3State = *compoundState->components[droneBaseIndex]
                                                             ->as<base::SO3StateSpace::StateType>();


        // Finally, enforce bounds within SO3
        SO3->enforceBounds(&droneSO3State);

        // Enforce velocity bounds for each drone
        cs->getSubspace(droneBaseIndex + 1)->enforceBounds(compoundState->components[droneBaseIndex + 1]);

        // Enforce bounds for cable angles and velocities for each drone
        base::RealVectorStateSpace::StateType *cableState =
            compoundState->components[droneBaseIndex + 2]->as<base::RealVectorStateSpace::StateType>();
        cs->getSubspace(droneBaseIndex + 2)->enforceBounds(cableState);

        // Normalize phi (cable elevation angle)
        double &phi = cableState->values[1]; // Assuming cable angles are stored as theta = [0], phi = [1]
        while (phi < 0)
            phi += 2 * M_PI; // Add 2π until phi is positive
        while (phi > 2 * M_PI)
            phi -= 2 * M_PI; // Subtract 2π until phi is within range
    }
}

void ompl::app::PayloadSystem::setDefaultBounds()
{
    // Enforce payload position bounds (-300, 600) for x, y, z
    base::RealVectorBounds positionBounds(3); // SE3 position bounds
    positionBounds.setLow(-1000);
    positionBounds.setHigh(1000);
    getStateSpace()->as<base::CompoundStateSpace>()->as<base::SE3StateSpace>(0)->setBounds(positionBounds);

    // Enforce payload velocity bounds (-10, 10) for x, y, z, and angular velocities
    base::RealVectorBounds velocityBounds(6); // Bounds for payload velocity (linear and angular)
    velocityBounds.setLow(-maxPayloadVel);
    velocityBounds.setHigh(maxPayloadVel);
    getStateSpace()->as<base::CompoundStateSpace>()->as<base::RealVectorStateSpace>(1)->setBounds(velocityBounds);

    // Loop through each drone and enforce bounds
    for (unsigned int i = 0; i < droneCount_; ++i)
    {
        // Enforce bounds on drone angular velocity (-10, 10) for x, y, z
        base::RealVectorBounds droneVelocityBounds(3); // Bounds for drone angular velocity
        droneVelocityBounds.setLow(-maxDroneVel);
        droneVelocityBounds.setHigh(maxDroneVel);
        getStateSpace()->as<base::CompoundStateSpace>()->as<base::RealVectorStateSpace>(3 + i * 3)->setBounds(droneVelocityBounds);

        // Enforce bounds on theta for each cable (-10, 10 degrees)
        base::RealVectorBounds cableAngleBounds(4); // Bounds for (theta, phi, theta_dot, phi_dot)

        cableAngleBounds.setLow(0, -maxTheta * M_PI / 180); // Theta (index 0) lower bound in radians
        cableAngleBounds.setHigh(0, maxTheta * M_PI / 180); // Theta (index 0) upper bound in radians
        cableAngleBounds.setLow(1, -1e6);                   // No restriction on phi
        cableAngleBounds.setHigh(1, 1e6);
        cableAngleBounds.setLow(2, -maxThetaVel); // Theta_dot lower bound
        cableAngleBounds.setHigh(2, maxThetaVel); // Theta_dot upper bound
        cableAngleBounds.setLow(3, -maxThetaVel); // Phi_dot lower bound
        cableAngleBounds.setHigh(3, maxThetaVel); // Phi_dot upper bound
        getStateSpace()->as<base::CompoundStateSpace>()->as<base::RealVectorStateSpace>(4 + i * 3)->setBounds(cableAngleBounds);
    }
}


