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

unsigned int ompl::app::PayloadSystem::droneCount_ = 4; // Default number of drones

using StepperType = boost::numeric::odeint::runge_kutta_cash_karp54<std::vector<double>>;

#include <boost/filesystem.hpp>

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
    si_->setMinMaxControlDuration(1, 100);

    si_->setStatePropagator(control::ODESolver::getStatePropagator(odeSolver,
                                                                   [this](const base::State *state, const control::Control *control, const double duration, base::State *result)
                                                                   {
                                                                       postPropagate(state, control, duration, result);
                                                                   }));

    // Load Environment and Robot Meshes
    std::string meshDir = boost::filesystem::absolute("../src/meshes").string();
    rigidBody_.setMeshPath({boost::filesystem::path(meshDir)});

    rigidBody_.setEnvironmentMesh(meshDir + "/env_3.dae");
    rigidBody_.setRobotMesh(meshDir + "/box.dae");

    for (unsigned int i = 0; i < getRobotCount(); ++i)
        rigidBody_.addRobotMesh(meshDir + "/drone.dae");

    // Set the state validity checker using our custom PayloadStateValidityChecker
    si_->setStateValidityChecker(
        std::make_shared<PayloadStateValidityChecker>(si_, rigidBody_, *this));

    // IMPORTANT: Set a finer state validity checking resolution (e.g., 3% of maximum extent)
    si_->setStateValidityCheckingResolution(0.03);

    si_->setup();

}


ompl::base::ScopedState<> ompl::app::PayloadSystem::getFullStateFromGeometricComponent(
    const base::ScopedState<> &state) const
{
    base::ScopedState<> fullState(getStateSpace());
    std::vector<double> reals = state.reals();

    fullState = 0.0;
    for (size_t i = 0; i < reals.size(); ++i)
    {
        fullState[i] = reals[i]; // Copy values to the full state
    }
    return fullState;
}

const ompl::base::State *ompl::app::PayloadSystem::getGeometricComponentStateInternal(const base::State *state, unsigned int index) const
{
    const base::CompoundState *compoundState = state->as<base::CompoundState>();
    return compoundState->components[index * 2];
}

ompl::base::StateSpacePtr ompl::app::PayloadSystem::constructStateSpace()
{
    auto stateSpace = std::make_shared<base::CompoundStateSpace>();

    // Add SE3 state space for the payload (position and orientation)
    stateSpace->addSubspace(std::make_shared<base::SE3StateSpace>(), 1.0);

    // Add RealVector state space for the payload's velocity (6 dimensions)
    stateSpace->addSubspace(std::make_shared<base::RealVectorStateSpace>(6), 0.02);

    for (unsigned int i = 0; i < droneCount_; ++i)
    {
        // Add SO3 state space for  orientation
        stateSpace->addSubspace(std::make_shared<base::SO3StateSpace>(), 0.02);

        // Add RealVector state space for velocity (3 dimensions)
        stateSpace->addSubspace(std::make_shared<base::RealVectorStateSpace>(3), 0.02);

        // Add RealVector state space for cable angles and velocities (4 dimensions)
        stateSpace->addSubspace(std::make_shared<base::RealVectorStateSpace>(4), 0.02);
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
        controlBounds.setLow(4 * i, 0);          // Thrust lower bound
        controlBounds.setHigh(4 * i, maxThrust); // Thrust upper bound

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
        // std::cout << "Drone " << i << " quaternion: " << droneRot.x() << " " << droneRot.y() << " " << droneRot.z() << " " << droneRot.w() << std::endl;
        Eigen::Vector3d omega(q[baseIndex + 4], q[baseIndex + 5], q[baseIndex + 6]);
        // std::cout << "Drone " << i << " angular velocity: " << omega.transpose() << std::endl;

        // Cables angles and velocities
        double theta = q[baseIndex + 7];
        double phi = q[baseIndex + 8];
        double thetaDot = q[baseIndex + 9];
        double phiDot = q[baseIndex + 10];

        // Trust direction
        Eigen::Vector3d thrustDir = droneRot * Eigen::Vector3d(0, 0, 1);

        // Retrieve thrust magnitude from control input
        double thrustMagnitude = u[i * 4];

        Eigen::Vector3d thrust = thrustMagnitude * thrustDir;

        // Cable unit vectors
        Eigen::Vector3d cableDir = Eigen::Vector3d(sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta));
        Eigen::Vector3d thetaDir = Eigen::Vector3d(cos(theta) * cos(phi), cos(theta) * sin(phi), -sin(theta));
        Eigen::Vector3d phiDir = Eigen::Vector3d(-sin(phi), cos(phi), 0);

        Eigen::Vector3d thrustOnCable = thrust.dot(cableDir) * cableDir;
        Eigen::Vector3d thrustTheta = thrust.dot(thetaDir) * thetaDir;
        Eigen::Vector3d thrustPhi = thrust.dot(phiDir) * phiDir;

        // Construct Omega(omega) matrix
        Eigen::Matrix4d Omega;
        Omega << 0, -omega.x(), -omega.y(), -omega.z(),
            omega.x(), 0, omega.z(), -omega.y(),
            omega.y(), -omega.z(), 0, omega.x(),
            omega.z(), omega.y(), -omega.x(), 0;

        // Convert quaternion to 4D vector
        Eigen::Vector4d quatVec(droneRot.w(), droneRot.x(), droneRot.y(), droneRot.z());

        // Compute quaternion derivative
        Eigen::Vector4d quatDot = 0.5 * Omega * quatVec;

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

        // Calculate the rotated directions of the payload
        Eigen::Vector3d payloadXDir = payloadRot * Eigen::Vector3d(1, 0, 0); // x-direction in the payload's local frame
        Eigen::Vector3d payloadYDir = payloadRot * Eigen::Vector3d(0, 1, 0); // y-direction in the payload's local frame
        Eigen::Vector3d payloadZDir = payloadRot * Eigen::Vector3d(0, 0, 1); // z-direction in the payload's local frame

        // Determine the corner position of the payload
        Eigen::Vector3d corner;
        switch (i)
        {
        case 0:
            corner = Eigen::Vector3d(-w / 2, -d / 2, h / 2);
            break;
        case 1:
            corner = Eigen::Vector3d(w / 2, -d / 2, h / 2);
            break;
        case 2:
            corner = Eigen::Vector3d(w / 2, d / 2, h / 2);
            break;
        case 3:
            corner = Eigen::Vector3d(-w / 2, d / 2, h / 2);
            break;
        default:
            throw std::runtime_error("Invalid drone index");
        }

        // Update forces and torques
        payloadForce += thrustOnCable; // Thrust force on the payload
        payloadTorque += corner.cross(thrustOnCable); // Torque on the payload
    }

    Eigen::Matrix4d Omega;
    Omega << 0, -payloadAngVel.x(), -payloadAngVel.y(), -payloadAngVel.z(),
        payloadAngVel.x(), 0, payloadAngVel.z(), -payloadAngVel.y(),
        payloadAngVel.y(), -payloadAngVel.z(), 0, payloadAngVel.x(),
        payloadAngVel.z(), payloadAngVel.y(), -payloadAngVel.x(), 0;

    // Convert quaternion to 4D vector
    Eigen::Vector4d quatVec(payloadRot.w(), payloadRot.x(), payloadRot.y(), payloadRot.z());

    // Compute quaternion derivative
    Eigen::Vector4d quatDot = 0.5 * Omega * quatVec;

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

void ompl::app::PayloadSystem::postPropagate(const base::State * /*state*/, const control::Control *control, const double /*duration*/, base::State *result)
{
    // Access the CompoundStateSpace and subspaces
    const base::CompoundStateSpace *cs = getStateSpace()->as<base::CompoundStateSpace>();

    // Cast the result to a CompoundStateSpace::StateType
    auto *compoundState = result->as<ompl::base::CompoundStateSpace::StateType>();

    // Normalize the SE3 quaternion for the payload
    const base::SE3StateSpace *SE3 = cs->as<base::SE3StateSpace>(0); // Assuming the payload is at index 0
    base::SE3StateSpace::StateType &payloadSE3State = *compoundState->as<base::SE3StateSpace::StateType>(0);

    // Convert quaternion to Eigen quaternion
    Eigen::Quaterniond quat(payloadSE3State.rotation().w,
                            payloadSE3State.rotation().x,
                            payloadSE3State.rotation().y,
                            payloadSE3State.rotation().z);

    // Convert quaternion to Euler angles (Yaw-Pitch-Roll)
    Eigen::Vector3d euler = quat.toRotationMatrix().eulerAngles(2, 1, 0); // ZYX order (Yaw, Pitch, Roll)

    // Enforce pitch and roll bounds
    constexpr double maxPayloadRad = maxAnglePayload * M_PI / 180.0; // Payload pitch and roll limits in radians
    euler[1] = std::clamp(euler[1], -maxPayloadRad, maxPayloadRad);  // Clamp pitch
    euler[2] = std::clamp(euler[2], -maxPayloadRad, maxPayloadRad);  // Clamp roll

    // Reconstruct quaternion from constrained Euler angles
    Eigen::Quaterniond constrainedQuat =
        Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitZ()) * // Yaw
        Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) * // Pitch
        Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitX());  // Roll

    // Set constrained quaternion values
    payloadSE3State.rotation().w = constrainedQuat.w();
    payloadSE3State.rotation().x = constrainedQuat.x();
    payloadSE3State.rotation().y = constrainedQuat.y();
    payloadSE3State.rotation().z = constrainedQuat.z();

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

        // Convert quaternion to Eigen quaternion
        Eigen::Quaterniond quat(droneSO3State.w, droneSO3State.x, droneSO3State.y, droneSO3State.z);

        // Convert quaternion to Euler angles (yaw-pitch-roll convention)
        Eigen::Vector3d euler = quat.toRotationMatrix().eulerAngles(2, 1, 0); // Yaw-Pitch-Roll (ZYX order)

        // Enforce pitch and roll bounds
        constexpr double maxDroneRad = maxDroneAngle * M_PI / 180.0; // Drone pitch and roll limits in radians
        euler[1] = std::clamp(euler[1], -maxDroneRad, maxDroneRad);  // Clamp pitch
        euler[2] = std::clamp(euler[2], -maxDroneRad, maxDroneRad);  // Clamp roll

        // Reconstruct quaternion from the constrained Euler angles
        Eigen::Quaterniond constrainedQuat =
            Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitZ()) * // Yaw
            Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) * // Pitch
            Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitX());  // Roll

        // Set constrained quaternion values
        droneSO3State.w = constrainedQuat.w();
        droneSO3State.x = constrainedQuat.x();
        droneSO3State.y = constrainedQuat.y();
        droneSO3State.z = constrainedQuat.z();

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
