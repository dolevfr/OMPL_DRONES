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

#include "MultiDronePlanning.h"



unsigned int ompl::app::MultiDronePlanning::droneCount_ = 4; // Default number of drones


ompl::app::MultiDronePlanning::MultiDronePlanning()
    : AppBase<AppType::CONTROL>(constructControlSpace(), Motion_3D),
      odeSolver(std::make_shared<control::ODEBasicSolver<>>(si_, [this](const control::ODESolver::StateType& q, const control::Control* ctrl, control::ODESolver::StateType& qdot)
          {
              ode(q, ctrl, qdot);
          }))
{
    name_ = std::string("MultiDrone");
    setDefaultBounds();

    si_->setPropagationStepSize(0.1);  // Example step size (adjust as needed)
    si_->setMinMaxControlDuration(1, 10);  // Allow between 1 and 10 propagation stepsz

    si_->setStatePropagator(control::ODESolver::getStatePropagator(odeSolver,
        [this](const base::State* state, const control::Control* control, const double duration, base::State* result)
        {
            postPropagate(state, control, duration, result);
        }));

    setEnvironmentMesh("/usr/local/share/ompl/resources/3D/Twistycool_env.dae");
    setRobotMesh("/usr/local/share/ompl/resources/3D/quadrotor.dae");

    // Initialize massInv_ and beta_
    size_t droneCount = 4;       // Example: 4 drones (adjust as needed)
    double droneMass = 1.0;      // Example: 1 kg per drone
    double dampingFactor = 0.1;  // Example damping factor

    massInv_.resize(droneCount);
    beta_.resize(droneCount);

    for (size_t i = 0; i < droneCount; ++i)
    {
        massInv_[i] = 1.0 / droneMass; // Inverse mass
        beta_[i] = dampingFactor;     // Damping coefficient
    }
}




bool ompl::app::MultiDronePlanning::isSelfCollisionEnabled() const
{
    return false;  // No self-collision for drones
}

unsigned int ompl::app::MultiDronePlanning::getRobotCount() const
{
    return droneCount_;
}

ompl::base::ScopedState<> ompl::app::MultiDronePlanning::getDefaultStartState() const
{
    // base::ScopedState<> start(getStateSpace());
    // start = 0.0;  // Initialize all state components to zero

    // for (unsigned int i = 0; i < droneCount_; ++i)
    // {
    //     unsigned int baseIndex = i * 2;

    //     // Initialize SE3 subspace
    //     auto *droneStart = start->as<base::CompoundState>()->components[baseIndex]->as<base::SE3StateSpace::StateType>();
    //     droneStart->setXYZ(250 + i * 50, 150 + (i % 2) * 50, -100);  // Adjust positions
    //     droneStart->rotation().setIdentity();  // Identity quaternion

    //     // Initialize velocity subspace
    //     auto *velocityStart = start->as<base::CompoundState>()->components[baseIndex + 1]->as<base::RealVectorStateSpace::StateType>();
    //     for (unsigned int j = 0; j < 6; ++j)
    //     {
    //         velocityStart->values[j] = 0.0;  // Zero velocity
    //     }
    // }

    // return start;

    // Dummy
    return base::ScopedState<>(getStateSpace());
}




ompl::base::ScopedState<> ompl::app::MultiDronePlanning::getFullStateFromGeometricComponent(
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

void ompl::app::MultiDronePlanning::ode(const control::ODESolver::StateType& q, const control::Control* ctrl, control::ODESolver::StateType& qdot)
{
    const double *u = ctrl->as<control::RealVectorControlSpace::ControlType>()->values;

    // Resize qdot to match q (state vector for all drones)
    qdot.resize(q.size(), 0);

    unsigned int droneStateSize = 13; // 13 state variables per drone
    for (unsigned int i = 0; i < droneCount_; ++i)
    {

        unsigned int baseIndex = i * droneStateSize; // Start index for the i-th drone in q

        // Derivative of position
        qdot[baseIndex + 0] = q[baseIndex + 7]; // dx = vx
        qdot[baseIndex + 1] = q[baseIndex + 8]; // dy = vy
        qdot[baseIndex + 2] = q[baseIndex + 9]; // dz = vz

        // Derivative of orientation
        base::SO3StateSpace::StateType qomega;
        qomega.w = 0;
        qomega.x = 0.5 * q[baseIndex + 10];
        qomega.y = 0.5 * q[baseIndex + 11];
        qomega.z = 0.5 * q[baseIndex + 12];

        double delta = q[baseIndex + 3] * qomega.x + q[baseIndex + 4] * qomega.y + q[baseIndex + 5] * qomega.z;

        qdot[baseIndex + 3] = qomega.x - delta * q[baseIndex + 3];
        qdot[baseIndex + 4] = qomega.y - delta * q[baseIndex + 4];
        qdot[baseIndex + 5] = qomega.z - delta * q[baseIndex + 5];
        qdot[baseIndex + 6] = qomega.w - delta * q[baseIndex + 6];

        // Derivative of velocity
        qdot[baseIndex + 7] = massInv_[i] * (-2 * u[i * 4 + 0] * (q[baseIndex + 6] * q[baseIndex + 4] + q[baseIndex + 3] * q[baseIndex + 5]) - beta_[i] * q[baseIndex + 7]);
        qdot[baseIndex + 8] = massInv_[i] * (-2 * u[i * 4 + 0] * (q[baseIndex + 4] * q[baseIndex + 5] - q[baseIndex + 6] * q[baseIndex + 3]) - beta_[i] * q[baseIndex + 8]);
        qdot[baseIndex + 9] = massInv_[i] * (-u[i * 4 + 0] * (q[baseIndex + 6] * q[baseIndex + 6] - q[baseIndex + 3] * q[baseIndex + 3] - q[baseIndex + 4] * q[baseIndex + 4] + q[baseIndex + 5] * q[baseIndex + 5]) - beta_[i] * q[baseIndex + 9]) - 9.81;

        // Derivative of rotational velocity
        qdot[baseIndex + 10] = u[i * 4 + 1];
        qdot[baseIndex + 11] = u[i * 4 + 2];
        qdot[baseIndex + 12] = u[i * 4 + 3];
    }
}


void ompl::app::MultiDronePlanning::postPropagate(const base::State* /*state*/, const control::Control* /*control*/, double /*duration*/, base::State* result)
{
    const base::CompoundStateSpace* cs = getStateSpace()->as<base::CompoundStateSpace>();

    for (unsigned int i = 0; i < droneCount_; ++i)
    {
        // Normalize quaternion for each drone
        const base::SO3StateSpace* SO3 = cs->as<base::SE3StateSpace>(i * 2)->as<base::SO3StateSpace>(1);
        base::CompoundStateSpace::StateType& csState = *result->as<base::CompoundStateSpace::StateType>();
        base::SO3StateSpace::StateType& so3State = csState.as<base::SE3StateSpace::StateType>(i * 2)->rotation();

        SO3->enforceBounds(&so3State);

        // Enforce velocity bounds for each drone
        cs->getSubspace(i * 2 + 1)->enforceBounds(csState[i * 2 + 1]);
    }
}


const ompl::base::State* ompl::app::MultiDronePlanning::getGeometricComponentStateInternal(const base::State* state, unsigned int index) const
{
    const base::CompoundState* compoundState = state->as<base::CompoundState>();
    return compoundState->components[index * 2];
}


ompl::base::StateSpacePtr ompl::app::MultiDronePlanning::constructStateSpace()
{
    auto stateSpace = std::make_shared<base::CompoundStateSpace>();

    for (unsigned int i = 0; i < droneCount_; ++i)
    {
        // Add SE3 state space for position and orientation
        stateSpace->addSubspace(std::make_shared<base::SE3StateSpace>(), 1.0);

        // Add RealVector state space for velocity
        stateSpace->addSubspace(std::make_shared<base::RealVectorStateSpace>(6), 0.3);
    }

    stateSpace->lock();
    return stateSpace;
}


ompl::control::ControlSpacePtr ompl::app::MultiDronePlanning::constructControlSpace()
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
        controlBounds.setHigh(4 * i, 30); // Thrust upper bound

        // Roll torque
        controlBounds.setLow(4 * i + 1, -5); // Roll torque lower bound
        controlBounds.setHigh(4 * i + 1, 5); // Roll torque upper bound

        // Pitch torque
        controlBounds.setLow(4 * i + 2, -5); // Pitch torque lower bound
        controlBounds.setHigh(4 * i + 2, 5); // Pitch torque upper bound

        // Yaw torque
        controlBounds.setLow(4 * i + 3, -5); // Yaw torque lower bound
        controlBounds.setHigh(4 * i + 3, 5); // Yaw torque upper bound
    }

    // Apply bounds to the control space
    controlSpace->as<ompl::control::RealVectorControlSpace>()->setBounds(controlBounds);

    return controlSpace;
}




void ompl::app::MultiDronePlanning::setDefaultBounds()
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
}