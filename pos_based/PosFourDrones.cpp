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

#include "PosFourDrones.h"

unsigned int ompl::app::PayloadSystem::droneCount_ = 4; // Default number of drones

using StepperType = boost::numeric::odeint::runge_kutta_cash_karp54<std::vector<double>>;

ompl::app::PayloadSystem::PayloadSystem()
    : AppBase<AppType::CONTROL>(constructControlSpace(), Motion_3D), 
      rigidBody_(ompl::app::MotionModel::Motion_3D, ompl::app::CollisionChecker::FCL)
{
    // Retrieve bounds from the control space (must match what was set in constructControlSpace)
    auto* rvcs = getControlSpace()->as<ompl::control::RealVectorControlSpace>();
    const ompl::base::RealVectorBounds& bounds = rvcs->getBounds();

    // Now use the bounds in the allocator
    rvcs->setControlSamplerAllocator(
        [bounds](const ompl::control::ControlSpace *space)
        {
            return std::make_shared<PosRRTControlSampler>(space, bounds);
        });

    name_ = std::string("PayloadSystem");
    setDefaultBounds();

    si_->setPropagationStepSize(timeStep_);
    si_->setMinMaxControlDuration(1, 10);

    // Initialize Python and import acados_steer module (now for position tracking!)
    py::module sys = py::module::import("sys");
    py::list sys_path = sys.attr("path");
    sys_path.append("../pos_based");  // or your Python path
    std::cout << "[INFO] Importing Python module 'acados_steer'..." << std::endl;
    acados_mod_ = py::module::import("acados_steer");
    std::cout << "[INFO] Setting up Acados OCP via Python..." << std::endl;
    py::object acados_ocp = acados_mod_.attr("setup_ocp")();
    std::cout << "[DEBUG] acados_ocp Python type: " << std::string(py::str(acados_ocp.get_type())) << std::endl;
    acados_ocp_ = acados_ocp;
    std::cout << "[INFO] Acados OCP initialized for position tracking." << std::endl;
    
    si_->setStatePropagator([this](const base::State *state, const control::Control *control, double duration, base::State *result)
        {
            postPropagate(state, control, duration, result);
        });

    auto motionValidator = std::make_shared<ompl::base::DiscreteMotionValidator>(si_);
    si_->setMotionValidator(motionValidator);
    si_->setup();
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
    stateSpace->addSubspace(std::make_shared<base::RealVectorStateSpace>(6), 0.05);

    for (unsigned int i = 0; i < droneCount_; ++i)
    {
        // Add SO3 state space for orientation
        stateSpace->addSubspace(std::make_shared<base::SO3StateSpace>(), 0.05);

        // Add RealVector state space for velocity (3 dimensions)
        stateSpace->addSubspace(std::make_shared<base::RealVectorStateSpace>(3), 0.05);

        // Add RealVector state space for cable angles and velocities (4 dimensions)
        stateSpace->addSubspace(std::make_shared<base::RealVectorStateSpace>(4), 0.05);
    }

    stateSpace->lock();

    return stateSpace;
}

ompl::control::ControlSpacePtr ompl::app::PayloadSystem::constructControlSpace()
{
    // Construct a control space with dimensions equal to 4 times the number of drones
    auto controlSpace = std::make_shared<ompl::control::RealVectorControlSpace>(constructStateSpace(), 3);

    // Set bounds for the target position (absolute) or position increment (delta)
    ompl::base::RealVectorBounds controlBounds(3);

    // If using *absolute* positions as control:
    controlBounds.setLow(0, -22);  // min x
    controlBounds.setHigh(0, 88);  // max x
    controlBounds.setLow(1, -53);  // min y
    controlBounds.setHigh(1, 50);  // max y
    controlBounds.setLow(2, -1);   // min z
    controlBounds.setHigh(2, 27);  // max z

    // Apply bounds to the control space
    controlSpace->as<ompl::control::RealVectorControlSpace>()->setBounds(controlBounds);

    return controlSpace;
}

void ompl::app::PayloadSystem::postPropagate(
    const base::State *from, const control::Control *control,
    double /*duration*/, base::State *to)
{
    std::cout << "============= New Propagation ===============" << std::endl;
    // 1. Extract previous full state -- HARDCODED for your state space!
    auto compoundFrom = from->as<ompl::base::CompoundState>();
    auto compoundSpace = si_->getStateSpace()->as<ompl::base::CompoundStateSpace>();

    // State vector size: 13 payload + 4*11 = 57
    int n_states = 57;
    Eigen::VectorXd current_state(n_states);
    int offset = 0;

    // [0] Payload SE3 (7)
    auto se3payload = compoundFrom->as<ompl::base::SE3StateSpace::StateType>(0);
    current_state[offset++] = se3payload->getX();
    current_state[offset++] = se3payload->getY();
    current_state[offset++] = se3payload->getZ();
    current_state[offset++] = se3payload->rotation().w;
    current_state[offset++] = se3payload->rotation().x;
    current_state[offset++] = se3payload->rotation().y;
    current_state[offset++] = se3payload->rotation().z;

    // [1] Payload velocity (6)
    auto payload_vel = compoundFrom->as<ompl::base::RealVectorStateSpace::StateType>(1);
    for (int i = 0; i < 6; ++i)
        current_state[offset++] = payload_vel->values[i];

    // [2..13]: 4 drones, each [SO3][R3][R4]
    for (int d = 0; d < 4; ++d) {
        int base = 2 + d*3;
        // SO3 (4)
        auto drone_quat = compoundFrom->as<ompl::base::SO3StateSpace::StateType>(base);
        current_state[offset++] = drone_quat->w;
        current_state[offset++] = drone_quat->x;
        current_state[offset++] = drone_quat->y;
        current_state[offset++] = drone_quat->z;

        // R3 (drone velocity)
        auto drone_vel = compoundFrom->as<ompl::base::RealVectorStateSpace::StateType>(base + 1);
        for (int i = 0; i < 3; ++i)
            current_state[offset++] = drone_vel->values[i];

        // R4 (cable)
        auto cable = compoundFrom->as<ompl::base::RealVectorStateSpace::StateType>(base + 2);
        for (int i = 0; i < 4; ++i)
            current_state[offset++] = cable->values[i];
    }
    assert(offset == n_states && "Mismatch in state extraction size!");

    int drone_state_len = 11;

    // std::cout << "\n========== CURRENT STATE ==========" << std::endl;
    // // Print payload position, quaternion, velocity, angular velocity
    // std::cout << "Payload pos:    [";
    // for (int i = 0; i < 3; ++i) std::cout << std::fixed << std::setprecision(4) << current_state(i) << (i < 2 ? ", " : "");
    // std::cout << "]" << std::endl;

    // std::cout << "Payload quat:   [";
    // for (int i = 3; i < 7; ++i) std::cout << std::fixed << std::setprecision(4) << current_state(i) << (i < 6 ? ", " : "");
    // std::cout << "]" << std::endl;

    // std::cout << "Payload vel:    [";
    // for (int i = 7; i < 10; ++i) std::cout << std::fixed << std::setprecision(4) << current_state(i) << (i < 9 ? ", " : "");
    // std::cout << "]" << std::endl;

    // std::cout << "Payload angvel: [";
    // for (int i = 10; i < 13; ++i) std::cout << std::fixed << std::setprecision(4) << current_state(i) << (i < 12 ? ", " : "");
    // std::cout << "]" << std::endl;

    // // Print drone states
    // for (int i = 0; i < droneCount_; ++i) {
    //     int offset = 13 + i * drone_state_len;
    //     std::cout << "Drone " << (i+1) << " quat:      [";
    //     for (int j = 0; j < 4; ++j) std::cout << std::fixed << std::setprecision(4) << current_state(offset + j) << (j < 3 ? ", " : "");
    //     std::cout << "]" << std::endl;

    //     std::cout << "Drone " << (i+1) << " angvel:    [";
    //     for (int j = 4; j < 7; ++j) std::cout << std::fixed << std::setprecision(4) << current_state(offset + j) << (j < 6 ? ", " : "");
    //     std::cout << "]" << std::endl;

    //     std::cout << "Drone " << (i+1) << " θ, φ:       ("
    //               << std::fixed << std::setprecision(4) << current_state(offset + 7) << ", "
    //               << std::fixed << std::setprecision(4) << current_state(offset + 8) << ")" << std::endl;

    //     std::cout << "Drone " << (i+1) << " θ̇, φ̇:      ("
    //               << std::fixed << std::setprecision(4) << current_state(offset + 9) << ", "
    //               << std::fixed << std::setprecision(4) << current_state(offset + 10) << ")" << std::endl;
    // }

    // 2. Previous payload position (from state)
    Eigen::Vector3d prev_pos(
        se3payload->getX(),
        se3payload->getY(),
        se3payload->getZ()
    );
    std::cout << "[DEBUG] Previous payload position: " << prev_pos.transpose() << std::endl;

    // 3. Sampled absolute target position from control
    auto ctrl = control->as<ompl::control::RealVectorControlSpace::ControlType>();
    Eigen::Vector3d sampled_target(ctrl->values[0], ctrl->values[1], ctrl->values[2]);

    // 4. Step in that direction by fixed length (posMaxDelta)
    Eigen::Vector3d diff = sampled_target - prev_pos;
    double dist = diff.norm();
    Eigen::Vector3d next_pos;
    if (dist <= posMaxDelta) {
        next_pos = sampled_target;
    } else {
        next_pos = prev_pos + diff * posMaxDelta / dist;
    }

    // 5. Build acc_ref_size-point linspace trajectory between prev_pos and next_pos
    Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor> pos_traj(pos_ref_size, 3);
    for (int j = 0; j < pos_ref_size; ++j) {
        double alpha = double(j) / double(pos_ref_size - 1);
        pos_traj.row(j) = prev_pos * (1.0 - alpha) + next_pos * alpha;
    }

    // 6. Convert to Python arrays, call OCP, etc.
    py::array_t<double> py_state(current_state.size(), current_state.data());
    py::array_t<double> py_pos_traj({pos_ref_size, 3}, pos_traj.data());

    try{
        // 6. Call Acados Python MPC (returns trajectory of states & controls)
        py::tuple py_result = acados_mod_.attr("steer_acados")(
            acados_ocp_, py_state, py_pos_traj);

        py::array_t<double> py_states = py_result[0].cast<py::array_t<double>>();
        // py::array_t<double> py_controls = py_result[1].cast<py::array_t<double>>(); // If needed
        py::array_t<double> py_final_pos = py_result[2].cast<py::array_t<double>>();
        auto buf_pos = py_final_pos.request();
        double* pos_ptr = static_cast<double*>(buf_pos.ptr);

        // 7. Extract last state and last position
        auto buf_states = py_states.request();
        int n_steps = buf_states.shape[0];
        int n_s = buf_states.shape[1];
        double *states_ptr = static_cast<double*>(buf_states.ptr);


        // The last state (row) includes all state variables
        Eigen::Map<Eigen::VectorXd> last_state(states_ptr + (n_steps - 1) * n_s, n_s);

        // std::cout << "\n========== LAST STATE ==========" << std::endl;
        // // Print payload position, quaternion, velocity, angular velocity from last_state
        // std::cout << "Payload pos:    [";
        // for (int i = 0; i < 3; ++i) std::cout << std::fixed << std::setprecision(4) << last_state(i) << (i < 2 ? ", " : "");
        // std::cout << "]" << std::endl;

        // std::cout << "Payload quat:   [";
        // for (int i = 3; i < 7; ++i) std::cout << std::fixed << std::setprecision(4) << last_state(i) << (i < 6 ? ", " : "");
        // std::cout << "]" << std::endl;

        // std::cout << "Payload vel:    [";
        // for (int i = 7; i < 10; ++i) std::cout << std::fixed << std::setprecision(4) << last_state(i) << (i < 9 ? ", " : "");
        // std::cout << "]" << std::endl;

        // std::cout << "Payload angvel: [";
        // for (int i = 10; i < 13; ++i) std::cout << std::fixed << std::setprecision(4) << last_state(i) << (i < 12 ? ", " : "");
        // std::cout << "]" << std::endl;

        // // Print drone states from last_state
        // for (int i = 0; i < droneCount_; ++i) {
        //     int offset = 13 + i * drone_state_len;
        //     std::cout << "Drone " << (i+1) << " quat:      [";
        //     for (int j = 0; j < 4; ++j) std::cout << std::fixed << std::setprecision(4) << last_state(offset + j) << (j < 3 ? ", " : "");
        //     std::cout << "]" << std::endl;

        //     std::cout << "Drone " << (i+1) << " angvel:    [";
        //     for (int j = 4; j < 7; ++j) std::cout << std::fixed << std::setprecision(4) << last_state(offset + j) << (j < 6 ? ", " : "");
        //     std::cout << "]" << std::endl;

        //     std::cout << "Drone " << (i+1) << " θ, φ:       ("
        //             << std::fixed << std::setprecision(4) << last_state(offset + 7) << ", "
        //             << std::fixed << std::setprecision(4) << last_state(offset + 8) << ")" << std::endl;

        //     std::cout << "Drone " << (i+1) << " θ̇, φ̇:      ("
        //             << std::fixed << std::setprecision(4) << last_state(offset + 9) << ", "
        //             << std::fixed << std::setprecision(4) << last_state(offset + 10) << ")" << std::endl;
        // }

        // 8. Write last_state to 'to' OMPL CompoundState
        auto compoundTo = to->as<ompl::base::CompoundState>();
        offset = 0;

        // Payload pose
        auto se3payload_to = compoundTo->as<ompl::base::SE3StateSpace::StateType>(0);
        se3payload_to->setX(last_state(offset++));
        se3payload_to->setY(last_state(offset++));
        se3payload_to->setZ(last_state(offset++));
        se3payload_to->rotation().w = last_state(offset++);
        se3payload_to->rotation().x = last_state(offset++);
        se3payload_to->rotation().y = last_state(offset++);
        se3payload_to->rotation().z = last_state(offset++);

        // Payload velocity
        auto payload_vel_to = compoundTo->as<ompl::base::RealVectorStateSpace::StateType>(1);
        for (int i = 0; i < 6; ++i)
            payload_vel_to->values[i] = last_state(offset++);

        for (int d = 0; d < droneCount_; ++d) {
            int baseIndex = 2 + d * 3;
            auto drone_quat_to = compoundTo->as<ompl::base::SO3StateSpace::StateType>(baseIndex);
            drone_quat_to->w = last_state(offset++);
            drone_quat_to->x = last_state(offset++);
            drone_quat_to->y = last_state(offset++);
            drone_quat_to->z = last_state(offset++);

            auto drone_vel_to = compoundTo->as<ompl::base::RealVectorStateSpace::StateType>(baseIndex + 1);
            for (int i = 0; i < 3; ++i)
                drone_vel_to->values[i] = last_state(offset++);

            auto cable_to = compoundTo->as<ompl::base::RealVectorStateSpace::StateType>(baseIndex + 2);
            for (int i = 0; i < 4; ++i)
                cable_to->values[i] = last_state(offset++);
        }

        // 9. Set the payload position to the last position from the trajectory
        std::cout << "[DEBUG] Set payload position to: "
          << se3payload_to->getX() << ", "
          << se3payload_to->getY() << ", "
          << se3payload_to->getZ() << std::endl;

        // std::cout << "============= Finished postPropagate ===============" << std::endl;

    } catch (const py::error_already_set& e) {
        std::cerr << "[ACADOS ERROR] Caught Python exception: " << e.what() << std::endl;
        std::cerr << "    Skipping this propagation step. Copying previous state." << std::endl;
        // Simply copy the previous state to 'to' (no change)
        si_->getStateSpace()->copyState(to, from);
        // Optionally: set acceleration subspace to zero or previous, or log more info
        return;
    }
}


void ompl::app::PayloadSystem::setDefaultBounds()
{
    // Enforce payload position bounds (-300, 600) for x, y, z
    base::RealVectorBounds positionBounds(3); // SE3 position bounds
    positionBounds.setLow(0, -22); // x lower bound
    positionBounds.setHigh(0, 88); // x upper bound
    positionBounds.setLow(1, -53); // y lower bound
    positionBounds.setHigh(1, 50); // y upper bound
    positionBounds.setLow(2, -1); // z lower bound
    positionBounds.setHigh(2, 27); // z upper bound
    getStateSpace()->as<base::CompoundStateSpace>()->as<base::SE3StateSpace>(0)->setBounds(positionBounds);

    // Enforce payload velocity bounds (-10, 10) for x, y, z, and angular velocities
    base::RealVectorBounds velBounds(6);
    for(unsigned i=0;i<3;++i){velBounds.setLow(i,-maxPayloadVel);velBounds.setHigh(i,maxPayloadVel);}
    for(unsigned i=3;i<6;++i){velBounds.setLow(i,-maxPayloadAngVel);velBounds.setHigh(i,maxPayloadAngVel);}
    getStateSpace()->as<base::CompoundStateSpace>()->as<base::RealVectorStateSpace>(1)->setBounds(velBounds);
    

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

        cableAngleBounds.setLow(0, (90-maxTheta) * M_PI / 180); // Theta (index 0) lower bound in radians
        cableAngleBounds.setHigh(0, (90+maxTheta) * M_PI / 180); // Theta (index 0) upper bound in radians
        cableAngleBounds.setLow(1, -1e6);                   // No restriction on phi
        cableAngleBounds.setHigh(1, 1e6);
        cableAngleBounds.setLow(2, -maxThetaVel); // Theta_dot lower bound
        cableAngleBounds.setHigh(2, maxThetaVel); // Theta_dot upper bound
        cableAngleBounds.setLow(3, -maxThetaVel); // Phi_dot lower bound
        cableAngleBounds.setHigh(3, maxThetaVel); // Phi_dot upper bound
        getStateSpace()->as<base::CompoundStateSpace>()->as<base::RealVectorStateSpace>(4 + i * 3)->setBounds(cableAngleBounds);
    }
}


