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

#include "AccelPidFourDrones.h"

unsigned int ompl::app::PayloadSystem::droneCount_ = 4; // Default number of drones

using StepperType = boost::numeric::odeint::runge_kutta_cash_karp54<std::vector<double>>;

ompl::app::PayloadSystem::PayloadSystem()
    : AppBase<AppType::CONTROL>(constructControlSpace(), Motion_3D), 
      rigidBody_(ompl::app::MotionModel::Motion_3D, ompl::app::CollisionChecker::FCL)
{

    // setControlSamplerAllocator must be called on the control space used by this system!
    getControlSpace()->as<ompl::control::RealVectorControlSpace>()->setControlSamplerAllocator(
        [this](const ompl::control::ControlSpace *space)
        {
            return std::make_shared<AccelDeltaControlSampler>(space, this->accelMaxDelta, this->zScale);
        });

    py::module sys = py::module::import("sys");
    py::list sys_path = sys.attr("path");
    sys_path.append("../accel_pid");   // adjust to folder where cable_opt.py lives

    
    name_ = std::string("PayloadSystem");
    setDefaultBounds();

    si_->setPropagationStepSize(timeStep_);
    si_->setMinMaxControlDuration(1, 10);

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

    stateSpace->addSubspace(std::make_shared<base::SE3StateSpace>(), 1.0);
    stateSpace->addSubspace(std::make_shared<base::RealVectorStateSpace>(6), 0.05);

    for (unsigned int i = 0; i < droneCount_; ++i)
    {
        stateSpace->addSubspace(std::make_shared<base::SO3StateSpace>(), 0.05);
        stateSpace->addSubspace(std::make_shared<base::RealVectorStateSpace>(3), 0.05);
        stateSpace->addSubspace(std::make_shared<base::RealVectorStateSpace>(4), 0.05);
    }

    stateSpace->addSubspace(std::make_shared<base::RealVectorStateSpace>(3), 0.01);
    stateSpace->lock();

    return stateSpace;
}



ompl::control::ControlSpacePtr ompl::app::PayloadSystem::constructControlSpace()
{
    // Construct a control space with dimensions equal to 4 times the number of drones
    auto controlSpace = std::make_shared<ompl::control::RealVectorControlSpace>(constructStateSpace(), 3);

    // Define control bounds
    ompl::base::RealVectorBounds controlBounds(3); // 4 controls per drone

    // Set bounds for each drone
    for (int i = 0; i < 3; ++i)
    {
        controlBounds.setLow(i, -10.0);  // or your acceleration bounds
        controlBounds.setHigh(i, 10.0);
    }

    // Apply bounds to the control space
    controlSpace->as<ompl::control::RealVectorControlSpace>()->setBounds(controlBounds);

    return controlSpace;
}


void ompl::app::PayloadSystem::ode_free(const std::vector<double> &q,
                                        const Eigen::VectorXd &u_cmd,
                                        std::vector<double> &qdot)
{
    qdot.assign(q.size(), 0.0);

    unsigned int droneStateSize = 11; // State size per drone and cable
    unsigned int droneIndex = 13;     // Start index for drones

    // Payload position and orientation
    Eigen::Vector3d payloadPos(q[0], q[1], q[2]);
    Eigen::Quaterniond payloadRot(q[3], q[4], q[5], q[6]);
    Eigen::Vector3d payloadVel(q[7], q[8], q[9]);
    Eigen::Vector3d payloadAngVel(q[10], q[11], q[12]);

    // Initialize payload force and torque
    Eigen::Vector3d payloadForce(0, 0, -m_payload * 9.81); // Gravity force on the payload
    Eigen::Vector3d payloadTorque(0, 0, 0);                // Accumulated torque on the payload

    for (unsigned int i = 0; i < droneCount_; ++i)
    {
        unsigned int baseIndex = droneIndex + i * droneStateSize; // Start index for drone i

        // Drone orientation and angular velocity
        Eigen::Quaterniond droneRot(q[baseIndex + 0], q[baseIndex + 1], q[baseIndex + 2], q[baseIndex + 3]);
        Eigen::Vector3d omega(q[baseIndex + 4], q[baseIndex + 5], q[baseIndex + 6]);

        // Cables angles and velocities
        double theta = q[baseIndex + 7];
        double phi = q[baseIndex + 8];
        double thetaDot = q[baseIndex + 9];
        double phiDot = q[baseIndex + 10];

        // Thrust direction
        Eigen::Vector3d thrustDir = droneRot * Eigen::Vector3d(0, 0, 1);

        // Retrieve thrust magnitude and torque from control input vector u_cmd
        double thrustMagnitude = u_cmd[i * 4 + 0];
        Eigen::Vector3d torque(u_cmd[i * 4 + 1], u_cmd[i * 4 + 2], u_cmd[i * 4 + 3]);

        // Net force on the drone
        Eigen::Vector3d droneForce = thrustMagnitude * thrustDir + m_drone * Eigen::Vector3d(0, 0, -9.81);

        // Cable unit vectors
        Eigen::Vector3d cableDir = Eigen::Vector3d(sin(theta) * cos(phi), -cos(theta), sin(theta) * sin(phi));
        Eigen::Vector3d thetaDir = Eigen::Vector3d(cos(theta) * cos(phi), sin(theta), cos(theta) * sin(phi));
        Eigen::Vector3d phiDir = Eigen::Vector3d(-sin(phi), 0, cos(phi));

        Eigen::Vector3d forceOnCable = droneForce.dot(cableDir) * cableDir;
        double forceTheta = droneForce.dot(thetaDir);
        double forcePhi = droneForce.dot(phiDir);

        Eigen::Quaterniond omega_quat(0, omega.x(), omega.y(), omega.z());
        Eigen::Quaterniond q_dot = Eigen::Quaterniond(0.5 * omega_quat.coeffs()) * droneRot;

        // Angular acceleration calculation
        Eigen::Vector3d angularAccel = droneInertia.inverse() * (torque - omega.cross(droneInertia * omega) - droneBeta * omega);

        // Update qdot for quaternion
        qdot[baseIndex + 0] = q_dot.w();
        qdot[baseIndex + 1] = q_dot.x();
        qdot[baseIndex + 2] = q_dot.y();
        qdot[baseIndex + 3] = q_dot.z();

        // Update qdot for angular accelerations
        qdot[baseIndex + 4] = angularAccel.x();
        qdot[baseIndex + 5] = angularAccel.y();
        qdot[baseIndex + 6] = angularAccel.z();

        // Update angular velocities of theta and phi
        qdot[baseIndex + 7] = thetaDot;
        qdot[baseIndex + 8] = phiDot;
        qdot[baseIndex + 9] = forceTheta / (l * m_drone) + phiDot * phiDot * sin(theta) * cos(theta);
        qdot[baseIndex + 10] = (forcePhi / (l * m_drone) - 2 * thetaDot * phiDot * cos(theta)) / sin(theta);

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
    Eigen::Vector3d payloadAngAccel = payloadInertia.inverse() * (payloadTorque - payloadAngVel.cross(payloadInertia * payloadAngVel));

    // Update qdot for payload indices
    qdot[0] = payloadVel.x();
    qdot[1] = payloadVel.y();
    qdot[2] = payloadVel.z();

    qdot[3] = q_dot_p.w();
    qdot[4] = q_dot_p.x();
    qdot[5] = q_dot_p.y();
    qdot[6] = q_dot_p.z();

    qdot[7] = payloadAccel.x();
    qdot[8] = payloadAccel.y();
    qdot[9] = payloadAccel.z();

    qdot[10] = payloadAngAccel.x();
    qdot[11] = payloadAngAccel.y();
    qdot[12] = payloadAngAccel.z();
}

void ompl::app::PayloadSystem::computeDroneCommandsP2(
    const std::vector<double>&          T_refs,
    const std::vector<Eigen::Quaterniond>& q_refs,
    const Eigen::VectorXd&              current_state,
    Eigen::VectorXd&                    u_cmd) const
{
    u_cmd.resize(4 * droneCount_);
    u_cmd.setZero();

    const int drone_state_len = 11;

    for (unsigned int i = 0; i < droneCount_; ++i)
    {
        // Current drone orientation and angular velocity
        int off = 13 + i * drone_state_len;
        Eigen::Quaterniond q_curr(
            current_state[off+0], current_state[off+1],
            current_state[off+2], current_state[off+3]
        );
        q_curr.normalize();
        Eigen::Vector3d omega_curr(
            current_state[off+4],
            current_state[off+5],
            current_state[off+6]
        );

        // Reference quaternion and thrust
        const Eigen::Quaterniond& q_des = q_refs[i];
        const double T_des = T_refs[i];

        // Quaternion error
        Eigen::Quaterniond q_err = q_curr.conjugate() * q_des;
        Eigen::Vector3d e_q = -q_err.vec();
        if (q_err.w() < 0.0) e_q = -e_q;  // shortest rotation

        // P² torque law: τ = -Pq * e_q - Pw * ω
        Eigen::Vector3d tau_ctrl = -Pq * e_q - Pw * omega_curr;

        // Pack output
        u_cmd[4*i + 0] = T_des;
        u_cmd[4*i + 1] = tau_ctrl.x();
        u_cmd[4*i + 2] = tau_ctrl.y();
        u_cmd[4*i + 3] = tau_ctrl.z();
    }
}



void ompl::app::PayloadSystem::integrateOnceRK4(
    const Eigen::VectorXd &q_eig,
    const Eigen::VectorXd &u_cmd,
    double dt,
    std::function<void(const std::vector<double>&,
                       const Eigen::VectorXd&,
                       std::vector<double>&)> rhs,
    Eigen::VectorXd &q_next_eig,
    Eigen::Vector3d &payloadAccel_out)
{
    using StateType = std::vector<double>;
    const std::size_t n = static_cast<std::size_t>(q_eig.size());

    StateType q(n);
    for (std::size_t i = 0; i < n; ++i)
        q[i] = q_eig[(Eigen::Index)i];

    StateType tmp(n), k1(n), k2(n), k3(n), k4(n);

    rhs(q, u_cmd, k1);
    for (std::size_t i = 0; i < n; ++i) tmp[i] = q[i] + 0.5 * dt * k1[i];
    rhs(tmp, u_cmd, k2);
    for (std::size_t i = 0; i < n; ++i) tmp[i] = q[i] + 0.5 * dt * k2[i];
    rhs(tmp, u_cmd, k3);
    for (std::size_t i = 0; i < n; ++i) tmp[i] = q[i] + dt * k3[i];
    rhs(tmp, u_cmd, k4);

    StateType q_next(n);
    for (std::size_t i = 0; i < n; ++i)
        q_next[i] = q[i] + (dt/6.0) * (k1[i] + 2*k2[i] + 2*k3[i] + k4[i]);

    q_next_eig.resize((Eigen::Index)n);
    for (std::size_t i = 0; i < n; ++i)
        q_next_eig[(Eigen::Index)i] = q_next[i];

    payloadAccel_out = Eigen::Vector3d(k1[7], k1[8], k1[9]);
}


void ompl::app::PayloadSystem::postPropagate(const base::State *from, const control::Control *control, double /*duration*/, base::State *to)
{
    std::cout << "\n============= Started postPropagate ===============" << std::endl;

    // Cast to compound state
    auto compoundFrom = from->as<ompl::base::CompoundState>();
    if (!compoundFrom) {
        std::cerr << "[ERROR] postPropagate: from-state is not a CompoundState!" << std::endl;
        si_->getStateSpace()->copyState(to, from);
        return;
    }

    auto compoundSpace = si_->getStateSpace()->as<ompl::base::CompoundStateSpace>();
    if (!compoundSpace) {
        std::cerr << "[ERROR] postPropagate: StateSpace is not a CompoundStateSpace!" << std::endl;
        si_->getStateSpace()->copyState(to, from);
        return;
    }

    int accelIdx = compoundSpace->getSubspaceCount() - 1;
    // std::cout << "[DEBUG] Subspace count: " << compoundSpace->getSubspaceCount()
    //           << " (accelIdx=" << accelIdx << ")" << std::endl;

    // Expected flat dimension (without accel): 13 payload + 4*11 = 57
    int n_states = 57;
    Eigen::VectorXd current_state(n_states);
    int offset = 0;

    // [0] Payload SE3 (7)
    auto se3payload = compoundFrom->as<ompl::base::SE3StateSpace::StateType>(0);
    std::cout << "[DEBUG] Payload pos from SE3: ("
              << se3payload->getX() << ", "
              << se3payload->getY() << ", "
              << se3payload->getZ() << ")" << std::endl;
    std::cout << "[DEBUG] Payload quat (wxyz): ("
              << se3payload->rotation().w << ", "
              << se3payload->rotation().x << ", "
              << se3payload->rotation().y << ", "
              << se3payload->rotation().z << ")" << std::endl;

    current_state[offset++] = se3payload->getX();
    current_state[offset++] = se3payload->getY();
    current_state[offset++] = se3payload->getZ();
    current_state[offset++] = se3payload->rotation().w;
    current_state[offset++] = se3payload->rotation().x;
    current_state[offset++] = se3payload->rotation().y;
    current_state[offset++] = se3payload->rotation().z;

    // [1] Payload velocity (6)
    auto payload_vel = compoundFrom->as<ompl::base::RealVectorStateSpace::StateType>(1);
    // Copy payload velocity values into current_state and increment offset separately
    for (int i = 0; i < 6; ++i) {
        current_state[offset + i] = payload_vel->values[i];
    }
    offset += 6;

    // // Then, print the payload velocity
    // std::cout << "[DEBUG] Payload velocity: [";
    // for (int i = 0; i < 6; ++i) {
    //     std::cout << payload_vel->values[i] << (i < 5 ? ", " : "");
    // }
    // std::cout << "]" << std::endl;

    // [2..13]: 4 drones, each [SO3][R3][R4]
    for (int d = 0; d < droneCount_; ++d) {
        int base = 2 + d*3;

        // SO3 (4)
        auto drone_quat = compoundFrom->as<ompl::base::SO3StateSpace::StateType>(base);
        // std::cout << "[DEBUG] Drone " << d
        //           << " quat (wxyz): ("
        //           << drone_quat->w << ", "
        //           << drone_quat->x << ", "
        //           << drone_quat->y << ", "
        //           << drone_quat->z << ")" << std::endl;
        current_state[offset++] = drone_quat->w;
        current_state[offset++] = drone_quat->x;
        current_state[offset++] = drone_quat->y;
        current_state[offset++] = drone_quat->z;

        // R3 (drone velocity)
        auto drone_vel = compoundFrom->as<ompl::base::RealVectorStateSpace::StateType>(base + 1);
        // Increment offset using drone velocity values first
        for (int i = 0; i < 3; ++i)
            current_state[offset + i] = drone_vel->values[i];
        offset += 3;
        // Then print drone angular velocity
        // std::cout << "[DEBUG] Drone " << d << " angvel: [";
        // for (int i = 0; i < 3; ++i) {
        //     std::cout << drone_vel->values[i] << (i < 2 ? ", " : "");
        // }
        // std::cout << "]" << std::endl;

        // R4 (cable)
        auto cable = compoundFrom->as<ompl::base::RealVectorStateSpace::StateType>(base + 2);
        // Increment offset using cable values first
        for (int i = 0; i < 4; ++i)
            current_state[offset + i] = cable->values[i];
        offset += 4;
        // Then print cable values
        // std::cout << "[DEBUG] Drone " << d << " cable (θ, φ, θ̇, φ̇): [";
        // for (int i = 0; i < 4; ++i) {
        //     std::cout << cable->values[i] << (i < 3 ? ", " : "");
        // }
        // std::cout << "]" << std::endl;
    }

    // std::cout << "[DEBUG] Finished extracting state, offset=" << offset
    //           << " (expected " << n_states << ")" << std::endl;

    assert(offset == n_states && "Mismatch in state extraction size!");
    // std::cout << "[DEBUG] Extracted full state vector of size " << n_states << std::endl;

    // std::cout << "\n========== CURRENT STATE ==========" << std::endl;
    // printState(current_state);

    // 2. Previous acceleration (3D, last subspace)
    auto prevAccelState = compoundFrom->as<ompl::base::RealVectorStateSpace::StateType>(accelIdx);
    Eigen::Vector3d prev_accel(
        prevAccelState->values[0],
        prevAccelState->values[1],
        prevAccelState->values[2]
    );
    std::cout << "[DEBUG] Previous acceleration: " << prev_accel.transpose() << std::endl;

    // 3. Next acceleration from control input
    auto ctrl = control->as<ompl::control::RealVectorControlSpace::ControlType>();
    Eigen::Vector3d next_accel(ctrl->values[0], ctrl->values[1], ctrl->values[2]);
    std::cout << "[DEBUG] Next acceleration (from control): " << next_accel.transpose() << std::endl;

    // Eigen::Vector3d next_accel(1, 1, -1); // Placeholder for testing
    // std::cout << "[DEBUG] Next acceleration (from control): " << next_accel.transpose() << std::endl;

    // --- Compute F_req (uses payload damping & gravity as in your model) ---
    Eigen::Vector3d v_payload(current_state[7], current_state[8], current_state[9]);
    Eigen::Vector3d F_req = m_payload * next_accel
                        + m_payload * Eigen::Vector3d(0, 0, 9.81)
                        + payloadBeta * v_payload;
    // std::cout << "[DEBUG] Computed F_req: " << F_req.transpose() << std::endl;

    // --- Payload orientation & relative corners (no +payload_pos here) ---
    Eigen::Quaterniond q_payload(current_state[3], current_state[4], current_state[5], current_state[6]);
    q_payload.normalize();
    Eigen::Matrix3d R_p = q_payload.toRotationMatrix();

    std::vector<Eigen::Vector3d> corners_local = {
        { -w / 2, -d / 2,  h / 2 },
        {  w / 2, -d / 2,  h / 2 },
        {  w / 2,  d / 2,  h / 2 },
        { -w / 2,  d / 2,  h / 2 }
    };
    std::array<Eigen::Vector3d,4> corners_rel_world;
    for (int i=0; i<4; ++i)
        corners_rel_world[i] = R_p * corners_local[i];

    // Initial guess x0 (4 tensions + 8 angles)
    double x0[12];
    for (int i=0; i<4; ++i) x0[i] = 50.0;
    for (int d=0; d<4; ++d) {
        x0[4 + 2*d]   = current_state[13 + d*11 + 7]; // θ
        x0[4 + 2*d+1] = current_state[13 + d*11 + 8]; // φ
    }

    // pybind11 imports
    py::module cable_opt_mod = py::module::import("cable_opt");

    // Convert Eigen data to Python
    py::list py_Freq;
    for (int i=0; i<3; ++i) py_Freq.append(F_req[i]);

    py::list py_corners;
    for (int i=0; i<4; ++i) {
        py::list corner;
        for (int j=0; j<3; ++j) corner.append(corners_rel_world[i][j]);
        py_corners.append(corner);
    }

    try{

        // Call Python function
        py::object result = cable_opt_mod.attr("cable_opt")(py_Freq, py_corners, mu, lambda);

        // Unpack results
        auto t_opt    = result.cast<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>>();
        std::vector<double> tensions = std::get<0>(t_opt);
        std::vector<double> thetas   = std::get<1>(t_opt);
        std::vector<double> phis     = std::get<2>(t_opt);

        // // Print results
        // std::cout << "[TENSIONS] ";
        // for (auto t : tensions) std::cout << t << " ";
        // std::cout << std::endl;

        // for (int d=0; d<4; ++d) {
        //     std::cout << "[ANGLE] drone " << d
        //             << " θ=" << thetas[d]
        //             << " φ=" << phis[d] << std::endl;
        // }

        // --- Build x = [t1..t4, θ1,φ1,...,θ4,φ4] ---
        Eigen::VectorXd ref(12);
        for (int i = 0; i < 4; ++i) {
            ref[i] = tensions[i];                 // t_i from cable_opt
            ref[4 + 2*i]   = thetas[i];           // θ_i from cable_opt
            ref[4 + 2*i+1] = phis[i];             // φ_i from cable_opt
        }

        // --- Precompute drone references: thrusts and desired quaternions ---
        std::vector<double> T_refs(droneCount_);
        std::vector<Eigen::Quaterniond> q_refs(droneCount_);

        const double g = 9.81;

        for (unsigned int i = 0; i < droneCount_; ++i)
        {
            // Cable direction from θ, φ in ref
            const double theta_opt = ref[4 + 2*i];
            const double phi_opt   = ref[4 + 2*i + 1];
            Eigen::Vector3d s_i(std::sin(theta_opt)*std::cos(phi_opt),
                                -std::cos(theta_opt),
                                std::sin(theta_opt)*std::sin(phi_opt));

            // Desired net force
            const double t_i = ref[i];  // tension
            const Eigen::Vector3d F_des = t_i * s_i + m_drone * Eigen::Vector3d(0,0,g);
            const double T_des = F_des.norm();
            const Eigen::Vector3d b3_des = (T_des > 1e-9) ? F_des / T_des
                                                        : Eigen::Vector3d(0,0,1);

            // Current orientation (for projection axis)
            int off = 13 + i * 11; // 11 states per drone
            Eigen::Quaterniond q_curr(
                current_state[off+0], current_state[off+1],
                current_state[off+2], current_state[off+3]
            );
            q_curr.normalize();
            Eigen::Matrix3d R_curr = q_curr.toRotationMatrix();

            // Build R_des with z-axis = b3_des
            Eigen::Vector3d ex_proj = R_curr.col(0) - (R_curr.col(0).dot(b3_des))*b3_des;
            if (ex_proj.norm() < 1e-6)
                ex_proj = Eigen::Vector3d(1,0,0) - (Eigen::Vector3d(1,0,0).dot(b3_des))*b3_des;
            Eigen::Vector3d b1_des = ex_proj.normalized();
            Eigen::Vector3d b2_des = b3_des.cross(b1_des).normalized();
            b1_des = b2_des.cross(b3_des);
            Eigen::Matrix3d R_des; R_des << b1_des, b2_des, b3_des;
            Eigen::Quaterniond q_des(R_des);

            // Store results
            T_refs[i]  = T_des;
            q_refs[i]  = q_des;
        }

        // // Print the z-axis direction for each drone from q_refs
        // std::cout << "\n[DEBUG] Drone z-axis reference orientations:" << std::endl;
        // for (unsigned int i = 0; i < droneCount_; ++i)
        // {
        //     Eigen::Matrix3d R = q_refs[i].toRotationMatrix();
        //     Eigen::Vector3d z_axis = R.col(2); // z-axis corresponds to the third column
        //     std::cout << "[DRONE " << i << "] z-axis: " << z_axis.transpose() << std::endl;
        // }


        // ================== CONTROL LOOP ==================
        Eigen::Vector3d payloadAccel;
        Eigen::VectorXd next_state;

        for (int iter = 0; iter < n_iter; ++iter) {

            // --- Compute control inputs for each drone ---
            Eigen::VectorXd u_cmd;
            computeDroneCommandsP2(T_refs, q_refs, current_state, u_cmd);  // u_cmd has size 4*droneCount_
            // std::cout << "[DEBUG] Computed control inputs:" << std::endl;
            // for (unsigned int i = 0; i < droneCount_; ++i) {
            //     std::cout << "[DRONE " << i << "] Thrust: " << u_cmd[4 * i]
            //               << ", Torque: (" << u_cmd[4 * i + 1] << ", "
            //               << u_cmd[4 * i + 2] << ", " << u_cmd[4 * i + 3] << ")"
            //               << std::endl;
            // }

            // --- Advance the dynamics one step with RK4 ---
            integrateOnceRK4(
                current_state,
                u_cmd,        // pass u_cmd directly
                timeStep_,
                [this](const std::vector<double> &x,
                    const Eigen::VectorXd &u_cmd,
                    std::vector<double> &xdot) {
                    this->ode_free(x, u_cmd, xdot);
                },
                next_state,
                payloadAccel
            );

            // // Logging
            // std::cout << "\n[ITER " << iter << "] Next state:" << std::endl;
            // printState(next_state);
            // std::cout << "[ITER " << iter << "] Payload accel: "
            //         << payloadAccel.transpose() << std::endl;

            // Update state for next iteration
            current_state = next_state;
        }

        Eigen::Map<Eigen::VectorXd> last_state(current_state.data(), current_state.size());

        // std::cout << "\n========== LAST STATE ==========" << std::endl;
        // printState(last_state);

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

        // 9. Set acceleration subspace in 'to' state to the last acceleration
        auto accel_to = compoundTo->as<ompl::base::RealVectorStateSpace::StateType>(accelIdx);
        for (int i = 0; i < 3; ++i)
            accel_to->values[i] = payloadAccel[i];

        std::cout << "[DEBUG] Set acceleration to: "
              << accel_to->values[0] << ", "
              << accel_to->values[1] << ", "
              << accel_to->values[2] << std::endl;

        std::cout << "============= Finished postPropagate ===============" << std::endl;

    } catch (const std::exception &e) {
        std::cerr << "[ERROR] Caught exception: " << e.what() << std::endl;
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

    base::RealVectorBounds accelBounds(3);
    accelBounds.setLow(-10);
    accelBounds.setHigh(10);
    getStateSpace()->as<base::CompoundStateSpace>()->as<base::RealVectorStateSpace>(getStateSpace()->as<base::CompoundStateSpace>()->getSubspaceCount() - 1)->setBounds(accelBounds);
}

void ompl::app::PayloadSystem::printState(const Eigen::VectorXd &state) const
{
    // Payload
    std::cout << "Payload pos:    [";
    for (int i = 0; i < 3; ++i)
        std::cout << std::fixed << std::setprecision(4) << state(i) << (i < 2 ? ", " : "");
    std::cout << "]" << std::endl;

    std::cout << "Payload quat:   [";
    for (int i = 3; i < 7; ++i)
        std::cout << std::fixed << std::setprecision(4) << state(i) << (i < 6 ? ", " : "");
    std::cout << "]" << std::endl;

    std::cout << "Payload vel:    [";
    for (int i = 7; i < 10; ++i)
        std::cout << std::fixed << std::setprecision(4) << state(i) << (i < 9 ? ", " : "");
    std::cout << "]" << std::endl;

    std::cout << "Payload angvel: [";
    for (int i = 10; i < 13; ++i)
        std::cout << std::fixed << std::setprecision(4) << state(i) << (i < 12 ? ", " : "");
    std::cout << "]" << std::endl;

    // Drones
    for (unsigned int i = 0; i < droneCount_; ++i) {
        int offset = 13 + i * drone_state_len;

        std::cout << "Drone " << (i+1) << " quat:      [";
        for (int j = 0; j < 4; ++j)
            std::cout << std::fixed << std::setprecision(4) << state(offset + j) << (j < 3 ? ", " : "");
        std::cout << "]" << std::endl;

        std::cout << "Drone " << (i+1) << " angvel:    [";
        for (int j = 4; j < 7; ++j)
            std::cout << std::fixed << std::setprecision(4) << state(offset + j) << (j < 6 ? ", " : "");
        std::cout << "]" << std::endl;

        std::cout << "Drone " << (i+1) << " θ, φ:       ("
                  << std::fixed << std::setprecision(4) << state(offset + 7) << ", "
                  << std::fixed << std::setprecision(4) << state(offset + 8) << ")" << std::endl;

        std::cout << "Drone " << (i+1) << " θ̇, φ̇:      ("
                  << std::fixed << std::setprecision(4) << state(offset + 9) << ", "
                  << std::fixed << std::setprecision(4) << state(offset + 10) << ")" << std::endl;
    }
}
