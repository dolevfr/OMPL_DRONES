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

#include "AccelFourDrones.h"
#include "AccelClasses.h" // Include the AccelClasses header for the PayloadAccelControlSampler


unsigned int ompl::app::AccelPayloadSystem::droneCount_ = 4; // Default number of drones
namespace ob = ompl::base;
namespace oc = ompl::control;

using StepperType = boost::numeric::odeint::runge_kutta_cash_karp54<std::vector<double>>;

ompl::app::AccelPayloadSystem::AccelPayloadSystem()
    : AppBase<AppType::CONTROL>(constructControlSpace(), Motion_3D),   // pass valid pointer
      rigidBody_(ompl::app::MotionModel::Motion_3D, ompl::app::CollisionChecker::FCL)
{

    // Build state and control spaces only once and associate them with AppBase
    // stateSpace_ = std::dynamic_pointer_cast<ompl::base::CompoundStateSpace>(constructStateSpace());
    auto controlSpace = constructControlSpace();


    name_ = std::string("AccelPayloadSystem");
    std::cout << "[INFO] Initializing AccelPayloadSystem..." << std::endl;
    setDefaultBounds();
    std::cout << "[INFO] Default bounds set." << std::endl;

    si_->setPropagationStepSize(timeStep_);
    si_->setMinMaxControlDuration(1, 10);
    std::cout << "[INFO] Propagation step size: " << timeStep_ << std::endl;

    // Initialize Python and import acados_steer module
    py::module sys = py::module::import("sys");
    py::list sys_path = sys.attr("path");
    sys_path.append("../accel_based");  // adjust path as needed
    std::cout << "[INFO] Importing Python module 'acados_steer'..." << std::endl;
    acados_mod_ = py::module::import("acados_steer");
    std::cout << "[INFO] Setting up Acados OCP via Python..." << std::endl;
    py::tuple acados_result = acados_mod_.attr("setup_ocp")();
    acados_ocp_ = acados_result[0];
    accel_fun_ = acados_result[1];
    std::cout << "[INFO] Acados OCP and acceleration function initialized." << std::endl;
        
    // Setup propagator to use Acados
    std::cout << "[INFO] Setting state propagator to use Acados..." << std::endl;
    si_->setStatePropagator(
        [this](const ompl::base::State *from, const ompl::control::Control *control, const double /*duration*/, ompl::base::State *to) {
            propagateWithAcados(from, control, to);
        }
    );

    auto motionValidator = std::make_shared<ompl::base::DiscreteMotionValidator>(si_);
    si_->setMotionValidator(motionValidator);

    si_->setup();
    std::cout << "[INFO] AccelPayloadSystem setup complete." << std::endl;
}


ompl::base::ScopedState<> ompl::app::AccelPayloadSystem::getDefaultStartState() const
{
    base::ScopedState<base::SE3StateSpace> s(getGeometricComponentStateSpace());

    s->setXYZ(AccelPayloadSystem::startPosition_.x(), AccelPayloadSystem::startPosition_.y(), AccelPayloadSystem::startPosition_.z());
    s->rotation().setIdentity();
    return getFullStateFromGeometricComponent(s);
}


ompl::base::StateSpacePtr ompl::app::AccelPayloadSystem::constructStateSpace()
{
    if (!stateSpace_) {
        stateSpace_ = std::make_shared<base::CompoundStateSpace>();

        // Add SE3 state space for the payload (position and orientation)
        stateSpace_->addSubspace(std::make_shared<base::SE3StateSpace>(), 1.0);

        // Add RealVector state space for the payload's velocity (6 dimensions)
        stateSpace_->addSubspace(std::make_shared<base::RealVectorStateSpace>(6), 0.05);

        for (unsigned int i = 0; i < droneCount_; ++i)
        {
            // Add SO3 state space for orientation
            stateSpace_->addSubspace(std::make_shared<base::SO3StateSpace>(), 0.05);

            // Add RealVector state space for velocity (3 dimensions)
            stateSpace_->addSubspace(std::make_shared<base::RealVectorStateSpace>(3), 0.05);

            // Add RealVector state space for cable angles and velocities (4 dimensions)
            stateSpace_->addSubspace(std::make_shared<base::RealVectorStateSpace>(4), 0.05);
        }

        // Add RealVector state space for the payload's acceleration (3 dimensions)
        stateSpace_->addSubspace(std::make_shared<base::RealVectorStateSpace>(3), 0.01);

        stateSpace_->lock();
    }
    return stateSpace_;
}

ompl::control::ControlSpacePtr ompl::app::AccelPayloadSystem::constructControlSpace()
{
    std::cout << "[INFO] Constructing acceleration control space (dim=3)" << std::endl;

    // Control space dimension set to 3 for (ax, ay, az)
    auto controlSpace = std::make_shared<ompl::control::RealVectorControlSpace>(stateSpace_, 3);

    ompl::base::RealVectorBounds controlBounds(3);
    for (int i = 0; i < 3; ++i)
    {
        controlBounds.setLow(i, -10.0);  // or your acceleration bounds
        controlBounds.setHigh(i, 10.0);
    }

    controlSpace->as<ompl::control::RealVectorControlSpace>()->setBounds(controlBounds);

    return controlSpace;
}



// Utility: Get the index of the last subspace (acceleration)
int getAccelSubspaceIdx(const ompl::base::StateSpacePtr &space) {
    return space->as<ompl::base::CompoundStateSpace>()->getSubspaceCount() - 1;
}


void ompl::app::AccelPayloadSystem::propagateWithAcados(
    const ompl::base::State *from,
    const ompl::control::Control *control,
    ompl::base::State *to)
{
    auto compoundFrom = from->as<ompl::base::CompoundState>();
    int accelIdx = getAccelSubspaceIdx(si_->getStateSpace());

    int n_states = 57; // your system's number of states excluding accel

    Eigen::VectorXd current_state(n_states);

    int offset = 0;

    // Note the 'template' keyword here!
    auto se3payload = compoundFrom->template as<ompl::base::SE3StateSpace::StateType>(0);
    current_state[offset++] = se3payload->getX();
    current_state[offset++] = se3payload->getY();
    current_state[offset++] = se3payload->getZ();
    current_state[offset++] = se3payload->rotation().w;
    current_state[offset++] = se3payload->rotation().x;
    current_state[offset++] = se3payload->rotation().y;
    current_state[offset++] = se3payload->rotation().z;

    auto payload_vel = compoundFrom->template as<ompl::base::RealVectorStateSpace::StateType>(1);
    for (int i = 0; i < 6; ++i)
        current_state[offset++] = payload_vel->values[i];

    for (int d = 0; d < droneCount_; ++d)
    {
        int baseIndex = 2 + d * 3;
        auto drone_quat = compoundFrom->template as<ompl::base::SO3StateSpace::StateType>(baseIndex);
        current_state[offset++] = drone_quat->w;
        current_state[offset++] = drone_quat->x;
        current_state[offset++] = drone_quat->y;
        current_state[offset++] = drone_quat->z;

        auto drone_vel = compoundFrom->template as<ompl::base::RealVectorStateSpace::StateType>(baseIndex + 1);
        for (int i = 0; i < 3; ++i)
            current_state[offset++] = drone_vel->values[i];

        auto cable = compoundFrom->template as<ompl::base::RealVectorStateSpace::StateType>(baseIndex + 2);
        for (int i = 0; i < 4; ++i)
            current_state[offset++] = cable->values[i];
    }

    // // Print current_state nicely (payload and drones)



    // 2. Extract previous acceleration (from last subspace)
    auto prevAccelState = compoundFrom->template as<ompl::base::RealVectorStateSpace::StateType>(accelIdx);
    Eigen::Vector3d prev_accel(prevAccelState->values[0], prevAccelState->values[1], prevAccelState->values[2]);

    std::cout << "[DEBUG] Previous acceleration: ["
              << std::fixed << std::setprecision(6)
              << prev_accel(0) << ", "
              << prev_accel(1) << ", "
              << prev_accel(2) << "]" << std::endl;

    // 3. Get new acceleration from control (assuming 3D)
    auto ctrl = control->as<ompl::control::RealVectorControlSpace::ControlType>();
    Eigen::Vector3d next_accel(ctrl->values[0], ctrl->values[1], ctrl->values[2]);

    std::cout << "[DEBUG] Next acceleration (from control): ["
              << std::fixed << std::setprecision(6)
              << next_accel(0) << ", "
              << next_accel(1) << ", "
              << next_accel(2) << "]" << std::endl;

    // 4. Build 20-point linspace between prev_accel and next_accel
    Eigen::MatrixXd accel_traj(20, 3);
    for (int j = 0; j < 20; ++j)
    {
        double alpha = double(j) / 19.0;
        accel_traj.row(j) = prev_accel * (1.0 - alpha) + next_accel * alpha;
    }
    
    // 5. Convert to numpy array for Python
    py::array_t<double> py_state(current_state.size(), current_state.data());
    std::vector<ssize_t> shape = {20, 3};
    std::vector<ssize_t> strides = {sizeof(double), sizeof(double) * 20};

    py::array_t<double> py_accel_traj(shape, strides, accel_traj.data());

    // Call Acados to get trajectory
    py::tuple py_result = acados_mod_.attr("steer_acados")(acados_ocp_, accel_fun_, py_state, py_accel_traj);
    py::array_t<double> py_states = py_result[0].cast<py::array_t<double>>();
    py::array_t<double> py_controls = py_result[1].cast<py::array_t<double>>();

    // Extract last state and last acceleration from Python output
    auto buf_states = py_states.request();
    int n_steps = buf_states.shape[0];
    int n_s = buf_states.shape[1];
    double *states_ptr = static_cast<double *>(buf_states.ptr);
    Eigen::Map<Eigen::VectorXd> last_state(states_ptr + (n_steps - 1) * n_s, n_s);

    auto buf_controls = py_controls.request();
    int n_c = buf_controls.shape[1];
    double *controls_ptr = static_cast<double *>(buf_controls.ptr);
    Eigen::Map<Eigen::VectorXd> last_control(controls_ptr + (n_steps - 2) * n_c, n_c); 



    // 8. Copy last_state back into OMPL 'to' state (all subspaces except accel)
    auto compoundTo = to->as<ompl::base::CompoundState>();
    offset = 0;

    auto se3payload_to = compoundTo->template as<ompl::base::SE3StateSpace::StateType>(0);
    se3payload_to->setX(last_state(offset++));
    se3payload_to->setY(last_state(offset++));
    se3payload_to->setZ(last_state(offset++));
    se3payload_to->rotation().w = last_state(offset++);
    se3payload_to->rotation().x = last_state(offset++);
    se3payload_to->rotation().y = last_state(offset++);
    se3payload_to->rotation().z = last_state(offset++);

    auto payload_vel_to = compoundTo->template as<ompl::base::RealVectorStateSpace::StateType>(1);
    for (int i = 0; i < 6; ++i)
        payload_vel_to->values[i] = last_state(offset++);

    for (int d = 0; d < droneCount_; ++d)
    {
        int baseIndex = 2 + d * 3;
        auto drone_quat_to = compoundTo->template as<ompl::base::SO3StateSpace::StateType>(baseIndex);
        drone_quat_to->w = last_state(offset++);
        drone_quat_to->x = last_state(offset++);
        drone_quat_to->y = last_state(offset++);
        drone_quat_to->z = last_state(offset++);

        auto drone_vel_to = compoundTo->template as<ompl::base::RealVectorStateSpace::StateType>(baseIndex + 1);
        for (int i = 0; i < 3; ++i)
            drone_vel_to->values[i] = last_state(offset++);

        auto cable_to = compoundTo->template as<ompl::base::RealVectorStateSpace::StateType>(baseIndex + 2);
        for (int i = 0; i < 4; ++i)
            cable_to->values[i] = last_state(offset++);
    }

    assert(offset == n_states);

    // 9. Extract last acceleration from the last state vector
    // Assuming acceleration is stored in the last 3 entries of last_state
    Eigen::Vector3d last_acceleration(last_state[n_s - 3], last_state[n_s - 2], last_state[n_s - 1]);

    // 10. Set acceleration subspace in 'to' state to last acceleration
    auto accel_to = compoundTo->template as<ompl::base::RealVectorStateSpace::StateType>(accelIdx);
    accel_to->values[0] = last_acceleration(0);
    accel_to->values[1] = last_acceleration(1);
    accel_to->values[2] = last_acceleration(2);
}

void ompl::app::AccelPayloadSystem::setDefaultBounds()
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

    // Set bounds for the last subspace: previous acceleration [accel_x, accel_y, accel_z]
    unsigned int accelIdx = getStateSpace()->as<base::CompoundStateSpace>()->getSubspaceCount() - 1;
    base::RealVectorBounds accelBounds(3);
    for (int i = 0; i < 3; ++i) {
        accelBounds.setLow(i, -10.0);
        accelBounds.setHigh(i, 10.0);
    }
    getStateSpace()->as<base::CompoundStateSpace>()->as<base::RealVectorStateSpace>(accelIdx)->setBounds(accelBounds);
}


void ompl::app::AccelPayloadSystem::ode(
    const std::vector<double> &q,
    const ompl::control::Control *ctrl,
    std::vector<double> &qdot)
{
    // Not used in acceleration-based propagation.
    // You can leave it empty or throw if you want to catch misuse.
    qdot.resize(q.size());
    std::fill(qdot.begin(), qdot.end(), 0.0);
}

void ompl::app::AccelPayloadSystem::postPropagate(
    const ompl::base::State *state,
    const ompl::control::Control *control,
    double duration,
    ompl::base::State *result)
{
    // Not used in acceleration-based propagation.
    // Optionally, you could enforce bounds here, or just do nothing.
}





    // std::cout << "====================================" << std::endl;
    // std::cout << "[DEBUG] State vector before propagation:" << std::endl;

    // // Payload
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

    // // Drones
    // int drone_state_len = 11;
    // for (unsigned int i = 0; i < droneCount_; ++i)
    // {
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






        // // 6. Print last state and last control
    // std::cout << "====================================" << std::endl;
    // std::cout << "[DEBUG] State vector after propagation:" << std::endl;

    // // Payload
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

    // // Drones
    // for (unsigned int i = 0; i < droneCount_; ++i)
    // {
    //     int offset = 13 + i * drone_state_len;
    //     std::cout << "Drone " << (i+1) << " quat:      [";
    //     for (int j = 0; j < 4; ++j) std::cout << std::fixed << std::setprecision(4) << last_state(offset + j) << (j < 3 ? ", " : "");
    //     std::cout << "]" << std::endl;

    //     std::cout << "Drone " << (i+1) << " angvel:    [";
    //     for (int j = 4; j < 7; ++j) std::cout << std::fixed << std::setprecision(4) << last_state(offset + j) << (j < 6 ? ", " : "");
    //     std::cout << "]" << std::endl;

    //     std::cout << "Drone " << (i+1) << " θ, φ:       ("
    //               << std::fixed << std::setprecision(4) << last_state(offset + 7) << ", "
    //               << std::fixed << std::setprecision(4) << last_state(offset + 8) << ")" << std::endl;

    //     std::cout << "Drone " << (i+1) << " θ̇, φ̇:      ("
    //               << std::fixed << std::setprecision(4) << last_state(offset + 9) << ", "
    //               << std::fixed << std::setprecision(4) << last_state(offset + 10) << ")" << std::endl;
    // }