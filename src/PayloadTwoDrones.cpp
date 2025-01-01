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

    std::cout << "Printing from the constructor:\n";
    auto *compoundSpace = getStateSpace()->as<base::CompoundStateSpace>();
    for (unsigned int i = 0; i < compoundSpace->getSubspaceCount(); ++i)
    {
        std::cout << "Subspace " << i << ": Dimension = " << compoundSpace->getSubspace(i)->getDimension() << std::endl;
    }


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

    // Payload position and orientation
    stateSpace->addSubspace(std::make_shared<base::SE3StateSpace>(), 1.0);

    // Drone and cable orientations
    for (unsigned int i = 0; i < droneCount_; ++i)
    {
        stateSpace->addSubspace(std::make_shared<base::SO3StateSpace>(), 1.0); // Drone
        stateSpace->addSubspace(std::make_shared<base::SO3StateSpace>(), 1.0); // Cable
    }

    // Derivatives for payload, drones, and cables
    stateSpace->addSubspace(std::make_shared<base::RealVectorStateSpace>(6), 0.3); // Payload derivatives
    for (unsigned int i = 0; i < droneCount_; ++i)
    {
        stateSpace->addSubspace(std::make_shared<base::RealVectorStateSpace>(3), 0.3); // Drone derivatives
        stateSpace->addSubspace(std::make_shared<base::RealVectorStateSpace>(3), 0.3); // Cable derivatives
    }

    stateSpace->lock();

    auto *compoundSpace = std::dynamic_pointer_cast<base::CompoundStateSpace>(stateSpace).get();
    std::cout << "Total subspaces: " << compoundSpace->getSubspaceCount() << std::endl;
    for (unsigned int i = 0; i < compoundSpace->getSubspaceCount(); ++i)
    {
        std::cout << "Subspace " << i << ": " << compoundSpace->getSubspace(i)->getName() << std::endl;
    }

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
    base::RealVectorBounds posBounds(3);
    posBounds.setLow(-200);
    posBounds.setHigh(500);

    base::RealVectorBounds velBoundsPayload(6);
    velBoundsPayload.setLow(-1);
    velBoundsPayload.setHigh(1);

    base::RealVectorBounds velBoundsDroneCable(3);
    velBoundsDroneCable.setLow(-10.0);
    velBoundsDroneCable.setHigh(10.0);

    auto *compoundSpace = getStateSpace()->as<base::CompoundStateSpace>();

    // Payload bounds
    compoundSpace->as<base::SE3StateSpace>(0)->setBounds(posBounds);
    compoundSpace->getSubspace(5)->as<base::RealVectorStateSpace>()->setBounds(velBoundsPayload);

    // Drone and cable bounds
    for (unsigned int i = 0; i < droneCount_; ++i)
    {
        unsigned int droneDerivativeIndex = 6 + i * 2;
        unsigned int cableDerivativeIndex = 7 + i * 2;

        compoundSpace->getSubspace(droneDerivativeIndex)
            ->as<base::RealVectorStateSpace>()->setBounds(velBoundsDroneCable);
        compoundSpace->getSubspace(cableDerivativeIndex)
            ->as<base::RealVectorStateSpace>()->setBounds(velBoundsDroneCable);
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

// void ompl::app::PayloadSystem::postPropagate(const base::State* state, const control::Control* control, double duration, base::State* result)
// {
//     unsigned int payloadIndex = droneCount_ * 2; // SE3 subspace for payload

//     auto *payloadState = result->as<base::CompoundState>()->as<base::SE3StateSpace::StateType>(payloadIndex);
//     if (!payloadState)
//     {
//         throw ompl::Exception("Payload state is null or invalid.");
//     }

//     Eigen::Vector3d payloadPos(payloadState->getX(), payloadState->getY(), payloadState->getZ());
//     Eigen::Quaterniond payloadRot(
//         payloadState->rotation().w,
//         payloadState->rotation().x,
//         payloadState->rotation().y,
//         payloadState->rotation().z
//     );

//     double norm = payloadRot.norm();
//     if (norm > 1e-3 && norm < 1e3)
//     {
//         payloadRot.normalize();
//     }
//     else
//     {
//         OMPL_ERROR("Payload quaternion norm is invalid: %f. Resetting to identity.", norm);
//         payloadRot = Eigen::Quaterniond::Identity();
//     }

//     for (unsigned int i = 0; i < droneCount_; ++i)
//     {
//         unsigned int droneIndex = i * 2;

//         auto *droneState = result->as<base::CompoundState>()->as<base::SE3StateSpace::StateType>(droneIndex);
//         if (!droneState)
//         {
//             throw ompl::Exception("Drone state is null or invalid.");
//         }

//         Eigen::Vector3d dronePos(droneState->getX(), droneState->getY(), droneState->getZ());
//         Eigen::Quaterniond droneRot(
//             droneState->rotation().w,
//             droneState->rotation().x,
//             droneState->rotation().y,
//             droneState->rotation().z
//         );

//         norm = droneRot.norm();
//         if (norm > 1e-3 && norm < 1e3)
//         {
//             droneRot.normalize();
//         }
//         else
//         {
//             OMPL_ERROR("Drone %u quaternion norm is invalid: %f. Resetting to identity.", i, norm);
//             droneRot = Eigen::Quaterniond::Identity();
//         }

//         Eigen::Vector3d cornerPos = payloadPos + payloadRot * getPayloadCorner(i);
//         if (cornerPos.norm() > 1e6 || std::isnan(cornerPos.norm()) || std::isinf(cornerPos.norm()))
//         {
//             OMPL_ERROR("Corner position for Drone %u is invalid. Skipping processing.", i);
//             continue;
//         }

//         Eigen::Vector3d cableVec = cornerPos - dronePos;
//         double cableLength = cableVec.norm();

//         if (cableLength > 1e3 || std::isnan(cableLength) || std::isinf(cableLength))
//         {
//             OMPL_ERROR("Cable length for Drone %u is invalid: %f. Resetting to default.", i, cableLength);
//             cableLength = l;
//             continue;
//         }

//         OMPL_DEBUG("Drone %u Cable Vector: (%f, %f, %f), Cable Length: %f", 
//                    i, cableVec.x(), cableVec.y(), cableVec.z(), cableLength);

//         if (std::abs(cableLength - l) > 1e-4)
//         {
//             OMPL_WARN("Cable length constraint violated for Drone %u. Expected: %f, Actual: %f", i, l, cableLength);
//         }
//     }
// }






