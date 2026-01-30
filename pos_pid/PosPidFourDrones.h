#ifndef OMPL_APP_PAYLOAD_SYSTEM_H
#define OMPL_APP_PAYLOAD_SYSTEM_H

#include <array>
#include <boost/numeric/odeint.hpp>
#include <boost/filesystem.hpp>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <chrono>
#include <Eigen/Dense>
#include <fstream>
#include <filesystem>
#include <iostream>

#include <ompl/base/DiscreteMotionValidator.h>
#include <ompl/base/goals/GoalRegion.h>

#include <ompl/base/objectives/ControlDurationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/MechanicalWorkOptimizationObjective.h>
#include <ompl/base/objectives/MinimaxObjective.h>
#include <ompl/base/OptimizationObjective.h>

#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/terminationconditions/IterationTerminationCondition.h>

#include <ompl/control/DirectedControlSampler.h>
#include <ompl/control/PlannerData.h>
#include <ompl/control/PlannerDataStorage.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/PathControl.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/sst/SST.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/spaces/RealVectorControlSpace.h> 

#include <omplapp/apps/AppBase.h>

#include <pybind11/embed.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include <random>
#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/NumericalDiff>
#include <vector>

namespace py = pybind11;

namespace ompl
{
    namespace app
    {
        /** \brief A class for planning with a payload system suspended by drones */
        class PayloadSystem : public AppBase<AppType::CONTROL>
        {
        public:
            PayloadSystem();

            ~PayloadSystem() override = default;

            // struct CableResidualCeres {
            //     CableResidualCeres(const Eigen::Vector3d& Freq,
            //                     const std::array<Eigen::Vector3d,4>& corners,
            //                     double mu, double lambda)
            //         : F_req(Freq), r(corners), mu(mu), lambda(lambda) {}

            //     template<typename T>
            //     bool operator()(const T* const x, T* residual) const {
            //         // x[0..3] = tensions t_i
            //         // x[4..11] = theta_i, phi_i

            //         Eigen::Matrix<T,3,1> F = Eigen::Matrix<T,3,1>::Zero();
            //         Eigen::Matrix<T,3,1> Tau = Eigen::Matrix<T,3,1>::Zero();

            //         for (int i=0; i<4; ++i) {
            //             T t     = x[i];
            //             T theta = x[4 + 2*i];
            //             T phi   = x[4 + 2*i + 1];

            //             Eigen::Matrix<T,3,1> s;
            //             s << sin(theta)*cos(phi),
            //                 -cos(theta),
            //                 sin(theta)*sin(phi);

            //             F   += t * s;
            //             Tau += r[i].cast<T>().cross(t * s);
            //         }

            //         // Residuals:
            //         residual[0] = F(0) - T(F_req(0));
            //         residual[1] = F(1) - T(F_req(1));
            //         residual[2] = F(2) - T(F_req(2));

            //         residual[3] = T(std::sqrt(mu)) * Tau(0);
            //         residual[4] = T(std::sqrt(mu)) * Tau(1);
            //         residual[5] = T(std::sqrt(mu)) * Tau(2);

            //         for (int i=0; i<4; ++i)
            //             residual[6+i] = T(std::sqrt(lambda)) * x[i];

            //         return true;
            //     }

            //     const Eigen::Vector3d F_req;
            //     const std::array<Eigen::Vector3d,4> r;
            //     const double mu, lambda;
            // };


            void printState(const Eigen::VectorXd &state) const;


            /** \brief Check if self-collision is enabled (not used for payload systems) */
            bool isSelfCollisionEnabled() const override { return false; }

            /** \brief Segmentation fault otherwise */
            void inferProblemDefinitionBounds() override {}

            const Eigen::Vector3d& getStartPosition() const { return startPosition_; }
            const Eigen::Vector3d& getGoalPosition() const { return goalPosition_; }

            double getDroneMass() const { return m_drone; }
            double getPayloadMass() const { return m_payload; }

            double getPayloadWidth() const { return w; }
            double getPayloadDepth() const { return d; }
            double getPayloadHeight() const { return h; }
            double getCableLength() const { return l; }

            unsigned int getRobotCount() const override { return droneCount_; }
            double getMaxTorquePitchRoll() const { return maxTorquePitchRoll; }
            double getMaxTorqueYaw() const { return maxTorqueYaw; }
            double getMinThrust() const { return minThrust; }
            double getMaxThrust() const { return maxThrust; }

            double getThrustStd() const { return thrustStd; }
            double getTorquePitchRollStd() const { return torquePitchRollStd; }
            double getTorqueYawStd() const { return torqueYawStd; }
            bool getSameControls() const { return sameControls; }

            double getSolveTime() const { return solveTime; }

            bool getPrintAllStates() const { return printAllStates; }

            bool getUseSST() const { return useSST; }

            double getMaxDroneAngle() const { return maxDroneAngle; }
            double getMaxAnglePayload() const { return maxAnglePayload; }
            
            /** \brief Get the default start state for the system */
            base::ScopedState<> getDefaultStartState() const override; 

            /** \brief Extract the full state from the geometric component */
            ompl::base::ScopedState<> getFullStateFromGeometricComponent(
                const base::ScopedState<> &state) const override
            {
                base::ScopedState<> s(getStateSpace());
                std::vector <double> reals = state.reals ();
            
                s = 0.0;
                for (size_t i = 0; i < reals.size (); ++i)
                    s[i] = reals[i];
                return s;
            }

            /** \brief Get the geometric component's state space */
            const base::StateSpacePtr& getGeometricComponentStateSpace() const override
            {
                return getStateSpace()->as<base::CompoundStateSpace>()->getSubspace(0);
            }

            /** \brief Get the geometric component state for internal use */
            const base::State* getGeometricComponentStateInternal(const base::State* state, unsigned int /*index*/) const override
            {
                return state->as<base::CompoundState>()->components[0];
            }

            /** \brief Set default bounds for the state and control spaces */
            virtual void setDefaultBounds();

            friend class PayloadStateValidityChecker;

        protected:
            /** \brief Compute the state derivative for the system */
            virtual void ode(const control::ODESolver::StateType& q,
                            const control::Control* ctrl,
                            control::ODESolver::StateType& qdot) {std::cout << "Base class ode called!" << std::endl;};

            void ode_free(const std::vector<double>& q,             // full state vector
                          const Eigen::VectorXd& u_cmd,   // 4*droneCount_ control vector [T, τx, τy, τz]×droneCount
                          std::vector<double>& qdot);               // output derivative

            
            /** \brief Perform one step of RK4 integration */
            void integrateOnceRK4(
                const Eigen::VectorXd &q_eig,
                const Eigen::VectorXd &u_cmd,
                double dt,
                std::function<void(const std::vector<double>&,
                                const Eigen::VectorXd&,
                                std::vector<double>&)> rhs,
                Eigen::VectorXd &q_next_eig,
                Eigen::Vector3d &payloadAccel_out);



            /** \brief Compute the drone commands (thrust and torques) given the desired payload acceleration */
            virtual void computeDroneCommandsP2(
                const std::vector<double>&           T_refs,
                const std::vector<Eigen::Quaterniond>& q_refs,
                const Eigen::VectorXd&               current_state,
                Eigen::VectorXd&                     u_cmd) const;

            /** \brief Post-processing after propagating the system state */
            virtual void postPropagate(const base::State* state, const control::Control* control, double duration, base::State* result);

            /** \brief Construct the control space for the system */
            control::ControlSpacePtr constructControlSpace();

            /** \brief Construct the state space for the system */
            static base::StateSpacePtr constructStateSpace();

            static unsigned int droneCount_;          // Number of drones in the system
            double timeStep_{0.01};                   // Time step for integration

            int drone_state_len = 11;

            double m_payload = 2.0;                   // Mass of the payload
            double m_drone = 0.25;                     // Mass of each drone

            double hover_t = (m_payload / droneCount_ + m_drone) * 9.81;

            double w = 2.0;                           // Payload width
            double d = 2.0;                           // Payload depth
            double h = 1.0;                           // Payload height

            double l = 2;                             // Length of the cables

            double droneH = 0.02;                      // Height of the drone
            double droneR = 0.2;                      // Radius of the drone

            Eigen::Matrix3d droneInertia = (Eigen::Matrix3d() << 
                (1.0 / 12.0) * m_drone * (3 * droneR * droneR + droneH * droneH), 0, 0,
                0, (1.0 / 12.0) * m_drone * (3 * droneR * droneR + droneH * droneH), 0,
                0, 0, (1.0 / 2.0) * m_drone * droneR * droneR).finished();


            Eigen::Matrix3d payloadInertia = (Eigen::Matrix3d() << 
                (1.0 / 12.0) * m_payload * (h * h + d * d), 0, 0,
                0, (1.0 / 12.0) * m_payload * (w * w + h * h), 0,
                0, 0, (1.0 / 12.0) * m_payload * (w * w + d * d)).finished();




            double droneBeta = 0.01;                  // Drone torque damping coefficient
            double payloadBeta = 0.1;                 // Payload linear damping coefficient

            // Inputs of drones
            double maxTorquePitchRoll = 0.001;
            double maxTorqueYaw = 0.0005;
            double minThrust = 0;
            double maxThrust = 30;

            double maxDroneAngle = 70;
            double maxDroneVel = 20;
            
            double maxAnglePayload = 70;
            double maxPayloadVel = 10;
            double maxPayloadAngVel = 10;

            // Angle of cable from vertical
            double maxTheta = 70;
            double maxThetaVel = 20;

            // Standard deviations for control inputs (RRT only)
            double thrustStd = 5;
            double torquePitchRollStd = 0.01;
            double torqueYawStd = 0.0001;

            bool sameControls = false; // true -> same controls for all drones, false -> different controls

            Eigen::Vector3d startPosition_{-10.0, -40.0, 20.0};
            Eigen::Vector3d goalPosition_{60.0, -15.0, 15.0};
        
            double solveTime = 60 * 2; // 10 hours

            int n_iter = 50;  // number of control iterations

            const double Pq = 10.0;
            const double Pw = 0.4;

            double accelMaxDelta = 0.01; // or your chosen maximum delta
            
            bool printAllStates = true; // Print all states to file
            
            bool useSST = false; // true -> SST, false -> RRT

            control::ODESolverPtr odeSolver;  // ODE solver for the system

            RigidBodyGeometry rigidBody_;  // Stores mesh and collision checking
        };
    }
}


// class AccelDeltaControlSampler : public ompl::control::ControlSampler
// {
// public:
//     AccelDeltaControlSampler(const ompl::control::ControlSpace *space, double accelMaxDelta)
//         : ompl::control::ControlSampler(space), accelMaxDelta_(accelMaxDelta) {}

//     void sample(ompl::control::Control *control) override
//     {
//         std::cout << "[SAMPLER] sample() (delta-bounded around zero) called." << std::endl;
//         double* sampled = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
//         for (int i = 0; i < 3; ++i)
//         {
//             sampled[i] = rng_.uniformReal(-accelMaxDelta_, accelMaxDelta_);
//             std::cout << "    Sampled[" << i << "] = " << sampled[i] << std::endl;
//         }
//     }

// protected:
//     ompl::RNG rng_;
//     double accelMaxDelta_;
// };

// class PosDeltaControlSampler : public ompl::control::ControlSampler
// {
// public:
//     PosDeltaControlSampler(const ompl::control::ControlSpace *space, double posMaxDelta)
//         : ompl::control::ControlSampler(space), posMaxDelta_(posMaxDelta) {}

//     void sample(ompl::control::Control *control) override
//     {
//         double* sampled = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
//         for (int i = 0; i < 3; ++i)
//             sampled[i] = rng_.uniformReal(-posMaxDelta_, posMaxDelta_);
//     }

// protected:
//     ompl::RNG rng_;
//     double posMaxDelta_;
// };

class PosRRTControlSampler : public ompl::control::ControlSampler
{
public:
    PosRRTControlSampler(const ompl::control::ControlSpace *space, const ompl::base::RealVectorBounds& bounds)
        : ompl::control::ControlSampler(space), bounds_(bounds) {}

    void sample(ompl::control::Control *control) override
    {
        double* sampled = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
        for (int i = 0; i < 3; ++i)
            sampled[i] = rng_.uniformReal(bounds_.low[i], bounds_.high[i]);
    }

protected:
    ompl::RNG rng_;
    ompl::base::RealVectorBounds bounds_;
};

#endif // OMPL_APP_PAYLOAD_SYSTEM_H
