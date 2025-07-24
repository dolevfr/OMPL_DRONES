#ifndef OMPL_APP_PAYLOAD_SYSTEM_H
#define OMPL_APP_PAYLOAD_SYSTEM_H

#include <acados_c/ocp_nlp_interface.h>

#include <array>
#include <boost/numeric/odeint.hpp>
#include <boost/filesystem.hpp>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <Eigen/Dense>
#include <fstream>
#include <filesystem>
#include <iostream>
#include <memory>
#include <nlohmann/json.hpp>

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
#include <random>
#include <stdexcept>
#include <vector>

#include "AcadosMPC.h"
#include "AcadosPythonBridge.h"
#include "MPCSampler.h"
#include <acados_c/ocp_nlp_interface.h>

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


            /** \brief Get the solve time for the system */
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
            virtual void ode(const control::ODESolver::StateType& q, const control::Control* ctrl, control::ODESolver::StateType& qdot);

            /** \brief Post-processing after propagating the system state */
            virtual void postPropagate(const base::State* state, const control::Control* control, double duration, base::State* result);

            /** \brief Construct the control space for the system */
            control::ControlSpacePtr constructControlSpace();

            /** \brief Construct the state space for the system */
            static base::StateSpacePtr constructStateSpace();

            static unsigned int droneCount_;          // Number of drones in the system
            double timeStep_{0.01};                   // Time step for integration

            double m_payload = 2.0;                   // Mass of the payload
            double m_drone = 0.25;                     // Mass of each drone

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
                double maxTorquePitchRoll = 0.01;
                double maxTorqueYaw = 0.005;
                double minThrust = 0;
                double maxThrust = 90;
    
                double maxDroneAngle = 70;
                double maxDroneVel = 20;
                
                double maxAnglePayload = 30;
                double maxPayloadVel = 10;
                double maxPayloadAngVel = 0.5;
    
                // Angle of cable from vertical
                double maxTheta = 10;
                double maxThetaVel = 2;
    
                // Standard deviations for control inputs (RRT only)
                double thrustStd = 3;
                double torquePitchRollStd = 0.005;
                double torqueYawStd = 0.0001;
    
                bool sameControls = false; // true -> same controls for all drones, false -> different controls
    
                Eigen::Vector3d startPosition_{-10.0, -40.0, 20.0};
                Eigen::Vector3d goalPosition_{60.0, -15.0, 15.0};
            
                double solveTime = 10;  
                
                bool printAllStates = false; // Print all states to file
                
                bool useSST = false; // true -> SST, false -> RRT
    
                control::ODESolverPtr odeSolver;  // ODE solver for the system
    
                RigidBodyGeometry rigidBody_;  // Stores mesh and collision checking
        };
    }
}





// class PayloadSystemValidityChecker : public ompl::base::StateValidityChecker
// {
// public:
//     PayloadSystemValidityChecker(const ompl::base::SpaceInformationPtr &si, const ompl::app::PayloadSystem &system)
//         : ompl::base::StateValidityChecker(si), payloadSystem_(system)
//     {}

//     bool isValid(const ompl::base::State *state) const override
//     {
//         static int payloadExceeded = 0;
//         static int droneExceeded = 0;
//         static int validStates = 0;

//         // --- Collision Check ---
//         // Clone the state and enforce bounds
//         ompl::base::State *correctedState = si_->cloneState(state);
//         si_->getStateSpace()->enforceBounds(correctedState);
//         si_->freeState(correctedState);
    
//         // --- Custom Tilt Constraint Check ---
//         const auto *compoundState = state->as<ompl::base::CompoundState>();
//         if (!compoundState)
//         {
//             std::cerr << "State is not a CompoundState!" << std::endl;
//             return false;
//         }
        
//         // Payload tilt check:
//         const auto *payloadSE3State = compoundState->as<ompl::base::SE3StateSpace::StateType>(0);
//         Eigen::Quaterniond payloadQuat(
//             payloadSE3State->rotation().w,
//             payloadSE3State->rotation().x,
//             payloadSE3State->rotation().y,
//             payloadSE3State->rotation().z);
//         payloadQuat.normalize(); // Ensure normalization
    
//         Eigen::Vector3d payloadUp = payloadQuat * Eigen::Vector3d::UnitZ();
//         double payloadTilt = std::acos( std::clamp(payloadUp.normalized().dot(Eigen::Vector3d::UnitZ()), -1.0, 1.0) );
//         if (payloadTilt > (payloadSystem_.getMaxAnglePayload() * M_PI / 180.0))
//         {
//             // std::cout << "Number of payload tilt exceed: " << ++payloadExceeded << std::endl;
//             return false;
//         }
        
//         // Drone tilt check:
//         for (unsigned int i = 0; i < payloadSystem_.getRobotCount() ; ++i)
//         {
//             unsigned int droneBaseIndex = 2 + i * 3; // Make sure this index is correct
//             const auto *droneSO3State = compoundState->as<ompl::base::SO3StateSpace::StateType>(droneBaseIndex);
//             Eigen::Quaterniond droneQuat(
//                 droneSO3State->w,
//                 droneSO3State->x,
//                 droneSO3State->y,
//                 droneSO3State->z);
//             droneQuat.normalize(); // Normalize the drone quaternion
    
//             Eigen::Vector3d droneUp = droneQuat * Eigen::Vector3d::UnitZ();
//             double droneTilt = std::acos( std::clamp(droneUp.normalized().dot(Eigen::Vector3d::UnitZ()), -1.0, 1.0) );
//             if (droneTilt > (payloadSystem_.getMaxDroneAngle() * M_PI / 180.0))
//             {
//                 // std::cout << "Number of drone tilt exceed: " << ++droneExceeded << std::endl;
//                 return false;
//             }
//         }
        
//         // std::cout << "Number of valid states: " << ++validStates << std::endl;
//         return true;
//     }
    
// private:
//     const ompl::app::PayloadSystem &payloadSystem_; // Reference to PayloadSystem
// };

#endif // OMPL_APP_PAYLOAD_SYSTEM_H