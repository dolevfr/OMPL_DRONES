#ifndef OMPL_APP_PAYLOAD_SYSTEM_H
#define OMPL_APP_PAYLOAD_SYSTEM_H


#include <Eigen/Dense>
#include <vector>
#include <omplapp/apps/AppBase.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h> 
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/goals/GoalRegion.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/base/terminationconditions/IterationTerminationCondition.h>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <boost/numeric/odeint.hpp>

#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/ControlDurationObjective.h>
#include <ompl/base/objectives/MinimaxObjective.h>
#include <ompl/base/objectives/MechanicalWorkOptimizationObjective.h>
#include <ompl/control/planners/sst/SST.h>


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

            unsigned int getRobotCount() const override { return droneCount_; }
            double getPayloadWidth() const { return w; }
            double getPayloadDepth() const { return d; }
            double getPayloadHeight() const { return h; }
            double getCableLength() const { return l; }


            /** \brief Get the solve time for the system */
            double getSolveTime() const { return solveTime; }
            
            /** \brief Get the default start state for the system */
            base::ScopedState<> getDefaultStartState() const override { return base::ScopedState<>(getStateSpace()); }

            /** \brief Extract the full state from the geometric component */
            base::ScopedState<> getFullStateFromGeometricComponent(const base::ScopedState<> &state) const override;

            /** \brief Get the geometric component's state space */
            const base::StateSpacePtr& getGeometricComponentStateSpace() const override
            {
                return getStateSpace()->as<base::CompoundStateSpace>()->getSubspace(0);
            }

            /** \brief Set default bounds for the state and control spaces */
            virtual void setDefaultBounds();

        protected:
            /** \brief Compute the state derivative for the system */
            virtual void ode(const control::ODESolver::StateType& q, const control::Control* ctrl, control::ODESolver::StateType& qdot);

            /** \brief Post-processing after propagating the system state */
            virtual void postPropagate(const base::State* state, const control::Control* control, double duration, base::State* result);

            /** \brief Get the geometric component state for internal use */
            const base::State* getGeometricComponentStateInternal(const base::State* state, unsigned int index) const override;

            /** \brief Construct the control space for the system */
            static control::ControlSpacePtr constructControlSpace();

            /** \brief Construct the state space for the system */
            static base::StateSpacePtr constructStateSpace();



            static unsigned int droneCount_;          // Number of drones in the system
            double timeStep_{1e-2};                   // Time step for integration

            double m_payload = 5.0;                   // Mass of the payload
            double m_drone = 1.0;                     // Mass of each drone

            double w = 2.0;                           // Payload width
            double d = 2.0;                           // Payload depth
            double h = 1.0;                           // Payload height

            double l = 1;                             // Length of the cables

            Eigen::Matrix3d droneInertia = Eigen::Matrix3d::Identity() * 0.01; // Example: uniform inertia

            Eigen::Matrix3d payloadInertia = (Eigen::Matrix3d() << 
                (1.0 / 12.0) * m_payload * (h * h + d * d), 0, 0,
                0, (1.0 / 12.0) * m_payload * (w * w + h * h), 0,
                0, 0, (1.0 / 12.0) * m_payload * (w * w + d * d)).finished();

            // Inputs of drones
            static constexpr double maxTorque = 0.1;
            static constexpr double maxThrust = 30;

            static constexpr double maxDroneAngle = 30;
            static constexpr double maxDroneVel = 2;
            
            static constexpr double maxAnglePayload = 10;
            static constexpr double maxPayloadVel = 20;

            // Angle of cable from vertical
            static constexpr double maxTheta = 30;
            static constexpr double maxThetaVel = 5;

            double solveTime = 30;

            control::ODESolverPtr odeSolver;          // ODE solver for the system

            RigidBodyGeometry rigidBody_;  // Stores mesh and collision checking
        };
    }
}

class PayloadStateValidityChecker : public ompl::base::StateValidityChecker
{
public:
    PayloadStateValidityChecker(const ompl::base::SpaceInformationPtr &si,
                                ompl::app::RigidBodyGeometry &rigidBody,
                                const ompl::app::PayloadSystem &system)
        : ompl::base::StateValidityChecker(si),
          rigidBody_(rigidBody),
          system_(system),
          validityChecker_(rigidBody.allocStateValidityChecker(si, nullptr, false))
    {
    }

    bool isValid(const ompl::base::State *state) const override
    {
        // Check the entire state for collisions using the allocated validity checker.
        // (This typically checks the payload and the robots as a whole.)
        if (!validityChecker_->isValid(state))
        {
            return false;
        }

        // Our compound state layout is:
        // [0] → Payload state (SE3: 7 components)
        // [1] → Payload velocities (6 components)
        // [2..] → For each drone: 11 components:
        //         [0-3]: Drone rotation (SO(3) quaternion)
        //         [4-6]: Drone angular velocity
        //         [7-8]: Cable angles (theta and phi)
        //         [9-10]: Derivatives of cable angles

        const auto *compoundState = state->as<ompl::base::CompoundState>();

        // Get payload SE3 state (index 0).
        const auto *payloadState = compoundState->as<ompl::base::SE3StateSpace::StateType>(0);
        Eigen::Vector3d payloadPos(payloadState->getX(), payloadState->getY(), payloadState->getZ());
        Eigen::Quaterniond payloadRot(payloadState->rotation().w,
                                      payloadState->rotation().x,
                                      payloadState->rotation().y,
                                      payloadState->rotation().z);

        // Payload geometry parameters.
        double w = system_.getPayloadWidth();
        double d = system_.getPayloadDepth();
        double h = system_.getPayloadHeight();
        double l = system_.getCableLength();

        // For each drone, compute the corresponding quadrotor position.
        for (unsigned int i = 0; i < system_.getRobotCount(); ++i)
        {
            // Drone states start at compound state index 2, each drone occupies one slot.
            unsigned int droneSubstateIndex = 2 + i;
            const auto *droneState = compoundState->as<ompl::base::RealVectorStateSpace::StateType>(droneSubstateIndex);

            // Extract cable angles: indices 7 (theta) and 8 (phi) in the drone’s 11-D substate.
            double theta = droneState->values[7];
            double phi   = droneState->values[8];

            // Compute cable direction.
            Eigen::Vector3d cableDir(sin(theta) * cos(phi),
                                     sin(theta) * sin(phi),
                                     cos(theta));

            // Determine the appropriate payload corner for the drone.
            Eigen::Vector3d corner;
            switch (i)
            {
                case 0:  corner = Eigen::Vector3d(-w / 2, -d / 2, h / 2); break;
                case 1:  corner = Eigen::Vector3d( w / 2, -d / 2, h / 2); break;
                case 2:  corner = Eigen::Vector3d( w / 2,  d / 2, h / 2); break;
                case 3:  corner = Eigen::Vector3d(-w / 2,  d / 2, h / 2); break;
                default: corner = Eigen::Vector3d(0, 0, 0); break;
            }

            // Compute the quadrotor’s global position.
            Eigen::Vector3d quadrotorPos = payloadPos + payloadRot * (corner + l * cableDir);

            // Allocate a temporary SE3 state for the drone.
            auto quadrotorSE3 = si_->getStateSpace()->as<ompl::base::SE3StateSpace>()->allocState();
            auto *quadrotorSE3State = quadrotorSE3->as<ompl::base::SE3StateSpace::StateType>();

            // Set the computed position.
            quadrotorSE3State->setXYZ(quadrotorPos.x(), quadrotorPos.y(), quadrotorPos.z());
            // Optionally, set the orientation. For example, if the drone collision geometry
            // should be aligned with the payload or identity, do so here.
            // Here, we set the identity rotation. Replace if a different orientation is desired.
            quadrotorSE3State->rotation().x = 0.0;
            quadrotorSE3State->rotation().y = 0.0;
            quadrotorSE3State->rotation().z = 0.0;
            quadrotorSE3State->rotation().w = 1.0;

            // Check for collision for this quadrotor state.
            if (!validityChecker_->isValid(quadrotorSE3))
            {
                si_->getStateSpace()->as<ompl::base::SE3StateSpace>()->freeState(quadrotorSE3);
                return false;
            }
            si_->getStateSpace()->as<ompl::base::SE3StateSpace>()->freeState(quadrotorSE3);
        }

        return true;
    }

private:
    ompl::app::RigidBodyGeometry &rigidBody_;
    const ompl::app::PayloadSystem &system_;
    ompl::base::StateValidityCheckerPtr validityChecker_;
};

#endif // OMPL_APP_PAYLOAD_SYSTEM_H

