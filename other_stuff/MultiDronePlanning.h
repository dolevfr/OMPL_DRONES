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

#ifndef OMPLAPP_MULTI_DRONE_PLANNING_
#define OMPLAPP_MULTI_DRONE_PLANNING_

#include "omplapp/apps/AppBase.h"
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>

namespace ompl
{
    namespace app
    {
        /** \brief A class for planning with multiple drones */
        class MultiDronePlanning : public AppBase<AppType::CONTROL>
        {
        public:
            MultiDronePlanning();

            ~MultiDronePlanning() override = default;
            
            bool isSelfCollisionEnabled() const override;
            unsigned int getRobotCount() const override;


            base::ScopedState<> getDefaultStartState() const override;
            base::ScopedState<> getFullStateFromGeometricComponent(const base::ScopedState<> &state) const override;

            const base::StateSpacePtr& getGeometricComponentStateSpace() const override
            {
                return getStateSpace()->as<base::CompoundStateSpace>()->getSubspace(0);
            }

            virtual void setDefaultBounds();

            // Getter for massInv_
            const std::vector<double>& getMassInv() const;

            // Setter for massInv_
            void setMassInv(const std::vector<double>& massInv);

            // Getter for beta_
            const std::vector<double>& getBeta() const;

            // Setter for beta_
            void setBeta(const std::vector<double>& beta);

        protected:
            virtual void ode(const control::ODESolver::StateType& q, const control::Control* ctrl, control::ODESolver::StateType& qdot);

            virtual void postPropagate(const base::State* state, const control::Control* control, double duration, base::State* result);

            const base::State* getGeometricComponentStateInternal(const base::State* state, unsigned int index) const override;

            static control::ControlSpacePtr constructControlSpace();
            static base::StateSpacePtr constructStateSpace();

            static unsigned int droneCount_;  // Number of drones
            double timeStep_{1e-2};

            std::vector<double> massInv_;   // Inverse mass for each drone
            std::vector<double> beta_;      // Damping coefficient for each drone
            control::ODESolverPtr odeSolver;
        };

    }
}

#endif
