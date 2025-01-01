#ifndef OMPL_APP_PAYLOAD_SYSTEM_H
#define OMPL_APP_PAYLOAD_SYSTEM_H


#include <Eigen/Dense>
#include <vector>
#include <omplapp/apps/AppBase.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>  // Include the correct header for RealVectorControlSpace
#include <ompl/base/spaces/SE3StateSpace.h>  // Include the correct header for SE3StateSpace
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h> // For working with numpy arrays
#include <pybind11/eigen.h> // For Eigen integration
#include <pybind11/embed.h>
#include <pybind11/attr.h> // Ensures that pybind11 symbols are properly defined
#include <experimental/filesystem>

#if defined(_WIN32) || defined(__CYGWIN__)
    #ifdef BUILDING_DLL
        #define PAYLOADSYSTEM_API __declspec(dllexport)
    #else
        #define PAYLOADSYSTEM_API __declspec(dllimport)
    #endif
#else
    #define PAYLOADSYSTEM_API __attribute__((visibility("default")))
#endif



namespace ompl
{
    namespace app
    {
        /** \brief A class for planning with a payload system suspended by drones */
        class PAYLOADSYSTEM_API PayloadSystem : public AppBase<AppType::CONTROL>
        {
        public:
            PayloadSystem();

            ~PayloadSystem() override = default;

            pybind11::object droneModel_;

            /** \brief Check if self-collision is enabled (not used for payload systems) */
            bool isSelfCollisionEnabled() const override { return false; }

            /** \brief Get the number of drones in the system */
            unsigned int getRobotCount() const override { return droneCount_; }

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

            /** \brief Get the inverse mass for each drone */
            const std::vector<double>& getMassInv() const;

            /** \brief Set the inverse mass for each drone */
            void setMassInv(const std::vector<double>& massInv);

            /** \brief Get the damping coefficients for each drone */
            const std::vector<double>& getBeta() const;

            /** \brief Set the damping coefficients for each drone */
            void setBeta(const std::vector<double>& beta);

            /** \brief Get the mass of the payload */
            double getPayloadMass() const { return m_payload; }

            /** \brief Set the mass of the payload */
            void setPayloadMass(double mass) { m_payload = mass; }

            /** \brief Get the mass of the drones */
            double getDroneMass() const { return m_drone; }

            /** \brief Set the mass of the drones */
            void setDroneMass(double mass) { m_drone = mass; }

            /** \brief Get the inertia tensor of the payload */
            const Eigen::Matrix3d& getPayloadInertia() const;

            /** \brief Set the inertia tensor of the payload */
            void setPayloadInertia(const Eigen::Matrix3d& inertia);

            /** \brief Get the dimensions of the payload (a, b) */
            const Eigen::Vector2d& getPayloadDimensions() const;

            /** \brief Set the dimensions of the payload (a, b) */
            void setPayloadDimensions(const Eigen::Vector2d& dimensions);

            /** \brief Get the length of the cable */
            double getCableLength() const { return l; }

            /** \brief Set the length of the cable */
            void setCableLength(double length) { l = length; }

            /** \brief Get the position of a payload corner in the local frame */
            // Eigen::Vector3d getPayloadCorner(unsigned int index) const;

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

            /** \brief Calculate the tension in a cable */
            double calculateCableTension(const Eigen::Vector3d& cableVec) const;

            static unsigned int droneCount_;          // Number of drones in the system
            double timeStep_{1e-2};                   // Time step for integration

            unsigned int iterationNumber_; // Track the iteration count

            double m_payload = 10.0;                   // Mass of the payload
            double m_drone = 1.0;                     // Mass of each drone
            double massInv_ = 1.0 / m_drone;              // Inverse masses for each drone
            double beta_ = 0.1;                // Damping coefficients for each drone
            Eigen::Matrix3d inertia_ = Eigen::Matrix3d::Identity() * 5; // Example: uniform inertia
            Eigen::Vector2d payloadDimensions = Eigen::Vector2d(50.0, 50.0); // Example: 1m x 0.5m rectangle
            double l = 50;                 // Length of the cables
            double cableStiffness_ = 1e5;              // Stiffness of the cables

            control::ODESolverPtr odeSolver;          // ODE solver for the system
        };
    }
}

#endif // OMPL_APP_PAYLOAD_SYSTEM_H
