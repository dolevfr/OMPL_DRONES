#ifndef PAYLOAD_CLASSES_H
#define PAYLOAD_CLASSES_H

#include "PayloadOneDrone.h"

// class PayloadSystemValidityChecker : public ompl::base::StateValidityChecker
// {
// public:
//     PayloadSystemValidityChecker(const ompl::base::SpaceInformationPtr &si, const ompl::app::PayloadSystem &system)
//         : ompl::base::StateValidityChecker(si), payloadSystem_(system)
//     {
//     }

//     bool isValid(const ompl::base::State *state) const override
//     {
//         std::cout << "Checking state validity..." << std::endl;
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

//         double maxAngleCosPayload = std::cos(payloadSystem_.getMaxAnglePayload() * M_PI / 180.0);
//         if ((payloadQuat.x() * payloadQuat.x() + payloadQuat.y() * payloadQuat.y()) > (1 - maxAngleCosPayload) / 2)
//         {
//             return false;
//         }

//         // Drone tilt check:
//         double maxAngleCosDrone = std::cos(payloadSystem_.getMaxDroneAngle() * M_PI / 180.0);

//         for (unsigned int i = 0; i < payloadSystem_.getRobotCount(); ++i)
//         {
//             unsigned int droneBaseIndex = 2 + i * 3; // Make sure this index is correct
//             const auto *droneSO3State = compoundState->as<ompl::base::SO3StateSpace::StateType>(droneBaseIndex);
//             Eigen::Quaterniond droneQuat(
//                 droneSO3State->w,
//                 droneSO3State->x,
//                 droneSO3State->y,
//                 droneSO3State->z);
//             droneQuat.normalize(); // Normalize the drzone quaternion

//             if ((droneQuat.x() * droneQuat.x() + droneQuat.y() * droneQuat.y()) > (1 - maxAngleCosDrone) / 2)
//             {
//                 return false;
//             }
//         }

//         // std::cout << "Number of valid states: " << ++validStates << std::endl;
//         return true;
//     }

// private:
//     const ompl::app::PayloadSystem &payloadSystem_; // Reference to PayloadSystem
// };

class PayloadSmoothDirectedControlSampler : public ompl::control::DirectedControlSampler
{
public:
    PayloadSmoothDirectedControlSampler(const ompl::control::SpaceInformation *si,
                                        const ompl::app::PayloadSystem *payloadSystem)
        : DirectedControlSampler(si), siC_(si), payloadSystem_(payloadSystem),
          plannerData_(nullptr),
          rng_(std::random_device{}()), normalDist_(0.0, 1.0)
    {
        droneCount_ = payloadSystem_->getRobotCount();
        maxTorquePitchRoll_ = payloadSystem_->getMaxTorquePitchRoll();
        maxTorqueYaw_ = payloadSystem_->getMaxTorqueYaw();
        minThrust_ = payloadSystem_->getMinThrust();
        maxThrust_ = payloadSystem_->getMaxThrust();

        // Standard deviations for control inputs
        thrustStd_ = payloadSystem_->getThrustStd();
        torquePitchRollStd_ = payloadSystem_->getTorquePitchRollStd();
        torqueYawStd_ = payloadSystem_->getTorqueYawStd();
        sameControls_ = payloadSystem_->getSameControls();
    }

    void setPlannerData(const ompl::control::PlannerData &plannerData)
    {
        plannerData_ = &plannerData; // safely store pointer/reference
    }

    unsigned int sampleTo(ompl::control::Control *control,
                          const ompl::control::Control *previous,
                          const ompl::base::State *source,
                          ompl::base::State *dest) override
    {
        if (previous)
        {
            const double *prevVals =
                previous->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
        
            bool allZeros = true;
            for (unsigned int i = 0; i < payloadSystem_->getRobotCount() * 4; ++i)
                if (std::fabs(prevVals[i]) > 0) { allZeros = false; break; }
            if (allZeros)            // first step → give hover thrust
            {
                double hover = (payloadSystem_->getDroneMass() +
                                payloadSystem_->getPayloadMass() /
                                payloadSystem_->getRobotCount()) * 9.81 * 1.2;
        
                double *vals =
                    control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
                for (unsigned int i = 0; i < payloadSystem_->getRobotCount(); ++i)
                {
                    unsigned int idx = i * 4;
                    vals[idx] = hover;
                    vals[idx + 1] = vals[idx + 2] = vals[idx + 3] = 0.0;
                }

                unsigned int dur = std::uniform_int_distribution<unsigned int>(
                    siC_->getMinControlDuration(), siC_->getMaxControlDuration())(rng_);
                return siC_->propagateWhileValid(source, control, dur, dest);
            }
            else                     // normal case
            {
                sampleAroundPrevious(control, previous);
            }
        }
        else                          // no previous control at all
        {
            sampleAroundPrevious(control, previous);
        }

        unsigned int duration = std::uniform_int_distribution<unsigned int>(
            siC_->getMinControlDuration(), siC_->getMaxControlDuration())(rng_);
        return siC_->propagateWhileValid(source, control, duration, dest);
    }

    unsigned int sampleTo(ompl::control::Control *control,
                          const ompl::base::State *source,
                          ompl::base::State *dest) override
    {
        std::cout << "Sampling control without previous" << std::endl;
        // Simply forward to the three-argument overload, with `previous = nullptr`.
        // This is typically called if OMPL hasn't stored a previous control for some reason.
        return sampleTo(control, /* previous */ nullptr, source, dest);
    }

private:
    const ompl::control::SpaceInformation *siC_;
    const ompl::app::PayloadSystem *payloadSystem_; // clearly defined pointer to PayloadSystem
    const ompl::control::PlannerData *plannerData_; // just store pointer!

    unsigned int droneCount_;
    double thrustStd_, torquePitchRollStd_, torqueYawStd_;
    double maxTorquePitchRoll_, maxTorqueYaw_, minThrust_, maxThrust_;
    bool sameControls_;

    std::mt19937 rng_;
    std::normal_distribution<double> normalDist_;

    double clamp(double val, double minVal, double maxVal) const
    {
        return std::max(minVal, std::min(val, maxVal));
    }

    void sampleAroundPrevious(ompl::control::Control *control, const ompl::control::Control *prevControl)
    {
        const double *prevVals = prevControl->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
        double *newControlVals = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;

        if (!sameControls_)
        {
            for (unsigned int i = 0; i < droneCount_; ++i)
            {
                unsigned int idx = i * 4;
                newControlVals[idx] = clamp(prevVals[idx] + thrustStd_ * normalDist_(rng_), minThrust_, maxThrust_);
                newControlVals[idx + 1] = clamp(prevVals[idx + 1] + torquePitchRollStd_ * normalDist_(rng_), -maxTorquePitchRoll_, maxTorquePitchRoll_);
                newControlVals[idx + 2] = clamp(prevVals[idx + 2] + torquePitchRollStd_ * normalDist_(rng_), -maxTorquePitchRoll_, maxTorquePitchRoll_);
                newControlVals[idx + 3] = clamp(prevVals[idx + 3] + torqueYawStd_ * normalDist_(rng_), -maxTorqueYaw_, maxTorqueYaw_);
            }
        }

        else
        {
            // Sample once around previous values
            double sampledThrust = clamp(prevVals[0] + thrustStd_ * normalDist_(rng_), minThrust_, maxThrust_);
            double sampledTorqueRoll = clamp(prevVals[1] + torquePitchRollStd_ * normalDist_(rng_), -maxTorquePitchRoll_, maxTorquePitchRoll_);
            double sampledTorquePitch = clamp(prevVals[2] + torquePitchRollStd_ * normalDist_(rng_), -maxTorquePitchRoll_, maxTorquePitchRoll_);
            double sampledTorqueYaw = clamp(prevVals[3] + torqueYawStd_ * normalDist_(rng_), -maxTorqueYaw_, maxTorqueYaw_);

            // Apply this identical control to all drones
            for (unsigned int i = 0; i < droneCount_; ++i)
            {
                unsigned int idx = i * 4;
                newControlVals[idx] = sampledThrust;
                newControlVals[idx + 1] = sampledTorqueRoll;
                newControlVals[idx + 2] = sampledTorquePitch;
                newControlVals[idx + 3] = sampledTorqueYaw;
            }
        }
    }

    const ompl::control::Control *getControlFromPlannerData(const ompl::base::State *state)
    {
        if (!plannerData_)
            return nullptr;

        for (unsigned int i = 0; i < plannerData_->numVertices(); ++i)
        {
            if (siC_->getStateSpace()->equalStates(plannerData_->getVertex(i).getState(), state))
            {
                std::vector<unsigned int> incomingEdges;
                plannerData_->getIncomingEdges(i, incomingEdges);
                if (!incomingEdges.empty())
                {
                    const ompl::base::PlannerDataEdge *baseEdge = &plannerData_->getEdge(incomingEdges[0], i);

                    const auto edge =
                        dynamic_cast<const ompl::control::PlannerDataEdgeControl *>(baseEdge);

                    if (edge)
                        return edge->getControl();
                }
            }
        }
        return nullptr;
    }
};

class MyRRT : public ompl::control::RRT
{
public:
    MyRRT(const ompl::control::SpaceInformationPtr &si)
        : ompl::control::RRT(si), siC_(si) {}

    ompl::control::DirectedControlSamplerPtr allocDirectedControlSampler()
    {
        return siC_->allocDirectedControlSampler();
    }

private:
    ompl::control::SpaceInformationPtr siC_;
};

/*********************************************************************
 *  Payload‑aware Gaussian control sampler (for SST)
 ********************************************************************/
class PayloadSmoothControlSampler : public ompl::control::ControlSampler
{
public:
    PayloadSmoothControlSampler(const ompl::control::ControlSpace *cs,
                                const ompl::app::PayloadSystem   *payloadSystem)
        : ControlSampler(cs)
        , cs_(cs)
        , payloadSystem_(payloadSystem)
        , rng_(std::random_device{}())
        , n_(0.0, 1.0)
    {
        droneCount_          = payloadSystem_->getRobotCount();
        maxTorquePR_         = payloadSystem_->getMaxTorquePitchRoll();
        maxTorqueYaw_        = payloadSystem_->getMaxTorqueYaw();
        minThrust_           = payloadSystem_->getMinThrust();
        maxThrust_           = payloadSystem_->getMaxThrust();
        thrustStd_           = payloadSystem_->getThrustStd();
        torquePitchRollStd_  = payloadSystem_->getTorquePitchRollStd();
        torqueYawStd_        = payloadSystem_->getTorqueYawStd();
        sameControls_        = payloadSystem_->getSameControls();
    }

    void sample(ompl::control::Control *c) override
    {
        const double hover = (payloadSystem_->getDroneMass() +
                              payloadSystem_->getPayloadMass()
                              / payloadSystem_->getRobotCount()) * 9.81;
        fillControl(c, /*prev*/ nullptr, hover);
    }

    /* called when OMPL knows the state but not a previous control --*/
    void sample(ompl::control::Control *c,
                const ompl::base::State * /*state*/) override
    {
        sample(c);          // just delegate to the version above
    }

    /* main “smooth perturbation” overload -------------------------*/
    void sampleNext(ompl::control::Control *c,
                    const ompl::control::Control *prev,
                    const ompl::base::State * /*state*/) override
    {
        fillControl(c, prev, 0.0 /*hover unused when prev given*/);
    }


private:
    /* helpers -----------------------------------------------------*/
    inline double clamp(double v, double lo, double hi) const
    { return std::max(lo, std::min(v, hi)); }

    void fillControl(ompl::control::Control *c,
                     const ompl::control::Control *prev,
                     double hoverThrust)
    {
        auto *vals = c->as<ompl::control::RealVectorControlSpace::ControlType>()->values;

        /* decide whether to sample w.r.t. previous or around hover */
        const double *base =
            prev ? prev->as<ompl::control::RealVectorControlSpace::ControlType>()->values
                 : nullptr;

        if (!sameControls_)
        {
            for (unsigned int i = 0; i < droneCount_; ++i)
            {
                unsigned idx = i * 4;
                vals[idx]     = clamp(randThrust(base ? base[idx] : hoverThrust),    minThrust_, maxThrust_);
                vals[idx + 1] = clamp(randPR   (base ? base[idx + 1] : 0.0),        -maxTorquePR_, maxTorquePR_);
                vals[idx + 2] = clamp(randPR   (base ? base[idx + 2] : 0.0),        -maxTorquePR_, maxTorquePR_);
                vals[idx + 3] = clamp(randYaw  (base ? base[idx + 3] : 0.0),        -maxTorqueYaw_, maxTorqueYaw_);
            }
        }
        else
        {
            const double thrust = clamp(randThrust(base ? base[0] : hoverThrust),  minThrust_,   maxThrust_);
            const double roll   = clamp(randPR   (base ? base[1] : 0.0),          -maxTorquePR_, maxTorquePR_);
            const double pitch  = clamp(randPR   (base ? base[2] : 0.0),          -maxTorquePR_, maxTorquePR_);
            const double yaw    = clamp(randYaw  (base ? base[3] : 0.0),          -maxTorqueYaw_, maxTorqueYaw_);

            for (unsigned int i = 0; i < droneCount_; ++i)
            {
                unsigned idx = i * 4;
                vals[idx]     = thrust;
                vals[idx + 1] = roll;
                vals[idx + 2] = pitch;
                vals[idx + 3] = yaw;
            }
        }
    }

    inline double randThrust(double mean) const      { return mean + thrustStd_          * n_(rng_); }
    inline double randPR(double mean)    const       { return mean + torquePitchRollStd_ * n_(rng_); }
    inline double randYaw(double mean)   const       { return mean + torqueYawStd_       * n_(rng_); }

    /* data --------------------------------------------------------*/
    const ompl::control::ControlSpace *cs_;
    const ompl::app::PayloadSystem    *payloadSystem_;

    unsigned   droneCount_;
    double     thrustStd_, torquePitchRollStd_, torqueYawStd_;
    double     maxTorquePR_, maxTorqueYaw_, minThrust_, maxThrust_;
    bool       sameControls_;

    mutable std::mt19937                     rng_;
    mutable std::normal_distribution<double> n_;
};

/*********************************************************************
 *  Thin SST wrapper that plugs in the sampler above
 ********************************************************************/
class MySST : public ompl::control::SST
{
public:
    MySST(const ompl::control::SpaceInformationPtr &si,
          const ompl::app::PayloadSystem           *sys)
        : ompl::control::SST(si)
    {
        /* Tell the **control space** to build our sampler */
        si->getControlSpace()->setControlSamplerAllocator(
            [sys](const ompl::control::ControlSpace *cs)
            {
                return std::make_shared<PayloadSmoothControlSampler>(cs, sys);
            });
    }
};




#endif // PAYLOAD_CLASSES_H
