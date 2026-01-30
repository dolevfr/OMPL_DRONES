#ifndef PAYLOAD_CLASSES_H
#define PAYLOAD_CLASSES_H

#include "AccelFourDrones.h"

class PayloadSmoothDirectedControlSampler : public ompl::control::DirectedControlSampler
{
public:
    PayloadSmoothDirectedControlSampler(const ompl::control::SpaceInformation *si,
                                        const ompl::app::AccelPayloadSystem *payloadSystem)
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
    const ompl::app::AccelPayloadSystem *payloadSystem_; // clearly defined pointer to AccelPayloadSystem
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
class PayloadAccelControlSampler : public ompl::control::ControlSampler
{
public:
    PayloadAccelControlSampler(const ompl::control::ControlSpace *cs,
                              const ompl::app::AccelPayloadSystem *payloadSystem)
        : ControlSampler(cs), cs_(cs), payloadSystem_(payloadSystem), rng_(std::random_device{}()),
          dist_(0.0, 1.0)
    {


        // std dev chosen so that 3-sigma ≈ 1e-3, so std dev ≈ 3.3e-4
        noiseStd_ = 1e-3;

        // Get control bounds for clipping
        auto bounds = cs_->as<ompl::control::RealVectorControlSpace>()->getBounds();
        for (unsigned i = 0; i < bounds.low.size(); ++i) {
            low_.push_back(bounds.low[i]);
            high_.push_back(bounds.high[i]);
        }
    }

    void sample(ompl::control::Control *control) override
    {
        std::cout << "Called sample() without state" << std::endl;
        fillControl(control, external_prev_accel);
    }

    void sample(ompl::control::Control *control, const ompl::base::State * /*state*/) override
    {
        std::cout << "Called sample() with state" << std::endl;
        sample(control);
    }

    void sampleNext(ompl::control::Control *control,
                    const ompl::control::Control *prev,
                    const ompl::base::State * /*state*/) override
    {
        if(prev == nullptr)
            std::cout << "[Sampler] No previous control, sampling from zero base\n";
        else
            std::cout << "[Sampler] Previous control first component: " << prev->as<ompl::control::RealVectorControlSpace::ControlType>()->values[0] << std::endl;
        fillControl(control, external_prev_accel);
    }

    Eigen::Vector3d external_prev_accel;


private:
    void fillControl(ompl::control::Control *control, const Eigen::Vector3d &base_accel)
    {
        double *vals = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;

        for (unsigned i = 0; i < 3; ++i)
        {
            double sample = base_accel(i) + noiseStd_ * dist_(rng_);
            vals[i] = sample; // No clamp as requested
        }
    }


    const ompl::control::ControlSpace *cs_;
    const ompl::app::AccelPayloadSystem *payloadSystem_;
    std::mt19937 rng_;
    std::normal_distribution<double> dist_;
    double noiseStd_;
    std::vector<double> low_, high_;
    std::shared_ptr<PayloadAccelControlSampler> sampler_;

};


/*********************************************************************
 *  Thin SST wrapper that plugs in the sampler above
 ********************************************************************/
class MySST : public ompl::control::SST
{
public:
    MySST(const ompl::control::SpaceInformationPtr &si,
          const ompl::app::AccelPayloadSystem           *sys)
        : ompl::control::SST(si)
    {
        /* Tell the **control space** to build our sampler */
        si->getControlSpace()->setControlSamplerAllocator(
            [sys](const ompl::control::ControlSpace *cs)
            {
                return std::make_shared<PayloadAccelControlSampler>(cs, sys);
            });
    }
};




#endif // PAYLOAD_CLASSES_H
