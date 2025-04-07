#ifndef PAYLOAD_CLASSES_H
#define PAYLOAD_CLASSES_H

#include "PayloadFourDrones.h"


class PayloadSystemValidityChecker : public ompl::base::StateValidityChecker
{
public:
    PayloadSystemValidityChecker(const ompl::base::SpaceInformationPtr &si, const ompl::app::PayloadSystem &system)
        : ompl::base::StateValidityChecker(si), payloadSystem_(system)
    {}

    bool isValid(const ompl::base::State *state) const override
    {
        static int payloadExceeded = 0;
        static int droneExceeded = 0;
        static int validStates = 0;

        // --- Collision Check ---
        // Clone the state and enforce bounds
        ompl::base::State *correctedState = si_->cloneState(state);
        si_->getStateSpace()->enforceBounds(correctedState);
        si_->freeState(correctedState);
    
        // --- Custom Tilt Constraint Check ---
        const auto *compoundState = state->as<ompl::base::CompoundState>();
        if (!compoundState)
        {
            std::cerr << "State is not a CompoundState!" << std::endl;
            return false;
        }
        
        // Payload tilt check:
        const auto *payloadSE3State = compoundState->as<ompl::base::SE3StateSpace::StateType>(0);
        Eigen::Quaterniond payloadQuat(
            payloadSE3State->rotation().w,
            payloadSE3State->rotation().x,
            payloadSE3State->rotation().y,
            payloadSE3State->rotation().z);
        payloadQuat.normalize(); // Ensure normalization
    
        Eigen::Vector3d payloadUp = payloadQuat * Eigen::Vector3d::UnitZ();
        double payloadTilt = std::acos( std::clamp(payloadUp.normalized().dot(Eigen::Vector3d::UnitZ()), -1.0, 1.0) );
        if (payloadTilt > (payloadSystem_.getMaxAnglePayload() * M_PI / 180.0))
        {
            // std::cout << "Number of payload tilt exceed: " << ++payloadExceeded << std::endl;
            return false;
        }
        
        // Drone tilt check:
        for (unsigned int i = 0; i < payloadSystem_.getRobotCount() ; ++i)
        {
            unsigned int droneBaseIndex = 2 + i * 3; // Make sure this index is correct
            const auto *droneSO3State = compoundState->as<ompl::base::SO3StateSpace::StateType>(droneBaseIndex);
            Eigen::Quaterniond droneQuat(
                droneSO3State->w,
                droneSO3State->x,
                droneSO3State->y,
                droneSO3State->z);
            droneQuat.normalize(); // Normalize the drone quaternion
    
            Eigen::Vector3d droneUp = droneQuat * Eigen::Vector3d::UnitZ();
            double droneTilt = std::acos( std::clamp(droneUp.normalized().dot(Eigen::Vector3d::UnitZ()), -1.0, 1.0) );
            if (droneTilt > (payloadSystem_.getMaxDroneAngle() * M_PI / 180.0))
            {
                // std::cout << "Number of drone tilt exceed: " << ++droneExceeded << std::endl;
                return false;
            }
        }
        
        // std::cout << "Number of valid states: " << ++validStates << std::endl;
        return true;
    }
    
    

private:
    const ompl::app::PayloadSystem &payloadSystem_; // Reference to PayloadSystem
};



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
        if (siC_->getStateSpace()->equalStates(source, payloadSystem_->getDefaultStartState().get()))
        {
            std::cout << "Sampling from the start state." << std::endl;
            // Set the initial thrust and zero torques
            double hoverThrust = (payloadSystem_->getDroneMass() +
                                  payloadSystem_->getPayloadMass() / payloadSystem_->getRobotCount()) * 9.81;

            double *vals = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
            for (unsigned int i = 0; i < payloadSystem_->getRobotCount(); ++i)
            {
                unsigned int idx = i * 4;
                // Scale the hover thrust by some factor if you like (e.g. * 3)
                vals[idx]     = hoverThrust * 3;
                vals[idx + 1] = 0.0;
                vals[idx + 2] = 0.0;
                vals[idx + 3] = 0.0;
            }
        }
        else if (previous)
        {
            sampleAroundPrevious(control, previous);
        }
        else
        {
            std::cout << "No previous control provided. Sampling new control." << std::endl;
            siC_->allocControlSampler()->sample(control);
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

            // std::cout << "[Sampler] Identical control: thrust=" << sampledThrust 
            //             << ", roll=" << sampledTorqueRoll
            //             << ", pitch=" << sampledTorquePitch
            //             << ", yaw=" << sampledTorqueYaw << std::endl;

            // Apply this identical control to all drones
            for (unsigned int i = 0; i < droneCount_; ++i)
            {
                unsigned int idx = i * 4;
                newControlVals[idx]     = sampledThrust;
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




// // Adapter that wraps your custom directed sampler so that it can be used as a ControlSampler.
// // It lets you set a previous control that will be passed to sampleTo().
// class DirectedControlSamplerAdapter : public ompl::control::ControlSampler
// {
// public:
//     // Constructor: note that the base class expects a pointer to a ControlSpace.
//     DirectedControlSamplerAdapter(ompl::control::SpaceInformation *si)
//         : ompl::control::ControlSampler(si->getControlSpace().get()),
//           siC_(si), previousControl_(nullptr)
//     {
//         directedSampler_ = si->allocDirectedControlSampler();
//     }

//     // Set the previous control (from the parent motion) to be used during sampling.
//     void setPreviousControl(const ompl::control::Control *prev)
//     {
//         previousControl_ = prev;
//     }

//     // sample() is called by the planner; it uses sampleTo() on the underlying directed sampler.
//     // In the adapter class, add a debug print in sample():
//     virtual void sample(ompl::control::Control *control) override
//     {
//         std::cout << "[Adapter] sample() called." << std::endl;
//         // Allocate temporary states for propagation.
//         ompl::base::State *source = siC_->allocState();
//         ompl::base::State *dest   = siC_->allocState();

//         if (previousControl_)
//         {
//             std::cout << "[Adapter] Using previous control for sampleTo()." << std::endl;
//             directedSampler_->sampleTo(control, previousControl_, source, dest);
//         }
//         else
//         {
//             std::cout << "[Adapter] No previous control; sampling normally." << std::endl;
//             directedSampler_->sampleTo(control, nullptr, source, dest);
//         }

//         siC_->freeState(source);
//         siC_->freeState(dest);

//         // Clear the stored previous control.
//         previousControl_ = nullptr;
//     }


// private:
//     ompl::control::SpaceInformation *siC_;
//     std::shared_ptr<ompl::control::DirectedControlSampler> directedSampler_;
//     const ompl::control::Control *previousControl_;
// };




// class MySST : public ompl::control::SST
// {
// public:
//     MySST(const ompl::control::SpaceInformationPtr &si)
//         : ompl::control::SST(si), siC_(si)
//     {
//         // Replace the default control sampler with our adapter.
//         controlSampler_ = std::make_shared<DirectedControlSamplerAdapter>(si.get());
//     }

//     void setup() override {
//         ompl::control::SST::setup();
//         controlSampler_ = std::make_shared<DirectedControlSamplerAdapter>(siC_.get());
//     }

//     // Helper method to insert a motion into the nearest–neighbor structure.
//     // This is a simplified version that calls the protected member nn_->add(motion).
//     // (Make sure nn_ is accessible in your OMPL version.)
// protected:
//     void addMotion(ompl::control::SST::Motion *motion) {
//         // Insert the new motion into the nearest neighbor data structure.
//         nn_->add(motion);
//         // In a complete implementation, you may need to update the witness set as well.

//     virtual ompl::control::SST::Motion* selectNode(ompl::control::SST::Motion* sample) override {
//         std::cout << "MySST::selectNode() called." << std::endl;
//         ompl::control::SST::Motion* result = ompl::control::SST::selectNode(sample);
//         std::cout << "MySST::selectNode() returns " << result << std::endl;
//         return result;
//     }

// public:
//     // Override the solve() method to inject our custom expansion logic.
//     virtual ompl::base::PlannerStatus solve(const ompl::base::PlannerTerminationCondition &ptc) override {
//         checkValidity();
//         ompl::base::State *sample = siC_->allocState();
//         auto stateSampler = siC_->getStateSpace()->allocStateSampler();
    
//         unsigned long iteration = 0;
//         std::cout << "MySST::solve() starting expansion loop." << std::endl;
    
//         while (!ptc())
//         {
//             iteration++;
//             if(iteration % 100 == 0)
//                 std::cout << "Iteration " << iteration << std::endl;
    
//             // Sample a random state.
//             stateSampler->sampleUniform(sample);
//             std::cout << "Iteration " << iteration << ": Sampled state." << std::endl;

//             // Create a dummy Motion for selectNode.
//             std::cout << "Iteration " << iteration << ": Creating dummy motion for selectNode." << std::endl;
//             ompl::control::SST::Motion *dummy = new ompl::control::SST::Motion(siC_.get());
//             siC_->getStateSpace()->copyState(dummy->state_, sample);

//             std::cout << "Iteration " << iteration << ": NN structure size = " << nn_->size() << std::endl;
//             ompl::control::SST::Motion *parentMotion = selectNode(dummy);
//             std::cout << "Iteration " << iteration << ": Returned from selectNode(dummy)." << std::endl;
//             delete dummy;

//             if (!parentMotion)
//             {
//                 std::cout << "Iteration " << iteration << ": No parent motion found." << std::endl;
//                 continue;
//             }
//             std::cout << "Iteration " << iteration << ": Parent motion found." << std::endl;
    
//             // Retrieve and prepare our adapter.
//             DirectedControlSamplerAdapter *adapter =
//                 dynamic_cast<DirectedControlSamplerAdapter*>(controlSampler_.get());
//             if (!adapter)
//             {
//                 std::cout << "Iteration " << iteration << ": Failed to cast control sampler to adapter." << std::endl;
//                 break;
//             }
//             std::cout << "Iteration " << iteration << ": Setting previous control from parent." << std::endl;
//             adapter->setPreviousControl(parentMotion->control_);
    
//             // Allocate a new control and state.
//             ompl::control::Control *u = siC_->allocControl();
//             ompl::base::State *newState = siC_->allocState();
//             std::cout << "Iteration " << iteration << ": Sampling new control using adapter." << std::endl;
//             adapter->sample(u);
    
//             // Propagate from parent's state using the sampled control.
//             unsigned int duration = siC_->propagateWhileValid(parentMotion->state_, u,
//                                                               siC_->getMaxControlDuration(), newState);
//             std::cout << "Iteration " << iteration << ": Propagation duration = " << duration << std::endl;
//             if (duration == 0)
//             {
//                 std::cout << "Iteration " << iteration << ": Propagation failed (duration 0). Freeing control and state." << std::endl;
//                 siC_->freeControl(u);
//                 siC_->freeState(newState);
//                 continue;
//             }
    
//             // Create a new motion.
//             ompl::control::SST::Motion *motion = new ompl::control::SST::Motion(siC_.get());
//             siC_->getStateSpace()->copyState(motion->state_, newState);
//             motion->control_ = u;
//             motion->parent_ = parentMotion;  // Link to parent.
//             addMotion(motion);
//             std::cout << "Iteration " << iteration << ": Motion added to tree." << std::endl;
    
//             siC_->freeState(newState);
//         }
    
//         siC_->freeState(sample);
//         std::cout << "MySST::solve() expansion loop ended after " << iteration << " iterations." << std::endl;
//         return ompl::base::PlannerStatus::EXACT_SOLUTION;
//     }
    

// private:
//     ompl::control::SpaceInformationPtr siC_;
// };




#endif // PAYLOAD_CLASSES_H
