#ifndef MPC_SAMPLER_H
#define MPC_SAMPLER_H

#include "AcadosOneDrone.h"

namespace ob = ompl::base;
namespace oc = ompl::control;

class MPCSampler : public oc::DirectedControlSampler
{
public:
    /// ctor: pass OMPL SI and your AcadosMPC instance
    MPCSampler(const oc::SpaceInformation *si,
               std::shared_ptr<AcadosMPC> mpc,
               int replanSteps);

    /// sample one control from 'state', aiming (softly) at 'dest'
    unsigned int sampleTo(oc::Control *control,
                        const ob::State *source,
                        ob::State *dest) override;


    unsigned int sampleTo(ompl::control::Control *control,
                      const ompl::control::Control *previous,
                      const ompl::base::State *source,
                      ompl::base::State *dest) override;



private:
    std::shared_ptr<AcadosMPC> mpc_;
    int replanSteps_;

    /// flatten an OMPL compound state → Eigen::VectorXd of length NX
    Eigen::VectorXd flattenState(const ob::State *s) const;
};
#endif // MPC_SAMPLER_H
