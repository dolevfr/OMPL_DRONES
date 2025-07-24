#include "AcadosOneDrone.h"

namespace ob = ompl::base;
namespace oc = ompl::control;

MPCSampler::MPCSampler(const ompl::control::SpaceInformation *si,
                       std::shared_ptr<AcadosMPC> mpc,
                       int replanSteps)
  : DirectedControlSampler(si)
  , mpc_(std::move(mpc))
  , replanSteps_(replanSteps)
{}

/// Overload 1: without “previous” control
unsigned int MPCSampler::sampleTo(ompl::control::Control *control,
                                  const ompl::base::State *source,
                                  ompl::base::State * /*dest*/)
{
    // 1) flatten OMPL state → Eigen x0
    Eigen::VectorXd x0 = flattenState(source);

    // 2) desired payload accel: here zero (hover). Adapt if you have a target.
    Eigen::Vector3d a_des(0, 0, 0);

    // 3) reset MPC and get u0
    mpc_->reset(x0, a_des);
    Eigen::VectorXd u0 = mpc_->nextControl(x0, a_des);

    // 4) write into OMPL control struct
    auto *rvcs = si_->getControlSpace()->as<ompl::control::RealVectorControlSpace>();
    auto *rc   = control->as<ompl::control::RealVectorControlSpace::ControlType>();
    for (int i = 0; i < static_cast<int>(u0.size()); ++i)
        rc->values[i] = u0[i];
    
    std::cout << "[MPCSampler] Sampled control: ";
    for (unsigned int i = 0; i < rvcs->getDimension(); ++i)
        std::cout << rc->values[i] << " ";
    std::cout << std::endl;

    // 5) how many steps to apply this control
    return replanSteps_;
}

/// Overload 2: with “previous” control (just delegate)
unsigned int MPCSampler::sampleTo(ompl::control::Control *control,
                                  const ompl::control::Control * /*previous*/,
                                  const ompl::base::State *source,
                                  ompl::base::State *dest)
{
    return sampleTo(control, source, dest);
}

Eigen::VectorXd MPCSampler::flattenState(const ompl::base::State* state) const
{
    // Access the CompoundStateSpace from PayloadSystem (or its base)
    auto css_sh = si_->getStateSpace()->as<ob::CompoundStateSpace>(); // Use si_ to access the state space
    if (!css_sh)
    {
        throw std::runtime_error("[flattenState] Failed to cast to CompoundStateSpace.");
    }

    // Dynamically calculate NX_ based on subspaces
    int NX_ = 0;
    for (size_t i = 0; i < css_sh->getSubspaceCount(); ++i)
    {
        const auto* sub = css_sh->getSubspace(i).get();
        if (dynamic_cast<const ob::SE3StateSpace*>(sub))
            NX_ += 7; // SE3: 3 position + 4 quaternion
        else if (auto rv = dynamic_cast<const ob::RealVectorStateSpace*>(sub))
            NX_ += rv->getDimension();
        else if (dynamic_cast<const ob::SO3StateSpace*>(sub))
            NX_ += 4; // SO3: 4 quaternion
        else
            throw std::runtime_error("[flattenState] Unsupported subspace type for dimension counting.");
    }

    // Initialize the vector to hold the flattened state
    Eigen::VectorXd x(NX_);
    int idx = 0;

    // Loop through each subspace again to flatten the state
    for (size_t i = 0; i < css_sh->getSubspaceCount(); ++i)
    {
        const auto* sub = css_sh->getSubspace(i).get();
        if (auto se3 = dynamic_cast<const ob::SE3StateSpace*>(sub))
        {
            const auto* st = state->as<ob::SE3StateSpace::StateType>(); // Correctly use 'as' to cast to SE3StateType
            x[idx++] = st->getX();
            x[idx++] = st->getY();
            x[idx++] = st->getZ();
            const auto& q = st->rotation();
            x[idx++] = q.x;
            x[idx++] = q.y;
            x[idx++] = q.z;
            x[idx++] = q.w;
        }
        else if (auto rv = dynamic_cast<const ob::RealVectorStateSpace*>(sub))
        {
            const auto* rvs = state->as<ob::RealVectorStateSpace::StateType>();
            for (unsigned j = 0; j < rv->getDimension(); ++j)
            {
                x[idx++] = rvs->values[j];
            }
        }
        else if (auto so3 = dynamic_cast<const ob::SO3StateSpace*>(sub))
        {
            const auto* qq = state->as<ob::SO3StateSpace::StateType>();
            x[idx++] = qq->x;
            x[idx++] = qq->y;
            x[idx++] = qq->z;
            x[idx++] = qq->w;
        }
        else
        {
            throw std::runtime_error("[flattenState] Unsupported subspace type while flattening state.");
        }
    }

    // Ensure that the flattened state vector has the correct size
    if (idx != NX_)
    {
        throw std::runtime_error("[flattenState] Dimension mismatch: packed dim = " + std::to_string(idx) + ", expected NX_ = " + std::to_string(NX_));
    }

    return x;
}







