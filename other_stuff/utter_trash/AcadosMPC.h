#pragma once
#include <Eigen/Dense>
#include <memory>
#include <acados_solver_payload_1drones.h>

class AcadosMPC
{
public:
    explicit AcadosMPC(bool verbose = false);
    ~AcadosMPC();

    void            reset(const Eigen::VectorXd &x0,
                          const Eigen::Vector3d &a_des);
    Eigen::VectorXd nextControl(const Eigen::VectorXd &x0,
                                const Eigen::Vector3d &a_des);

private:
    using CapsulePtr =
        std::unique_ptr<payload_1drones_solver_capsule,
                        decltype(&payload_1drones_acados_free)>;

    CapsulePtr          capsule_;
    const int           NX_, NU_, N_;

    ocp_nlp_config  *nlp_config_ = nullptr;   // <-- new
    ocp_nlp_dims    *nlp_dims_   = nullptr;   // <-- new
    ocp_nlp_in      *nlp_in_     = nullptr;
    ocp_nlp_out     *nlp_out_    = nullptr;
    ocp_nlp_solver  *nlp_solver_ = nullptr;
};
