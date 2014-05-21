
#include "../ccrrt/Constraint.h"

using namespace ccrrt;

bool Constraint::fillJacobian(Eigen::MatrixXd &, const Eigen::VectorXd &, size_t )
{
    return true;
}

Constraint::validity_t Constraint::configValidity(Eigen::VectorXd& , const Eigen::VectorXd&, size_t )
{
    return VALID;
}
