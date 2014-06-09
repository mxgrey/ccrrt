
#include "../ccrrt/CircleConstraint.h"
#include <iostream>

using namespace ccrrt;
using namespace Eigen;

CircleConstraint::CircleConstraint(const Vector2d &mCenter, double mRadius, double mBuffer) :
    center(mCenter),
    radius(mRadius),
    buffer(mBuffer)
{

}

Constraint::validity_t CircleConstraint::getCostGradient(VectorXd &gradient,
                                                         const VectorXd& ,
                                                         const VectorXd &config,
                                                         const VectorXd& )
{
    Vector2d grad2d;
    Vector2d config2d(config);
    
    validity_t result = _basicCostGrad(grad2d, config2d);
    gradient = VectorXd(grad2d);
    return result;
}

Constraint::validity_t CircleConstraint::getValidity(const VectorXd& config)
{
    double norm = (config-center).norm();
    if(norm >= radius+buffer)
    {
        return VALID;
    }
    else if(norm >= radius && fabs(buffer) > 1e-10)
    {
        return AT_RISK;
    }
    else
    {
        return INVALID;
    }

    return INVALID;
}

Constraint::validity_t CircleConstraint::_basicCostGrad(Eigen::Vector2d& grad_c, const Eigen::Vector2d& config)
{
    grad_c = config - center;
    double norm = grad_c.norm();
    
    if(norm >= radius+buffer)
    {
        grad_c.setZero();
        return VALID;
    }
    else if(norm >= radius && fabs(buffer) > 1e-10)
    {
        grad_c = grad_c/norm*(norm-radius-buffer)/buffer;
        return AT_RISK;
    }
    else if(norm > 1e-10)
    {
        grad_c = -grad_c/norm;
        return INVALID;
    }
    else
    {
        grad_c.setZero();
        return INVALID;
    }
}

size_t CircleConstraint::constraintDimension() const
{
    return 1;
}


