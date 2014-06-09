#ifndef CIRCLECONSTRAINT_H
#define CIRCLECONSTRAINT_H

#include "Constraint.h"

namespace ccrrt {

class CircleConstraint : public Constraint
{
public:
    
    CircleConstraint(const Eigen::Vector2d& mCenter = Eigen::Vector2d::Zero(),
                     double mRadius = 0, double mBuffer = 0);
    
    validity_t getCostGradient(Eigen::VectorXd &gradient,
                               const Eigen::VectorXd& parent,
                               const Eigen::VectorXd& config,
                               const Eigen::VectorXd& target);

    validity_t getValidity(const Eigen::VectorXd &config);
    
    size_t constraintDimension() const;
    
    Eigen::Vector2d center;
    double radius;
    double buffer;

protected:

    validity_t _basicCostGrad(Eigen::Vector2d& grad_c, const Eigen::Vector2d& config);

};

} // namespace ccrrt

#endif // CIRCLECONSTRAINT_H
