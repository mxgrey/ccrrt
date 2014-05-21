#ifndef CIRCLECONSTRAINT_H
#define CIRCLECONSTRAINT_H

#include "Constraint.h"

namespace ccrrt {

class CircleConstraint : public Constraint
{
public:
    
    CircleConstraint(const Eigen::Vector2d& center = Eigen::Vector2d::Zero(),
                     double radius = 0);
    
    bool fillJacobian(Eigen::MatrixXd &H, const Eigen::VectorXd &xi, size_t numWaypoints);
    
    validity_t configValidity(Eigen::VectorXd& lambda, 
                              const Eigen::VectorXd &xi, 
                              size_t numWaypoints);
    
    size_t constraintDimension() const;
    
protected:
    
    Eigen::Vector2d _center;
    double _radius;
    
};

}

#endif // CIRCLECONSTRAINT_H
