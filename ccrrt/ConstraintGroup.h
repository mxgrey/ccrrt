#ifndef CONSTRAINTGROUP_H
#define CONSTRAINTGROUP_H

#include "Constraint.h"
#include <vector>

namespace ccrrt {

class ConstraintGroup : public Constraint
{
public:
    
    ConstraintGroup();
    
    bool fillJacobian(Eigen::MatrixXd &H, const Eigen::VectorXd &xi, size_t numWaypoints);
    validity_t configValidity(Eigen::VectorXd& lambda, 
                              const Eigen::VectorXd &xi, 
                              size_t numWaypoints);
    size_t constraintDimension() const;
    
    void addConstraint(Constraint* constraint);
    size_t numConstraints() const;
    
protected:
    
    size_t _dimension;
    std::vector<Constraint*> _constraints;
};

}

#endif // CONSTRAINTGROUP_H
