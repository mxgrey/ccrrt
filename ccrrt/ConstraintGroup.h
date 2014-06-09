#ifndef CONSTRAINTGROUP_H
#define CONSTRAINTGROUP_H

#include "Constraint.h"
#include <vector>

namespace ccrrt {

class ConstraintGroup : public Constraint
{
public:
    
    ConstraintGroup();
    
    validity_t getCostGradient(Eigen::VectorXd& gradient,
                               const Eigen::VectorXd& parent,
                               const Eigen::VectorXd& config,
                               const Eigen::VectorXd& target);
    validity_t getValidity(const Eigen::VectorXd &config);
    size_t constraintDimension() const;
    
    void addConstraint(Constraint* constraint);
    size_t numConstraints() const;
    
protected:
    
    size_t _dimension;
    std::vector<Constraint*> _constraints;
};

}

#endif // CONSTRAINTGROUP_H
