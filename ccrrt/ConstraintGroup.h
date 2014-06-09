#ifndef CONSTRAINTGROUP_H
#define CONSTRAINTGROUP_H

#include "Constraint.h"
#include <vector>

namespace ccrrt {

class ConstraintGroup : public Constraint
{
public:
    
    validity_t getCostGradient(Eigen::VectorXd& gradient,
                               const Eigen::VectorXd& parent,
                               const Eigen::VectorXd& config,
                               const Eigen::VectorXd& target);
    validity_t getValidity(const Eigen::VectorXd &config);
    
    void addConstraint(Constraint* constraint);
    size_t numConstraints() const;
    
protected:
    
    std::vector<Constraint*> _constraints;
};

}

#endif // CONSTRAINTGROUP_H
