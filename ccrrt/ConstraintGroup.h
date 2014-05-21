#ifndef CONSTRAINTGROUP_H
#define CONSTRAINTGROUP_H

#include "Constraint.h"
#include <vector>

namespace ccrrt {

class ConstraintGroup : public Constraint
{
public:
    
    ConstraintGroup();
    
    bool fillJacobian(Eigen::MatrixXd &H, const ccrrt::Trajectory& traj);
    validity_t getCost(Eigen::VectorXd& cost, const ccrrt::Trajectory& traj);
    size_t constraintDimension() const;
    
    void addConstraint(Constraint* constraint);
    size_t numConstraints() const;
    
protected:
    
    size_t _dimension;
    std::vector<Constraint*> _constraints;
};

}

#endif // CONSTRAINTGROUP_H
