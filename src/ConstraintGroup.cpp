
#include "../ccrrt/ConstraintGroup.h"

using namespace ccrrt;

ConstraintGroup::ConstraintGroup()
{
    _dimension = 0;
}

bool ConstraintGroup::getJacobian(Eigen::MatrixXd& J, const Trajectory& traj)
{
    bool result = true;
    
    J.resize(_dimension, traj.xi.size());
    Eigen::MatrixXd Htemp;
    size_t c_counter = 0;
    for(size_t i=0; i<_constraints.size(); ++i)
    {
        result &= _constraints[i]->getJacobian(Htemp, traj);
        J.block(c_counter,0,_constraints[i]->constraintDimension(),traj.xi.size());
        c_counter += _constraints[i]->constraintDimension();
    }
    
    return result;
}

Constraint::validity_t ConstraintGroup::getCost(Eigen::VectorXd& cost,
                                                const Trajectory& traj)
{
    cost.resize(_dimension);
    validity_t result = VALID;
    validity_t tempresult = VALID;
    Eigen::VectorXd tempCost;
    for(size_t i=0; i<_constraints.size(); ++i)
    {
        tempresult = _constraints[i]->getCost(tempCost, traj);
        cost.block(i,0,_constraints[i]->constraintDimension(),traj.xi.size()) = tempCost;
        if((int)tempresult < (int)result)
            result = tempresult;
    }
    
    return result;
}

Constraint::validity_t ConstraintGroup::getValidity(const Trajectory &traj)
{
    validity_t result = VALID;
    validity_t tempresult = VALID;

    for(size_t i=0; i<_constraints.size(); ++i)
    {
        tempresult = _constraints[i]->getValidity(traj);
        if((int)tempresult < (int)result)
            result = tempresult;
    }

    return result;
}

size_t ConstraintGroup::constraintDimension() const
{
    return _dimension;
}

void ConstraintGroup::addConstraint(Constraint *constraint)
{
    _dimension += constraint->constraintDimension();
    _constraints.push_back(constraint);
}

size_t ConstraintGroup::numConstraints() const
{
    return _constraints.size();
}




