
#include "../ccrrt/ConstraintGroup.h"
#include <iostream>

using namespace ccrrt;

ConstraintGroup::ConstraintGroup()
{
    _dimension = 0;
}

size_t ConstraintGroup::getJacobian(Eigen::MatrixXd& J, const Trajectory& traj)
{
    bool rank = 0;
    
    J.resize(_dimension, traj.xi.size());
    Eigen::MatrixXd Htemp;
    size_t rank_check = 0;
    for(size_t i=0; i<_constraints.size(); ++i)
    {
        rank_check = _constraints[i]->getJacobian(Htemp, traj);
        J.block(rank,0,rank_check,traj.xi.size()) = Htemp.block(0,0,rank_check,traj.xi.size());
        rank += rank_check;
    }
    
    return rank;
}

Constraint::validity_t ConstraintGroup::getCostGradient(Eigen::VectorXd &gradient, 
                                      const Eigen::VectorXd &config)
{
    validity_t result = VALID;
    validity_t tempresult = VALID;
    Eigen::VectorXd tempgrad;
    for(size_t i=0; i<_constraints.size(); ++i)
    {
        tempresult = _constraints[i]->getCostGradient(tempgrad, config);
        if(i==0)
            gradient = tempgrad;
        else
            gradient += tempgrad;
        
        if((int)tempresult < (int)result)
            result = tempresult;
    }
    
    gradient = gradient/_constraints.size();
    return result;
}

Constraint::validity_t ConstraintGroup::getCost(Eigen::VectorXd& cost,
                                                const Trajectory& traj)
{
    cost.resize(_dimension);
    validity_t result = VALID;
    validity_t tempresult = VALID;
    Eigen::VectorXd tempCost;
    size_t next_start = 0;
    for(size_t i=0; i<_constraints.size(); ++i)
    {
        tempresult = _constraints[i]->getCost(tempCost, traj);
        if((int)tempresult < (int)result)
            result = tempresult;
        
        if(tempresult == Constraint::VALID)
            continue;
        
        cost.block(next_start,0,_constraints[i]->constraintDimension(),1) = tempCost;
        next_start += _constraints[i]->constraintDimension();
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




