
#include "../ccrrt/ConstraintGroup.h"
#include <iostream>

using namespace ccrrt;

Constraint::validity_t ConstraintGroup::getCostGradient(Eigen::VectorXd& gradient,
                                                        const Eigen::VectorXd& parent,
                                                        const Eigen::VectorXd& config,
                                                        const Eigen::VectorXd& target)
{
    validity_t result = VALID;
    validity_t tempresult = VALID;
    Eigen::VectorXd tempgrad;
    for(size_t i=0; i<_constraints.size(); ++i)
    {
        tempresult = _constraints[i]->getCostGradient(tempgrad, parent, config, target);
//        std::cout << gradient.transpose() << "|\t";
        if(i==0)
            gradient = tempgrad;
        else
            gradient += tempgrad;
        
        if((int)tempresult < (int)result)
            result = tempresult;
    }
    
    gradient = gradient/_constraints.size();
//    std::cout << /*"final:" << gradient.transpose() <<*/ std::endl;
    return result;
}

Constraint::validity_t ConstraintGroup::getValidity(const Eigen::VectorXd& config)
{
    validity_t result = VALID;
    validity_t tempresult = VALID;

    for(size_t i=0; i<_constraints.size(); ++i)
    {
        tempresult = _constraints[i]->getValidity(config);
        if((int)tempresult < (int)result)
            result = tempresult;
    }

    return result;
}

void ConstraintGroup::addConstraint(Constraint *constraint)
{
    _constraints.push_back(constraint);
}

size_t ConstraintGroup::numConstraints() const
{
    return _constraints.size();
}




