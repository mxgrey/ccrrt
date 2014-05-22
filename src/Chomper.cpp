
#include "../ccrrt/Chomper.h"

using namespace ccrrt;
using namespace Eigen;

Chomper::Chomper()
{
    _constraint = NULL;
}

const Trajectory& Chomper::getTrajectory() const
{
    return _trajectory;
}

const Constraint* const Chomper::getConstraint() const
{
    return _constraint;
}

void Chomper::initialize(const Trajectory& trajectory, Constraint* constraint)
{
    _trajectory = trajectory;
    _constraint = constraint;
    
    size_t nm = trajectory.state_space*trajectory.waypoints;
    size_t c = _constraint->constraintDimension();
    
    Df.resize(nm);
    h.resize(_constraint->constraintDimension());
    
    A.resize(nm,nm);
    Ainv.resize(nm,nm);
    H.resize(c,nm);
    HAinvHt_inv.resize(c,c);
    Ainv_Ht_HAinvHt_inv.resize(nm,c);
}

Constraint::validity_t Chomper::iterate()
{
    return Constraint::INVALID;
}


