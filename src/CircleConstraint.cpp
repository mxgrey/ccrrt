
#include "../ccrrt/CircleConstraint.h"
#include <iostream>

using namespace ccrrt;
using namespace Eigen;

CircleConstraint::CircleConstraint(const Vector2d &mCenter, double mRadius, double mBuffer) :
    center(mCenter),
    radius(mRadius),
    buffer(mBuffer)
{

}

bool CircleConstraint::fillJacobian(MatrixXd &H, const Trajectory& traj)
{
    assert( traj.state_space == 0 );

    H.resize(constraintDimension(), traj.xi.size()); // 1 x nm
    
    Eigen::Vector2d config;
    Eigen::Vector2d diff;
    for(size_t i=0; i<traj.waypoints; ++i)
    {
        config[0] = traj.xi[0+2*i];
        config[1] = traj.xi[1+2*i];
        
        diff = config - center;
        double norm = diff.norm();

        if(norm >= radius+buffer)
        {
            H.block(0,2*i,1,2).setZero();
        }
        else if(norm >= radius && fabs(buffer) > 1e-10)
        {
            double dist = buffer-(norm-radius);
            H.block(0,2*i,1,2) = diff.transpose()/norm*dist*dist/(2*buffer);
        }
        else if(norm <= 1e-10)
        {
            H.block(0,2*i,1,2).setZero();
        }
        else
        {
            H.block(0,2*i,1,2) = diff.transpose()/norm*(radius-norm + buffer/2);
        }

    }
    
    return true;
}

Constraint::validity_t CircleConstraint::getCost(VectorXd& cost, const Trajectory& traj)
{
    cost.resize(1); // 1 x nm

    cost[0] = 0;

    const VectorXd& xi = traj.xi;
    for(size_t i=0; i<traj.waypoints; ++i)
    {
        double c = _basicCost(Vector2d(xi[2*i],xi[1+2*i]));
        double v = getSpeed(traj, i);

        cost[0] += c*v/traj.waypoints;
    }

    return getValidity(traj);
}

Constraint::validity_t CircleConstraint::getValidity(const Trajectory &traj)
{
    validity_t result = VALID;
    validity_t tempresult = VALID;
    for(size_t i=0; i<traj.waypoints; ++i)
    {
        tempresult = _basicValidity(Vector2d(traj.xi[2*i],traj.xi[1+2*i]));
        if((int)tempresult < (int)result)
            result = tempresult;
    }

    return result;
}

Constraint::validity_t CircleConstraint::_basicValidity(const Eigen::Vector2d &config)
{
    double norm = (config-center).norm();
    if(norm >= radius+buffer)
    {
        return VALID;
    }
    else if(norm >= radius && fabs(buffer) > 1e-10)
    {
        return AT_RISK;
    }
    else
    {
        return INVALID;
    }

    return INVALID;
}

double CircleConstraint::_basicCost(const Eigen::Vector2d& config)
{
    Eigen::Vector2d diff;

    diff = config - center;
    double norm = diff.norm();

    if(norm >= radius+buffer)
    {
        return 0;
    }
    else if(norm >= radius && fabs(buffer) > 1e-10)
    {
        double dist = buffer-(norm-radius);
        return dist*dist/(2*buffer);
    }
    else
    {
        return radius-norm + buffer/2;
    }

    return 0;
}

size_t CircleConstraint::constraintDimension() const
{
    return 1;
}
