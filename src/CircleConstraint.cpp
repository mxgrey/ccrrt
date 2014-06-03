
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

size_t CircleConstraint::getJacobian(MatrixXd& J, const Trajectory& traj)
{
    assert( traj.state_space == 2 );

    J.resize(constraintDimension(), traj.xi.size()); // 1 x nm
    
    Matrix<double,1,2> Jtemp;
    size_t rank_check=0;
    for(size_t i=0; i<traj.waypoints; ++i)
    {
        rank_check += _basicJacobian(Jtemp, traj, i);
        J.block(0,2*i,1,2) = Jtemp;
    }
    
    if(rank_check > 0)
        return 1;

    return 0;
}

size_t CircleConstraint::_basicJacobian(Eigen::Matrix<double,1,2> &J, 
                                        const Trajectory &traj, 
                                        size_t waypoint)
{
    Vector2d config = Vector2d(traj.xi[2*waypoint],traj.xi[2*waypoint+1]);
    
    double c = _basicCost(config);
    if(c == 0)
    {
        J.setZero();
        return 0;
    }
    
    Vector2d grad_c;
    _basicCostGrad(grad_c, config);
    
    VectorXd vel;
    getVelocity(vel, traj, waypoint);
    double vel_norm = vel.norm();
    Vector2d unit_vel = Vector2d(vel[0],vel[1]).normalized();
    
    VectorXd accel;
    getAcceleration(accel, traj, waypoint);
    
    Matrix2d projection = Matrix2d::Identity()-unit_vel*unit_vel.transpose();
    Vector2d k = 1/(vel_norm*vel_norm)*projection*accel;
    
    J = vel_norm*projection*grad_c - c*k;
    
    return 1;
}

Constraint::validity_t CircleConstraint::getCost(VectorXd& cost, 
                                                 const Trajectory& traj)
{
    cost.resize(1); // 1 x 1

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

Constraint::validity_t CircleConstraint::_basicValidity(const Vector2d &config)
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

double CircleConstraint::_basicCost(const Vector2d& config)
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

void CircleConstraint::_basicCostGrad(Eigen::Vector2d& grad_c, const Eigen::Vector2d& config)
{
    grad_c = config - center;
    double norm = grad_c.norm();
    
    if(norm >= radius+buffer)
    {
        grad_c.setZero();
    }
    else if(norm >= radius && fabs(buffer) > 1e-10)
    {
        grad_c = grad_c/norm*(norm-radius-buffer)/buffer;
    }
    else if(norm > 1e-10)
    {
        grad_c = -grad_c/norm;
    }
    else
    {
        grad_c.setZero();
    }
}

size_t CircleConstraint::constraintDimension() const
{
    return 1;
}


