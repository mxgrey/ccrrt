
#include "../ccrrt/BlockConstraint.h"

using namespace ccrrt;
using namespace Eigen;

static inline double sign(double x)
{
    return x == 0? 0 :
            x > 0? 1 : -1;
}

BlockConstraint::BlockConstraint(const Vector2d &mCenter, double mAngle, double mWidth, 
                                 double mHeight, double mBuffer) :
    scales(mWidth/2,mHeight/2),
    buffer(mBuffer)
{
    setLocation(mCenter, mAngle);
}

void BlockConstraint::setLocation(const Vector2d &mCenter, double mAngle)
{
    _tf = Isometry2d::Identity();
    _tf.translate(mCenter);
    _tf.rotate(mAngle);
    
    _tfinv = _tf.inverse();
}

size_t BlockConstraint::getJacobian(MatrixXd &J, const Trajectory &traj)
{
    assert(traj.state_space == 2);
    
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

size_t BlockConstraint::_basicJacobian(Eigen::Matrix<double, 1, 2> &J,
                                       const Trajectory &traj,
                                       size_t waypoint)
{
    Vector2d config = Vector2d(traj.xi[2*waypoint], traj.xi[2*waypoint+1]);
    
    Vector2d grad_c;
    double c = _basicCostGrad(grad_c, config);
    if(c == 0)
    {
        J.setZero();
        return 0;
    }
    
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

Constraint::validity_t BlockConstraint::getCost(VectorXd &cost, const Trajectory &traj)
{
    cost.resize(1);
    
    cost[0] = 0;
    
    const VectorXd& xi = traj.xi;
    Vector2d grad_c;
    for(size_t i=0; i<traj.waypoints; ++i)
    {
        double c = _basicCostGrad(grad_c, Vector2d(xi[2*i],xi[2*i+1]));
        double v = getSpeed(traj, i);
        
        cost[0] += c*v/traj.waypoints;
    }
    
    return getValidity(traj);
}

Constraint::validity_t BlockConstraint::getValidity(const Trajectory &traj)
{
    validity_t result = VALID;
    validity_t tempresult = VALID;
    for(size_t i=0; i<traj.waypoints; ++i)
    {
        tempresult = _basicValidity(Vector2d(traj.xi[2*i], traj.xi[2*i+1]));
        if((int)tempresult < (int)result)
            result = tempresult;
    }
    
    return result;
}

Constraint::validity_t BlockConstraint::_basicValidity(const Vector2d &config)
{
    Vector2d p = _tfinv*(config-_center);
    validity_t result = INVALID;
    for(size_t i=0; i<2; ++i)
    {
        double v = fabs(p[i])-fabs(scales[i]);
        if( v >= buffer )
        {
            return VALID;
        }
        else if( v >= 0 )
        {
            result = AT_RISK;
        }
    }
    
    return result;
}

size_t BlockConstraint::constraintDimension() const
{
    return 1;
}

double BlockConstraint::_basicCostGrad(Eigen::Vector2d& grad_c, const Vector2d &config)
{
    Eigen::Vector2d p = _tfinv*(config-_center);
    
    assert( buffer > 0 && "We only support positive-valued buffer sizes!");
    
    double c = INFINITY;
    for(size_t i=0; i<2; ++i)
    {
        if(fabs(p[i]) >= fabs(scales[i])+fabs(buffer))
        {
            grad_c.setZero();
            return 0;
        }
        else if(fabs(p[i]) >= fabs(scales[i]))
        {
            double dist = buffer-(fabs(p[i])-fabs(scales[i]));
            double ctemp = dist*dist/(2*buffer);
            if(ctemp < c)
            {
                c = ctemp;
                grad_c.setZero();
                grad_c[i] = -(fabs(p[i])-fabs(scales[i])-buffer)/buffer*sign(p[i]);
            }
        }
        else
        {
            double ctemp = scales[i]-p[i] + buffer/2;
            if(ctemp < c)
            {
                c = ctemp;
                grad_c.setZero();
                grad_c[i] = -sign(p[i]);
            }
        }
    }
    
    return c;
}
